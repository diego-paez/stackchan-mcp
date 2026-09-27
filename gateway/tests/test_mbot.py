"""Tests for the mBot2 tools (gateway -> self.mbot.* over Stack-chan's BLE link)."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
from mcp.types import CallToolRequest, ListToolsRequest

from stackchan_mcp import esp32_client, mbot
from stackchan_mcp.esp32_client import ESP32Manager, _hardware_lane
from stackchan_mcp.notify_config import DEFAULT_MESSAGE_TEMPLATES, NotifyConfig
import stackchan_mcp.stdio_server as stdio_server
from stackchan_mcp.stdio_server import STACKCHAN_EVENT_METHOD, create_server

MBOT_TOOLS = {
    "mbot_status": "self.mbot.status",
    "mbot_move": "self.mbot.move",
    "mbot_stop": "self.mbot.stop",
    "mbot_set_led": "self.mbot.set_led",
    "mbot_arm": "self.mbot.arm",
    "mbot_gripper": "self.mbot.gripper",
    "mbot_home": "self.mbot.home",
    "mbot_read_sensors": "self.mbot.read_sensors",
    "mbot_run_program": "self.mbot.run_program",
    "mbot_step": "self.mbot.step",
    "mbot_turn": "self.mbot.turn",
    "mbot_odometry": "self.mbot.odometry",
    "mbot_sync": "self.mbot.sync",
}


# ---------------------------------------------------------------- limits


def test_limits_mirror_protocol_table():
    """Same numbers as docs/protocol.md and firmware mbot_limits.h."""
    assert mbot.MBOT_MAX_RPM == 60
    assert mbot.MBOT_MAX_MOVE_S == 5.0
    assert mbot.MBOT_MAX_PROG_S == 30.0
    assert mbot.MBOT_MAX_PROG_STEPS == 20
    assert (mbot.MBOT_ARM_MIN, mbot.MBOT_ARM_MAX, mbot.MBOT_ARM_HOME) == (40, 120, 90)
    assert (mbot.MBOT_GRIP_MIN, mbot.MBOT_GRIP_MAX, mbot.MBOT_GRIP_HOME) == (45, 120, 90)
    assert mbot.MBOT_OBSTACLE_CM == 10
    assert mbot.MBOT_WATCHDOG_S == 3.0
    assert mbot.MBOT_MAX_STEP_CM == 30
    assert mbot.MBOT_MAX_TURN_DEG == 180


def test_firmware_limits_header_matches_gateway():
    """The firmware header carries the same values (single source: protocol.md)."""
    header = (
        Path(__file__).resolve().parents[2]
        / "firmware/components/mbot_link/mbot_limits.h"
    )
    if not header.exists():  # gateway-only checkout (e.g. sdist)
        pytest.skip("firmware tree not present")
    text = header.read_text()
    expected = {
        "MBOT_MAX_RPM": "60",
        "MBOT_MAX_MOVE_S": "5.0",
        "MBOT_MAX_PROG_S": "30.0",
        "MBOT_MAX_PROG_STEPS": "20",
        "MBOT_ARM_MIN": "40",
        "MBOT_ARM_MAX": "120",
        "MBOT_ARM_HOME": "90",
        "MBOT_GRIP_MIN": "45",
        "MBOT_GRIP_MAX": "120",
        "MBOT_GRIP_HOME": "90",
        "MBOT_SERVO_DEG_PER_S": "90",
        "MBOT_OBSTACLE_CM": "10",
        "MBOT_WATCHDOG_S": "3.0",
        "MBOT_HB_PERIOD_S": "1.0",
        "MBOT_MAX_STEP_CM": "30",
        "MBOT_MAX_TURN_DEG": "180",
    }
    for name, value in expected.items():
        assert f"{name} = {value};" in text, name


# ---------------------------------------------------------------- schemas


async def _list_tools():
    server = create_server()
    result = await server.request_handlers[ListToolsRequest](
        ListToolsRequest(method="tools/list")
    )
    return {tool.name: tool for tool in result.root.tools}


@pytest.mark.asyncio
async def test_list_tools_includes_all_mbot_tools():
    tools = await _list_tools()
    for name in MBOT_TOOLS:
        assert name in tools, name


@pytest.mark.asyncio
async def test_mbot_schemas_carry_the_safety_limits():
    tools = await _list_tools()

    move = tools["mbot_move"].inputSchema
    assert move["required"] == ["direction"]
    assert move["properties"]["direction"]["enum"] == ["forward", "backward", "left", "right"]
    assert move["properties"]["speed_percent"]["minimum"] == 0
    assert move["properties"]["speed_percent"]["maximum"] == 100
    assert move["properties"]["seconds"]["minimum"] == 0.1
    assert move["properties"]["seconds"]["maximum"] == 5.0

    arm = tools["mbot_arm"].inputSchema["properties"]
    assert arm["angle"]["minimum"] == 40 and arm["angle"]["maximum"] == 120
    assert arm["position"]["enum"] == ["up", "down", "home"]

    grip = tools["mbot_gripper"].inputSchema["properties"]
    assert grip["angle"]["minimum"] == 45 and grip["angle"]["maximum"] == 120
    assert grip["position"]["enum"] == ["open", "close", "home"]

    led = tools["mbot_set_led"].inputSchema
    for channel in ("r", "g", "b"):
        assert led["properties"][channel]["minimum"] == 0
        assert led["properties"][channel]["maximum"] == 255
    assert led["required"] == ["r", "g", "b"]

    prog = tools["mbot_run_program"].inputSchema["properties"]["steps"]
    assert prog["maxLength"] == mbot.MBOT_MAX_PROGRAM_CHARS
    assert "20 steps" in tools["mbot_run_program"].description
    assert "30 s" in tools["mbot_run_program"].description

    step = tools["mbot_step"].inputSchema
    assert step["required"] == ["distance_cm"]
    assert step["properties"]["distance_cm"]["minimum"] == -30
    assert step["properties"]["distance_cm"]["maximum"] == 30
    assert step["properties"]["speed_percent"]["minimum"] == 5
    assert step["properties"]["speed_percent"]["default"] == 30
    turn = tools["mbot_turn"].inputSchema
    assert turn["properties"]["degrees"]["minimum"] == -180
    assert turn["properties"]["degrees"]["maximum"] == 180
    assert "positive = left" in tools["mbot_turn"].description
    for name in ("mbot_step", "mbot_turn"):
        assert "stop -> look -> decide -> step" in tools[name].description
        assert "done" in tools[name].description
    odom = tools["mbot_odometry"].inputSchema["properties"]
    assert odom["stream_ms"]["maximum"] == 5000
    assert "straight" in tools["mbot_run_program"].description

    # The LLM must learn that stop always works.
    assert "always" in tools["mbot_stop"].description
    for name in ("mbot_move", "mbot_run_program", "mbot_status"):
        assert "mbot_stop is always available" in tools[name].description


# ---------------------------------------------------------------- mapping


class _FakeESP32:
    device_connected = True

    def __init__(self):
        self.calls: list[tuple[str, dict]] = []

    async def call_tool(self, name, arguments):
        self.calls.append((name, arguments))
        return {"content": [{"type": "text", "text": json.dumps({"ok": True})}]}, None


@pytest.fixture
def fake_esp32(monkeypatch):
    esp32 = _FakeESP32()

    class FakeGateway:
        pass

    FakeGateway.esp32 = esp32
    monkeypatch.setattr(stdio_server, "get_gateway", lambda: FakeGateway())
    return esp32


async def _call_text(name, arguments):
    server = create_server()
    result = await server.request_handlers[CallToolRequest](
        CallToolRequest(method="tools/call", params={"name": name, "arguments": arguments})
    )
    return result.root.content[0].text


async def _call(name, arguments):
    return json.loads(await _call_text(name, arguments))


@pytest.mark.parametrize(
    ("tool", "arguments", "device_args"),
    [
        ("mbot_status", {}, {}),
        ("mbot_stop", {}, {}),
        ("mbot_home", {}, {}),
        ("mbot_read_sensors", {}, {}),
        (
            "mbot_move",
            {"direction": "forward"},
            {"direction": "forward", "speed_percent": 40, "duration_ms": 1000},
        ),
        (
            "mbot_move",
            {"direction": "left", "speed_percent": 70, "seconds": 2.5},
            {"direction": "left", "speed_percent": 70, "duration_ms": 2500},
        ),
        (
            "mbot_move",
            {"direction": "backward", "speed_percent": 0, "seconds": 0.1},
            {"direction": "backward", "speed_percent": 0, "duration_ms": 100},
        ),
        ("mbot_set_led", {"r": 0, "g": 255, "b": 0}, {"r": 0, "g": 255, "b": 0, "index": "all"}),
        ("mbot_set_led", {"r": 1, "g": 2, "b": 3, "index": 5}, {"r": 1, "g": 2, "b": 3, "index": "5"}),
        ("mbot_arm", {"position": "up"}, {"position": "up"}),
        ("mbot_arm", {"angle": 40}, {"angle": 40}),
        ("mbot_gripper", {"position": "close"}, {"position": "close"}),
        ("mbot_gripper", {"angle": 120}, {"angle": 120}),
        ("mbot_sync", {}, {}),
        ("mbot_step", {"distance_cm": 20}, {"distance_cm": 20, "speed_percent": 30}),
        (
            "mbot_step",
            {"distance_cm": -30, "speed_percent": 5},
            {"distance_cm": -30, "speed_percent": 5},
        ),
        ("mbot_turn", {"degrees": 90}, {"degrees": 90, "speed_percent": 30}),
        ("mbot_turn", {"degrees": -180, "speed_percent": 100}, {"degrees": -180, "speed_percent": 100}),
        ("mbot_odometry", {}, {}),
        ("mbot_odometry", {"done_id": 42}, {"done_id": 42}),
        ("mbot_odometry", {"stream_ms": 0}, {"stream_ms": 0}),
        ("mbot_odometry", {"stream_ms": 500}, {"stream_ms": 500}),
        (
            "mbot_run_program",
            {"steps": "fwd 40 1; led 0 255 0; wait 0.5; arm up"},
            {"steps": "fwd 40 1; led 0 255 0; wait 0.5; arm up"},
        ),
    ],
)
@pytest.mark.asyncio
async def test_mbot_tools_map_to_device_tools(fake_esp32, tool, arguments, device_args):
    payload = await _call(tool, arguments)
    assert payload == {"ok": True}
    assert fake_esp32.calls == [(MBOT_TOOLS[tool], device_args)]


_BAD_ARGUMENTS = pytest.mark.parametrize(
    ("tool", "arguments", "fragment"),
    [
        ("mbot_move", {}, "direction"),
        ("mbot_move", {"direction": "up"}, "direction"),
        ("mbot_move", {"direction": "forward", "speed_percent": 101}, "speed_percent"),
        ("mbot_move", {"direction": "forward", "speed_percent": -1}, "speed_percent"),
        ("mbot_move", {"direction": "forward", "speed_percent": True}, "speed_percent"),
        ("mbot_move", {"direction": "forward", "seconds": 6}, "seconds"),
        ("mbot_move", {"direction": "forward", "seconds": 0.05}, "seconds"),
        ("mbot_move", {"direction": "forward", "seconds": float("inf")}, "seconds"),
        ("mbot_set_led", {"r": 256, "g": 0, "b": 0}, "r"),
        ("mbot_set_led", {"r": 0, "g": 0}, "b"),
        ("mbot_set_led", {"r": 0, "g": 0, "b": 0, "index": 6}, "index"),
        ("mbot_arm", {"angle": 130}, "angle"),
        ("mbot_arm", {"angle": 39}, "angle"),
        ("mbot_arm", {"position": "open"}, "position"),
        ("mbot_arm", {"position": "up", "angle": 90}, "not both"),
        ("mbot_arm", {}, "position"),
        ("mbot_gripper", {"angle": 44}, "angle"),
        ("mbot_gripper", {"position": "up"}, "position"),
        ("mbot_run_program", {"steps": ""}, "steps"),
        ("mbot_run_program", {"steps": ";".join(["led 0 0 0"] * 21)}, "21 steps"),
        ("mbot_run_program", {"steps": "wait 1;" * 100}, "too long"),
        ("mbot_step", {}, "distance_cm"),
        ("mbot_step", {"distance_cm": 31}, "distance_cm"),
        ("mbot_step", {"distance_cm": -31}, "distance_cm"),
        ("mbot_step", {"distance_cm": 10, "speed_percent": 4}, "speed_percent"),
        ("mbot_step", {"distance_cm": 10.5}, "distance_cm"),
        ("mbot_turn", {"degrees": 181}, "degrees"),
        ("mbot_turn", {"degrees": 90, "speed_percent": 101}, "speed_percent"),
        ("mbot_odometry", {"stream_ms": 50}, "stream_ms"),
        ("mbot_odometry", {"stream_ms": 5001}, "stream_ms"),
        ("mbot_odometry", {"done_id": 0}, "done_id"),
    ],
)


@_BAD_ARGUMENTS
def test_gateway_rejects_out_of_limit_arguments(tool, arguments, fragment):
    """The gateway's own check, independent of the MCP SDK's schema check."""
    with pytest.raises(ValueError, match=fragment):
        mbot.map_mbot_call(tool, arguments)


@_BAD_ARGUMENTS
@pytest.mark.asyncio
async def test_mbot_out_of_limit_arguments_never_reach_the_device(
    fake_esp32, tool, arguments, fragment
):
    text = await _call_text(tool, arguments)
    # Either the MCP SDK's JSON-schema validation or the gateway's own check
    # refuses it; in both cases nothing is sent to the robot.
    if text.startswith("Input validation error"):
        pass
    else:
        payload = json.loads(text)
        assert payload["ok"] is False
        assert fragment in payload["error"]
    assert fake_esp32.calls == []


# ---------------------------------------------------------------- lanes


def test_mbot_lanes():
    assert _hardware_lane("self.mbot.move") == "mbot"
    assert _hardware_lane("self.mbot.run_program") == "mbot"
    assert _hardware_lane("self.mbot.stop") == "mbot_stop"
    assert _hardware_lane("self.mbot.step") == "mbot"
    assert _hardware_lane("self.mbot.turn") == "mbot"
    assert _hardware_lane("self.mbot.odometry") == "mbot"
    assert _hardware_lane("self.mbot.sync") == "mbot"


class _GateableConnection:
    connected = True
    initialized = True

    def __init__(self, releases):
        self.releases = releases
        self.started: list[str] = []

    async def call_tool(self, name, arguments):  # noqa: ARG002 - test fake
        self.started.append(name)
        await self.releases[name].wait()
        return {"content": [{"type": "text", "text": name}]}, None


@pytest.mark.asyncio
async def test_mbot_stop_is_not_queued_behind_a_move():
    """A stop goes out while a move is still waiting for its reply."""
    releases = {"self.mbot.move": asyncio.Event(), "self.mbot.stop": asyncio.Event()}
    connection = _GateableConnection(releases)
    mgr = ESP32Manager()
    mgr._connection = connection  # type: ignore[assignment]

    move = asyncio.create_task(mgr.call_tool("self.mbot.move", {"direction": "forward"}))
    await asyncio.sleep(0)
    stop = asyncio.create_task(mgr.call_tool("self.mbot.stop", {}))
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    assert connection.started == ["self.mbot.move", "self.mbot.stop"]

    releases["self.mbot.stop"].set()
    await asyncio.wait_for(stop, timeout=1.0)
    assert not move.done()
    releases["self.mbot.move"].set()
    await asyncio.wait_for(move, timeout=1.0)


@pytest.mark.asyncio
async def test_mbot_moves_share_one_lane():
    """Two moves stay in order (the firmware also sends one command at a time)."""
    releases = {"self.mbot.move": asyncio.Event(), "self.mbot.set_led": asyncio.Event()}
    connection = _GateableConnection(releases)
    mgr = ESP32Manager()
    mgr._connection = connection  # type: ignore[assignment]

    task = asyncio.create_task(
        mgr.call_tools([("self.mbot.move", {}), ("self.mbot.set_led", {})])
    )
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    assert connection.started == ["self.mbot.move"]
    releases["self.mbot.move"].set()
    releases["self.mbot.set_led"].set()
    await asyncio.wait_for(task, timeout=1.0)
    assert connection.started == ["self.mbot.move", "self.mbot.set_led"]


# ---------------------------------------------------------------- events


def _notify_config(**overrides) -> NotifyConfig:
    values = {
        "legacy_event_enabled": True,
        "channels_enabled": False,
        "jsonl_enabled": False,
        "jsonl_path": Path("/tmp/stackchan-events-test.jsonl"),
        "messages": dict(DEFAULT_MESSAGE_TEMPLATES),
    }
    values.update(overrides)
    return NotifyConfig(**values)


def _mbot_frame(subtype, detail=None):
    frame = {
        "session_id": "session-1",
        "type": "stackchan-event",
        "event_type": "mbot",
        "subtype": subtype,
        "duration_ms": 0,
        "ts": 4242,
    }
    if detail is not None:
        frame["detail"] = detail
    return frame


@pytest.mark.parametrize("subtype", sorted(mbot.MBOT_EVENT_SUBTYPES))
def test_every_mbot_event_has_a_default_template(subtype):
    assert ("mbot", subtype) in DEFAULT_MESSAGE_TEMPLATES


@pytest.mark.asyncio
async def test_mbot_event_passes_through_with_detail(monkeypatch):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())

    await manager._emit_stackchan_event(_mbot_frame("obstacle", "7"))

    assert calls == [
        (
            STACKCHAN_EVENT_METHOD,
            {
                "event_type": "mbot",
                "subtype": "obstacle",
                "duration_ms": 0,
                "action": "mbot_obstacle",
                "ts": 4242,
                "session_id": "session-1",
                "detail": "7",
            },
        )
    ]


@pytest.mark.asyncio
async def test_mbot_event_channel_content_renders_detail(monkeypatch):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    monkeypatch.setattr(esp32_client.time, "time", lambda: 1717000000.0)
    manager = ESP32Manager(
        notify_config=_notify_config(legacy_event_enabled=False, channels_enabled=True)
    )

    await manager._emit_stackchan_event(_mbot_frame("stopped", "watchdog"))

    assert len(calls) == 1
    method, params = calls[0]
    assert method == "notifications/claude/channel"
    assert params["content"] == "the mBot stopped (watchdog)"
    assert params["meta"]["event_type"] == "mbot"
    assert params["meta"]["detail"] == "watchdog"


@pytest.mark.asyncio
async def test_mbot_event_without_detail_omits_it(monkeypatch):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())

    await manager._emit_stackchan_event(_mbot_frame("locked"))

    assert len(calls) == 1
    assert "detail" not in calls[0][1]
    assert calls[0][1]["action"] == "mbot_locked"


@pytest.mark.asyncio
async def test_mbot_event_is_logged_to_jsonl_with_detail(monkeypatch, tmp_path):
    monkeypatch.delenv("STACKCHAN_EVENTS_PATH", raising=False)
    log_path = tmp_path / "events.jsonl"
    manager = ESP32Manager(
        notify_config=_notify_config(
            legacy_event_enabled=False, jsonl_enabled=True, jsonl_path=log_path
        )
    )

    await manager._emit_stackchan_event(_mbot_frame("done", "123"))

    line = json.loads(log_path.read_text().strip())
    assert line["event_type"] == "mbot"
    assert line["subtype"] == "done"
    assert line["detail"] == "123"


@pytest.mark.parametrize(
    ("overrides", "warning"),
    [
        ({"subtype": "tap"}, "subtype='tap'"),
        ({"subtype": "hb"}, "subtype='hb'"),
        ({"detail": 7}, "detail=7"),
        ({"detail": "x" * 200}, "detail="),
    ],
)
@pytest.mark.asyncio
async def test_malformed_mbot_event_is_dropped(monkeypatch, caplog, overrides, warning):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())
    frame = _mbot_frame("obstacle", "7")
    frame.update(overrides)

    with caplog.at_level("WARNING"):
        await manager._emit_stackchan_event(frame)

    assert calls == []
    assert f"Malformed stackchan-event frame: {warning}" in caplog.text


@pytest.mark.asyncio
async def test_touch_events_still_reject_mbot_subtypes(monkeypatch, caplog):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())
    frame = _mbot_frame("obstacle")
    frame["event_type"] = "touch"

    with caplog.at_level("WARNING"):
        await manager._emit_stackchan_event(frame)

    assert calls == []


@pytest.mark.asyncio
async def test_mbot_v11_done_event_passes_through_with_detail(monkeypatch):
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())
    detail = "42 target dist=29.8 yaw=-0.4 t=123456"

    await manager._emit_stackchan_event(_mbot_frame("done", detail))

    assert len(calls) == 1
    assert calls[0][1]["subtype"] == "done"
    assert calls[0][1]["detail"] == detail


@pytest.mark.asyncio
async def test_mbot_odom_is_not_an_event(monkeypatch, caplog):
    """odom streams are cached on the device, never forwarded as events."""
    calls = []

    async def fake_notify(method, params):
        calls.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    manager = ESP32Manager(notify_config=_notify_config())

    await manager._emit_stackchan_event(_mbot_frame("odom", "t=1 l=0.0 r=0.0 yaw=0.0"))

    assert calls == []
