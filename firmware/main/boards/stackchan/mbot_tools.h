// SPDX-License-Identifier: MIT
//
// self.mbot.* MCP tools: drive a Makeblock mBot2 over the BLE link in
// firmware/components/mbot_link/. Compiled only with
// CONFIG_STACKCHAN_MBOT_LINK (see main/Kconfig.projbuild).

#pragma once

#include <functional>

class McpServer;

struct MbotBoardHooks {
    // Show a short reaction face (runs on the main task). May be empty.
    std::function<void(const char* face)> show_reaction_face;
    // Reset the power-save timer (runs on the main task). Called every second
    // while the mBot is connected so Stack-chan does not sleep or power off
    // in the middle of a session. May be empty.
    std::function<void()> keep_awake;
};

// Registers the tools, wires mBot events to the gateway (stackchan-event,
// event_type "mbot") and the local reflexes, and starts the BLE link.
void RegisterMbotTools(McpServer& mcp_server, MbotBoardHooks hooks);
