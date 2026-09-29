// SPDX-License-Identifier: MIT
//
// Direct Wi-Fi joystick -> Stack-chan -> mBot2 teleop (Stacky teleop v1, UDP).
// Packets: firmware/components/mbot_link/teleop_packet.h.
// Spec: mblock-stacky-bridge/docs/teleop.md.
// Compiled only with CONFIG_STACKCHAN_MBOT_TELEOP.

#pragma once

// Starts the UDP listener task. It waits for Wi-Fi by itself; call once.
void StartMbotTeleop();

// The gateway WebSocket state, reported to the joystick's screen.
void MbotTeleopSetGatewayUp(bool up);
