#pragma once
// ============================================================
// microros_transport.h — micro-ROS lifecycle management
// ============================================================

#include <Arduino.h>

// ── Agent state (visible to main.cpp for status printing) ──
enum class AgentState : uint8_t {
    WAITING_WIFI   = 0,
    WAITING_AGENT  = 1,
    CONNECTED      = 2,
    DISCONNECTED   = 3,
};

extern volatile AgentState agent_state;
extern TaskHandle_t microros_task_handle;

// ── Public API ─────────────────────────────────────────────
void microros_start_task();