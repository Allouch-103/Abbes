"""
llm_dispatcher_node.py
======================

The robot-facing half of the dispatcher (AI Step 4, Piece 2).

WHAT THIS FILE IS (and what it is NOT, yet)
-------------------------------------------
This node is the MIRROR IMAGE of `command_server_node.py`:

    command_server_node.py  hosts  5 action servers + 2 service servers
    llm_dispatcher_node.py  holds  5 action clients + 2 service clients

That's all Piece 2 is: the plumbing that lets *Python* drive the robot's
command API. There is NO Ollama / LLM here yet — that is Piece 3. Building
the client half on its own means we can answer one question in isolation:

    "Can we call every action and service and get a clean result back,
     including when the server REJECTS a goal?"

Once that's solid, Piece 3 just feeds tool names + args (chosen by the
model) into the single `execute_tool()` entry point below, and relays the
returned dict back to the model.

THE ONE SEAM PIECE 3 PLUGS INTO
-------------------------------
    execute_tool(name: str, args: dict) -> dict

`name` is one of `VALID_TOOL_NAMES` (from command_tools.py). The method
routes to the right client and always returns a uniform dict:

    {"ok": bool, "message": str, ...extra fields...}

so the LLM layer never has to know whether a command was an action or a
service.

WHY SYNCHRONOUS (spin_until_future_complete) AND NOT A BACKGROUND EXECUTOR
-------------------------------------------------------------------------
The dispatcher runs ONE command at a time, in order: the model picks a
tool, we run it to completion, we report back, the model picks the next.
That is naturally sequential, so each helper simply BLOCKS until the goal
finishes using `rclpy.spin_until_future_complete`. No threads, no executor
juggling — the simplest thing that is correct for a one-at-a-time loop.
(The command SERVER is the multi-threaded side; the client can stay simple.)
"""

import sys
import json

import requests

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from humanoid_msgs.action import Walk, StandStill, SitDown, WaveHand, Bow
from humanoid_msgs.srv import Stop, GetStatus

from .command_tools import VALID_TOOL_NAMES, ROBOT_TOOLS
from .safety_validator import SafetyValidator


# How long we wait for a server/service to be available before giving up.
# The command server should already be running; if it isn't, we want a
# clear error instead of hanging forever.
SERVER_WAIT_TIMEOUT_SEC = 5.0


# ── Ollama (the LLM brain) ───────────────────────────────────
# The dispatcher is BOTH a ROS2 client (the action/service clients above)
# and an Ollama tool-use client (the dispatch loop below). It is the same
# local server, model, and 4-phase tool-use loop proven in
# ai_experiments/02_weather_tool.py — the ONLY difference is that the tools
# are the robot's real command vocabulary (ROBOT_TOOLS) and running a tool
# actually moves the robot (via self.execute_tool).
OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
MODEL = "llama3.2:3b"

# The system prompt is the model's "job description" — the one place we tell
# it WHO it is and HOW to behave. Kept short and concrete because a 3B model
# follows tight instructions far better than long essays.
ROBOT_SYSTEM_PROMPT = (
    "You are the control brain of a real humanoid robot. "
    "When the user asks for something physical, do NOT describe it — CALL "
    "ONE of the provided tools to make the robot actually move, choosing "
    "the single tool that best fits and filling in its arguments. "
    "If the user is only chatting or asking something that needs no "
    "movement, reply in one short sentence and call no tool. "
    "Never invent tools or arguments that were not provided. "
    "Safety first: if the user sounds urgent or says stop/halt/freeze, "
    "call the 'stop' tool."
)


class LLMDispatcherNode(Node):

    def __init__(self):
        super().__init__("llm_dispatcher")

        # ── Action clients ───────────────────────────────────
        # One per goal-driven motion. The string MUST match the action
        # name the server advertised in command_server_node.py.
        self.walk_client  = ActionClient(self, Walk,       "/humanoid/walk")
        self.stand_client = ActionClient(self, StandStill, "/humanoid/stand_still")
        self.sit_client   = ActionClient(self, SitDown,    "/humanoid/sit_down")
        self.wave_client  = ActionClient(self, WaveHand,   "/humanoid/wave_hand")
        self.bow_client   = ActionClient(self, Bow,        "/humanoid/bow")

        # ── Service clients ──────────────────────────────────
        self.stop_client   = self.create_client(Stop,      "/humanoid/stop")
        self.status_client = self.create_client(GetStatus, "/humanoid/get_status")

        # ── Safety validator (AI Step 5) ─────────────────────
        # Consulted between the model's tool choice and execute_tool().
        # Stateful (holds the rate-limit clock), so it lives for the
        # lifetime of the node, not per-dispatch.
        self.validator = SafetyValidator()

        self.get_logger().info("dispatcher clients ready: 5 actions + 2 services")

    # ─────────────────────────────────────────────────────────
    #  Generic action helper
    #
    #  Sends one goal and BLOCKS until the server returns a result
    #  (or rejects the goal). Returns a uniform dict so callers never
    #  touch rclpy futures directly.
    #
    #  result_fields: names to copy out of the action Result message
    #  into the returned dict (e.g. ["success", "message"]).
    # ─────────────────────────────────────────────────────────
    def _run_action(self, client: ActionClient, goal_msg, action_name: str,
                    result_fields: list) -> dict:
        # 1. Make sure the server is actually there.
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
            return {"ok": False,
                    "message": f"{action_name}: server not available "
                               f"(is command_server_node running?)"}

        # 2. Send the goal. A feedback callback lets us watch progress —
        #    handy while testing, and the same hook Piece 3 can surface.
        def _on_feedback(feedback_msg):
            fb = feedback_msg.feedback
            progress = getattr(fb, "progress", None)
            if progress is not None:
                self.get_logger().info(
                    f"{action_name} feedback: {progress * 100:.0f}%")

        send_future = client.send_goal_async(goal_msg, feedback_callback=_on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        # 3. The server's goal_callback may REJECT (another action active,
        #    or args out of range). That's a normal outcome, not a crash.
        if goal_handle is None or not goal_handle.accepted:
            return {"ok": False,
                    "message": f"{action_name}: goal REJECTED by server "
                               f"(another action may be active, or args invalid)."}

        # 4. Wait for the action to finish and pull out the result.
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        result = wrapped.result

        out = {"ok": bool(getattr(result, "success", False))}
        for field in result_fields:
            out[field] = getattr(result, field, None)
        # Promote the action's own message to the top-level message if present.
        if "message" in out and out["message"]:
            out["message"] = str(out["message"])
        return out

    # ─────────────────────────────────────────────────────────
    #  Generic service helper
    # ─────────────────────────────────────────────────────────
    def _call_service(self, client, request, service_name: str,
                      response_fields: list) -> dict:
        if not client.wait_for_service(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
            return {"ok": False,
                    "message": f"{service_name}: service not available "
                               f"(is command_server_node running?)"}

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None:
            return {"ok": False, "message": f"{service_name}: no response."}

        # Stop uses .success; GetStatus uses .ok. Normalize to "ok".
        ok = getattr(response, "success", None)
        if ok is None:
            ok = getattr(response, "ok", False)
        out = {"ok": bool(ok)}
        for field in response_fields:
            out[field] = getattr(response, field, None)
        if "message" in out and out["message"]:
            out["message"] = str(out["message"])
        return out

    # ─────────────────────────────────────────────────────────
    #  Per-command wrappers — one per tool in command_tools.py.
    #  Each builds the goal/request and delegates to the helpers.
    # ─────────────────────────────────────────────────────────
    def do_walk(self, distance_m: float, direction: str) -> dict:
        goal = Walk.Goal()
        goal.distance_m = float(distance_m)
        goal.direction = str(direction)
        return self._run_action(
            self.walk_client, goal, "walk",
            result_fields=["distance_actual_m", "message"])

    def do_stand_still(self) -> dict:
        return self._run_action(
            self.stand_client, StandStill.Goal(), "stand_still",
            result_fields=["message"])

    def do_sit_down(self) -> dict:
        return self._run_action(
            self.sit_client, SitDown.Goal(), "sit_down",
            result_fields=["message"])

    def do_wave_hand(self, which: str) -> dict:
        goal = WaveHand.Goal()
        goal.which = str(which)
        return self._run_action(
            self.wave_client, goal, "wave_hand",
            result_fields=["message"])

    def do_bow(self) -> dict:
        return self._run_action(
            self.bow_client, Bow.Goal(), "bow",
            result_fields=["message"])

    def do_stop(self) -> dict:
        return self._call_service(
            self.stop_client, Stop.Request(), "stop",
            response_fields=["message"])

    def do_get_status(self) -> dict:
        return self._call_service(
            self.status_client, GetStatus.Request(), "get_status",
            response_fields=["current_action", "tilt_pitch_deg",
                             "tilt_roll_deg", "message"])

    # ─────────────────────────────────────────────────────────
    #  THE SEAM: name + args  ->  the right call  ->  uniform dict.
    #
    #  Piece 3 (Ollama) calls ONLY this. It guards against a model
    #  hallucinating a tool that doesn't exist, and against missing
    #  required args, so a bad LLM output becomes a clean error dict
    #  instead of an exception.
    # ─────────────────────────────────────────────────────────
    def execute_tool(self, name: str, args: dict) -> dict:
        args = args or {}

        if name not in VALID_TOOL_NAMES:
            return {"ok": False,
                    "message": f"unknown tool {name!r} "
                               f"(valid: {sorted(VALID_TOOL_NAMES)})"}

        try:
            if name == "walk":
                if "distance_m" not in args or "direction" not in args:
                    return {"ok": False,
                            "message": "walk requires 'distance_m' and 'direction'."}
                return self.do_walk(args["distance_m"], args["direction"])

            if name == "wave_hand":
                if "which" not in args:
                    return {"ok": False, "message": "wave_hand requires 'which'."}
                return self.do_wave_hand(args["which"])

            if name == "stand_still":
                return self.do_stand_still()
            if name == "sit_down":
                return self.do_sit_down()
            if name == "bow":
                return self.do_bow()
            if name == "stop":
                return self.do_stop()
            if name == "get_status":
                return self.do_get_status()

        except Exception as exc:  # last-resort guard so the loop never dies
            return {"ok": False, "message": f"{name} raised: {exc!r}"}

        # Should be unreachable given the VALID_TOOL_NAMES check above.
        return {"ok": False, "message": f"tool {name!r} not wired."}

    # ─────────────────────────────────────────────────────────
    #  THE LLM BRAIN (Piece 3)
    #
    #  Natural language in -> robot motion + spoken-style reply out.
    #  Same 4-phase tool-use loop as ai_experiments/02_weather_tool.py;
    #  the difference is Phase 3 runs the chosen tool ON THE ROBOT via
    #  execute_tool() instead of a fake weather lookup.
    # ─────────────────────────────────────────────────────────
    def _call_ollama(self, messages: list) -> dict:
        """POST the running transcript + robot tools to Ollama and return
        the assistant message object (which may carry 'tool_calls')."""
        payload = {
            "model": MODEL,
            "messages": messages,
            "tools": ROBOT_TOOLS,   # <-- this is what enables tool calling
            "stream": False,
        }
        resp = requests.post(OLLAMA_URL, json=payload, timeout=120)
        resp.raise_for_status()
        return resp.json()["message"]

    def dispatch(self, user_text: str) -> str:
        """Run one full natural-language → action → reply cycle.

        Returns the model's final natural-language reply (a string the
        caller can print or, later, speak).
        """
        # The transcript. We APPEND to it as the loop runs so the model
        # always sees the full context: its own tool request, then the
        # real result the robot produced.
        messages = [
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": user_text},
        ]

        # ── Phase 1 & 2: ask the model; it chats OR requests tool(s) ──
        try:
            assistant_msg = self._call_ollama(messages)
        except requests.exceptions.ConnectionError:
            return "ERROR: Ollama not reachable (check: systemctl status ollama)."
        except requests.exceptions.HTTPError as e:
            return f"ERROR from Ollama: {e} (is the model {MODEL} pulled?)"
        messages.append(assistant_msg)

        tool_calls = assistant_msg.get("tool_calls")
        if not tool_calls:
            # No movement needed — the model just talked.
            return assistant_msg.get("content", "")

        # ── Phase 3: run each requested command ON THE ROBOT ──────────
        for call in tool_calls:
            name = call["function"]["name"]
            args = call["function"].get("arguments", {}) or {}
            self.get_logger().info(f"LLM -> {name}({args})")

            # AI Step 5: gate the chosen tool BEFORE it runs. On deny we
            # skip execute_tool entirely and report the reason as the tool
            # result, so the model explains the refusal instead of moving.
            allowed, reason = self.validator.check(name, args, self.do_get_status)
            if not allowed:
                self.get_logger().warn(f"   SAFETY denied {name}: {reason}")
                result = {"ok": False, "denied_by_safety": True, "message": reason}
            else:
                result = self.execute_tool(name, args)   # the Piece 2 seam
            self.get_logger().info(f"   robot result: {result}")
            # Feed the result back with role="tool" so the model sees what
            # its requested action actually produced.
            messages.append({
                "role": "tool",
                "tool_name": name,
                "content": json.dumps(result),
            })

        # ── Phase 4: results back in; model writes the human reply ────
        try:
            final_msg = self._call_ollama(messages)
        except requests.exceptions.RequestException as e:
            return f"(action ran, but the follow-up reply failed: {e})"
        return final_msg.get("content", "")


def _run_smoke_test(node: "LLMDispatcherNode") -> None:
    """Piece-2 smoke test (no Ollama): drive each tool once and print the
    result dict, to confirm the ROS2 client half talks to the server."""
    sequence = [
        ("get_status", {}),
        ("wave_hand", {"which": "right"}),
        ("bow", {}),
        ("walk", {"distance_m": 0.5, "direction": "forward"}),
        ("stand_still", {}),
        ("get_status", {}),
    ]
    for name, args in sequence:
        node.get_logger().info(f"--- execute_tool({name!r}, {args}) ---")
        node.get_logger().info(f"    result: {node.execute_tool(name, args)}")


def main():
    """Interactive natural-language control of the robot (Piece 3).

    Prerequisites: Ollama running with llama3.2:3b pulled, AND the command
    server up:  ros2 run humanoid_command_api command_server
    Then:       ros2 run humanoid_command_api llm_dispatcher

    Type plain English ("wave your right hand", "walk forward half a meter",
    "stop", "what are you doing?"). 'quit'/'exit'/Ctrl-D to leave.

    With  --test  it runs the no-LLM Piece-2 smoke test instead, to check
    the ROS2 client half in isolation.
    """
    rclpy.init()
    node = LLMDispatcherNode()
    try:
        if "--test" in sys.argv:
            _run_smoke_test(node)
            return

        print("Robot dispatcher ready. Type a command in plain English "
              "('quit' to exit).")
        while True:
            try:
                user_text = input("you> ").strip()
            except EOFError:
                break
            if not user_text:
                continue
            if user_text.lower() in ("quit", "exit"):
                break
            reply = node.dispatch(user_text)
            print(f"robot> {reply}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
