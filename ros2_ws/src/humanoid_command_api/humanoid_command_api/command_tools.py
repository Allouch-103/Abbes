"""
command_tools.py
================

The LLM's "menu": the 7 robot commands described as Ollama tool-use
(function-calling) schemas.

WHY THIS FILE EXISTS (and why it's separate)
--------------------------------------------
This is the single source of truth for *what the LLM is allowed to ask
for*. It is the exact analogue of PART B in
`~/Abbes/ai_experiments/02_weather_tool.py` — but instead of one fake
`get_weather` tool, it lists the robot's real 7-command vocabulary.

It is deliberately PURE DATA (no rclpy, no requests). That means:
  - the LLM dispatcher imports it to tell the model what's available,
  - and it can be unit-tested / eyeballed in isolation,
  - and nothing here can move a motor. The model only ever *names* a
    tool and *fills in arguments*; the dispatcher (next piece) is the
    only thing that turns a name into a real ROS2 action/service call.

KEEP IN SYNC WITH THE COMMAND SERVER
------------------------------------
Every tool below maps 1:1 to an action/service hosted by
`command_server_node.py`. The argument names, value ranges, and string
enums here MUST match the goal/request fields in `humanoid_msgs`:

    walk         action  /humanoid/walk         distance_m (0.1–3.0), direction ("forward"|"backward")
    stand_still  action  /humanoid/stand_still  (no args)
    sit_down     action  /humanoid/sit_down     (no args)
    wave_hand    action  /humanoid/wave_hand    which ("left"|"right")
    bow          action  /humanoid/bow          (no args)
    stop         service /humanoid/stop         (no args)
    get_status   service /humanoid/get_status   (no args)

The descriptions are written for a small (3B) model: short, concrete,
and explicit about when to choose each tool, because tool-selection
quality on a small model lives or dies by the description text.
"""


# ─────────────────────────────────────────────────────────────
#  The tool schemas, in Ollama's expected shape:
#    {"type": "function",
#     "function": {"name", "description", "parameters": <JSON-Schema>}}
#
#  An empty-parameters tool still needs a valid (empty) object schema.
# ─────────────────────────────────────────────────────────────

NO_ARGS = {"type": "object", "properties": {}, "required": []}


ROBOT_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "walk",
            "description": (
                "Make the robot walk a set distance forwards or backwards. "
                "Use this when the user asks the robot to move, go, step, or "
                "walk somewhere. Pick a distance in meters between 0.1 and 3.0."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "distance_m": {
                        "type": "number",
                        "description": (
                            "How far to walk, in meters. Must be between "
                            "0.1 and 3.0. If the user is vague (e.g. 'come "
                            "here', 'take a step'), choose a small value like 0.5."
                        ),
                    },
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward"],
                        "description": (
                            "Direction of travel: 'forward' (default for "
                            "'come', 'go', 'walk') or 'backward' ('step back', "
                            "'move away')."
                        ),
                    },
                },
                "required": ["distance_m", "direction"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stand_still",
            "description": (
                "Return the robot to its neutral standing (home) pose: "
                "straight legs, arms down, head forward. Use this for "
                "'stand up', 'stand', 'reset', 'go neutral', or to recover "
                "from any other pose. This is NOT an emergency stop — use "
                "'stop' for that."
            ),
            "parameters": NO_ARGS,
        },
    },
    {
        "type": "function",
        "function": {
            "name": "sit_down",
            "description": (
                "Lower the robot into a sitting/rest pose by bending the "
                "knees and hips. Use for 'sit', 'sit down', 'take a seat', "
                "'rest'."
            ),
            "parameters": NO_ARGS,
        },
    },
    {
        "type": "function",
        "function": {
            "name": "wave_hand",
            "description": (
                "Wave one hand as a greeting. Use for 'wave', 'say hi', "
                "'greet', 'hello'. The other arm stays at the side."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "which": {
                        "type": "string",
                        "enum": ["left", "right"],
                        "description": (
                            "Which hand to wave with. If the user does not "
                            "specify, default to 'right'."
                        ),
                    },
                },
                "required": ["which"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "bow",
            "description": (
                "Make the robot bow by leaning the torso forward at the "
                "hips. Use for 'bow', 'take a bow', 'greeting bow'."
            ),
            "parameters": NO_ARGS,
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop",
            "description": (
                "EMERGENCY STOP. Immediately cancel whatever the robot is "
                "doing and snap back to the safe home pose. Use for 'stop', "
                "'halt', 'freeze', 'cancel', 'abort', or any sign of danger. "
                "Prefer this over 'stand_still' when the user sounds urgent."
            ),
            "parameters": NO_ARGS,
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_status",
            "description": (
                "Report the robot's current state: what action it is doing, "
                "its tilt (pitch/roll in degrees), and joint positions. Use "
                "for 'what are you doing', 'status', 'are you ok', 'how do "
                "you feel', 'are you tilted'. This only READS state; it does "
                "not move the robot."
            ),
            "parameters": NO_ARGS,
        },
    },
]


# Convenience: the set of valid tool names, so the dispatcher can quickly
# reject a hallucinated tool the model might invent that isn't real.
VALID_TOOL_NAMES = {t["function"]["name"] for t in ROBOT_TOOLS}
