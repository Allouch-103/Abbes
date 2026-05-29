#!/usr/bin/env python3
"""
02_weather_tool.py  —  Toy "function calling" (a.k.a. tool use) demo.

GOAL (AI Step 3, deliverable: "a toy function-calling demo / fake weather
tool"):
    Teach the ONE idea the entire AI phase is built on:

        The LLM does NOT do things itself. It looks at what the user asked,
        and if a job needs a real-world action, it OUTPUTS A STRUCTURED
        REQUEST asking *us* to run one of the functions we told it about.
        WE run the function. We hand the result back. The model then writes
        the final human answer.

    Swap "get_weather" for "walk / wave_hand / sit_down" and this is exactly
    how the robot dispatcher (Step 4) will work. Understand this file and you
    understand the spine of the project.

THE LOOP (4 phases):
    1. We send the user's question + a DESCRIPTION of the tools available.
    2. The model replies NOT with prose, but with a "tool_calls" request:
       "please run get_weather(city='Tunis')".
    3. OUR python code runs the real function and gets a result.
    4. We send the result back; the model writes the final natural answer.

RUN IT (after `ollama pull llama3.2:3b`):
    python3 02_weather_tool.py
    python3 02_weather_tool.py "Should I bring a jacket in London right now?"
"""

import sys
import json
import requests

OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
MODEL = "llama3.2:3b"


# ===========================================================================
# PART A — the REAL function.
# ===========================================================================
# This is ordinary python. It knows nothing about LLMs. In the real project
# this would be "send a walk goal to the ROS2 action server". Here it just
# returns fake weather so we can focus on the wiring, not the data.
FAKE_WEATHER_DB = {
    "tunis":  {"temp_c": 28, "condition": "sunny"},
    "london": {"temp_c": 12, "condition": "rainy"},
    "tokyo":  {"temp_c": 18, "condition": "cloudy"},
}


def get_weather(city: str) -> dict:
    """Return fake current weather for a city. (Stand-in for a real action.)"""
    print(f"    [python] get_weather(city={city!r}) actually executing...")
    info = FAKE_WEATHER_DB.get(city.strip().lower())
    if info is None:
        return {"error": f"no weather data for '{city}'"}
    return {"city": city, **info}


# A registry so we can look the function up by the name the model uses.
AVAILABLE_FUNCTIONS = {
    "get_weather": get_weather,
}


# ===========================================================================
# PART B — the TOOL DESCRIPTION we give the model.
# ===========================================================================
# This is how the model LEARNS the tool exists. It's a JSON-Schema-ish
# description: the name, what it's for, and what arguments it takes. The model
# never sees our python code — only this description. Good descriptions =
# the model picks the right tool with the right arguments. This matters a lot
# later: our robot tool descriptions are what make the LLM choose "wave_hand"
# vs "bow" correctly.
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_weather",
            "description": "Get the current weather for a given city.",
            "parameters": {
                "type": "object",
                "properties": {
                    "city": {
                        "type": "string",
                        "description": "The city name, e.g. 'Tunis'.",
                    },
                },
                "required": ["city"],
            },
        },
    }
]


def call_ollama(messages: list) -> dict:
    """POST the conversation (+tools) to Ollama, return the message object."""
    payload = {
        "model": MODEL,
        "messages": messages,
        "tools": TOOLS,       # <-- this is what enables tool calling
        "stream": False,
    }
    resp = requests.post(OLLAMA_URL, json=payload, timeout=120)
    resp.raise_for_status()
    return resp.json()["message"]


def main() -> None:
    user_question = (
        " ".join(sys.argv[1:]) if len(sys.argv) > 1
        else "What's the weather like in Tunis right now?"
    )

    # `messages` is the running transcript. We APPEND to it as the loop goes,
    # so the model always sees the full context (its own tool request, then
    # the tool's result).
    messages = [
        {"role": "system",
         "content": "You are a helpful weather assistant. Use the tools "
                    "provided to answer questions about the weather."},
        {"role": "user", "content": user_question},
    ]

    print("=" * 64)
    print(f"USER: {user_question}")
    print("=" * 64)

    # ---- PHASE 1 & 2: ask the model; it may request a tool call ---------
    print("\n[phase 1] sending question + tool descriptions to the model...")
    try:
        assistant_msg = call_ollama(messages)
    except requests.exceptions.ConnectionError:
        sys.exit("ERROR: Ollama not reachable. -> systemctl status ollama")
    except requests.exceptions.HTTPError as e:
        sys.exit(f"ERROR from Ollama: {e}\n(Did you run: ollama pull {MODEL} ?)")

    # Add the model's reply to the transcript no matter what it said.
    messages.append(assistant_msg)

    tool_calls = assistant_msg.get("tool_calls")

    if not tool_calls:
        # The model decided it didn't need a tool (e.g. you asked "hello").
        print("\n[phase 2] model answered directly, no tool needed:")
        print("MODEL REPLY:", assistant_msg.get("content"))
        return

    print("\n[phase 2] model did NOT answer directly. It REQUESTED tool(s):")
    print(json.dumps(tool_calls, indent=2))

    # ---- PHASE 3: WE run each requested function ------------------------
    print("\n[phase 3] running the requested function(s) in python...")
    for call in tool_calls:
        fn_name = call["function"]["name"]
        fn_args = call["function"]["arguments"]   # already a dict in Ollama

        func = AVAILABLE_FUNCTIONS.get(fn_name)
        if func is None:
            result = {"error": f"unknown function '{fn_name}'"}
        else:
            result = func(**fn_args)

        print(f"    -> result: {result}")

        # Feed the result back as a new message with role="tool".
        # This is how the model "sees" what its requested action produced.
        messages.append({
            "role": "tool",
            "tool_name": fn_name,
            "content": json.dumps(result),
        })

    # ---- PHASE 4: ask again; now the model writes the human answer ------
    print("\n[phase 4] sending tool results back so the model can answer...")
    final_msg = call_ollama(messages)

    print("\n" + "=" * 64)
    print("FINAL MODEL REPLY:")
    print(final_msg.get("content"))
    print("=" * 64)


if __name__ == "__main__":
    main()
