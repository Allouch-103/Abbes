
#!/usr/bin/env python3
"""
01_chat.py  —  Talk to a local Ollama model from Python.

GOAL (AI Step 3, deliverable: "a Python script that talks to Ollama
programmatically"):
    Send one message to the model and print its reply, using nothing but
    the `requests` library and Ollama's local HTTP API.

WHY THE RAW HTTP API (instead of the `ollama` python package)?
    Ollama runs a small web server on your machine (127.0.0.1:11434).
    EVERYTHING — the terminal `ollama run`, the python lib, and later our
    robot dispatcher — ultimately POSTs JSON to that server. By using
    `requests` directly here, you SEE the exact JSON we send and get back.
    Once that's clear, switching to the convenience library later is trivial.

RUN IT (after `ollama pull llama3.2:3b`):
    python3 01_chat.py
    python3 01_chat.py "Tell me a joke about robots."
"""

import sys
import json
import requests

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Ollama listens here by default. This is a LOCAL address — nothing leaves
# your machine, which is exactly why the project uses Ollama (no cloud).
OLLAMA_URL = "http://127.0.0.1:11434/api/chat"

# The model we decided on for this 4 GB-VRAM GPU. If you ever pull a
# different one, change just this line.
MODEL = "llama3.2:3b"


def chat(user_message: str) -> str:
    """Send one user message to the model and return its text reply."""

    # -- The request body -----------------------------------------------
    # `messages` is a LIST of turns. Each turn has a `role` and `content`.
    #   role="system"     -> instructions that set the model's behavior
    #   role="user"       -> what the human said
    #   role="assistant"  -> what the model said (we'll add these in later
    #                        steps to give the model memory of the convo)
    # This list-of-messages shape is the SAME structure used by function
    # calling and by the robot command loop later, so it's worth getting
    # comfortable with now.
    payload = {
        "model": MODEL,
        "messages": [
            {
                "role": "system",
                "content": "You are a concise, friendly assistant.",
            },
            {
                "role": "user",
                "content": user_message,
            },
        ],
        # stream=False -> Ollama waits until the whole answer is ready and
        # sends it as ONE json object. stream=True would send many little
        # chunks (nice for a typing effect, but harder to read while
        # learning). We keep it simple here.
        "stream": False,
    }

    # -- Show the human what we are about to send -----------------------
    print("--> POST", OLLAMA_URL)
    print("--> request body:")
    print(json.dumps(payload, indent=2))
    print()

    # -- Make the HTTP call ---------------------------------------------
    response = requests.post(OLLAMA_URL, json=payload, timeout=120)
    response.raise_for_status()  # turn HTTP errors (404/500) into exceptions
    data = response.json()

    # -- Show the raw reply so the structure is visible -----------------
    print("<-- raw response json:")
    print(json.dumps(data, indent=2))
    print()

    # The model's text lives at data["message"]["content"]. The rest of the
    # response is metadata (timing, token counts, whether it's done, etc.).
    return data["message"]["content"]


def main() -> None:
    # Use a command-line argument if given, else a default question.
    if len(sys.argv) > 1:
        user_message = " ".join(sys.argv[1:])
    else:
        user_message = "In one sentence, what are you?"

    try:
        reply = chat(user_message)
    except requests.exceptions.ConnectionError:
        sys.exit(
            "ERROR: could not reach Ollama at 127.0.0.1:11434.\n"
            "Is the service running?  ->  systemctl status ollama"
        )
    except requests.exceptions.HTTPError as e:
        # The most common cause here is 'model not found' (not pulled yet).
        sys.exit(f"ERROR from Ollama: {e}\n(Did you run: ollama pull {MODEL} ?)")

    print("=" * 60)
    print("MODEL REPLY:")
    print(reply)


if __name__ == "__main__":
    main()
