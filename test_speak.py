#!/usr/bin/env python3
"""Test the /api/speak endpoint.

Usage:
    python3 test_speak.py
    python3 test_speak.py "hello world"
    python3 test_speak.py --host 10.0.0.x --port 8090 "say something"
"""

import argparse
import sys
import urllib.request
import urllib.error
import json


def speak(text: str, host: str = "localhost", port: int = 8090) -> bool:
    url = f"http://{host}:{port}/api/speak"
    payload = json.dumps({"text": text}).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            result = json.loads(resp.read())
            print(f"Response: {result}")
            return result.get("status") == "ok"
    except urllib.error.URLError as e:
        print(f"Error: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Test Robbie TTS speak endpoint")
    parser.add_argument("text", nargs="?", default="Hello, I am Robbie. TTS is working correctly.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()

    print(f"Sending to http://{args.host}:{args.port}/api/speak")
    print(f'Text: "{args.text}"')
    ok = speak(args.text, host=args.host, port=args.port)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
