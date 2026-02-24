#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
cd "$SCRIPT_DIR"
source .venv/bin/activate
python3 -m robbie_control.robbie_voice_server "$@"
