#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
cd "$SCRIPT_DIR/.."
source voice_control/.venv/bin/activate
python -m robbie_control.robbie_voice_server "$@"
