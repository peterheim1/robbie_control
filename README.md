# robbie_control

Voice control server for Robbie. Handles wake word detection, speech-to-text,
intent classification, ROS2 dispatch, text-to-speech, task sequencing, and a
browser-based control panel.

---

## Quick Start

```bash
cd ~/ros2_ws/src/robbie_control
source venv/bin/activate
python3 -m robbie_control.robbie_voice_server --config config/voice_config.yaml
```

Or use the system start script (also starts base driver and navigation):

```bash
~/robbie_start
```

Web UI: **`http://robot-ip:8090`**

---

## Architecture

```
USB Mic / TCP audio (ESP32)
        │
        ▼
  openwakeword  ─── "hey Jarvis" ───►  faster-whisper (STT)
                                              │
                                    stop_detector (fast path)
                                              │
                                    IntentClassifier
                                    ├── commands.txt  (phrase match)
                                    └── intents.yaml  (regex)
                                              │
                                    ROS2Dispatcher ──► /voice/intent
                                    ├── navigate_to (Nav2 action)
                                    ├── publish_stop
                                    ├── publish_head
                                    └── dispatch_ros_actions
                                              │
                                      Piper TTS ──► speaker
                                              │
                                        WebServer :8090
```

---

## Pipeline Components

| Module | File | Role |
|---|---|---|
| Voice server | `robbie_voice_server.py` | Main entry point, orchestrates all components |
| STT | `stt_engine.py` | Faster-Whisper transcription |
| Stop detector | `stop_detector.py` | Tiny model — checks first 0.5 s for stop keywords |
| Intent classifier | `intent_classifier.py` | Matches utterance to commands.txt / intents.yaml |
| Command loader | `command_loader.py` | Parses `config/commands.txt` into RosAction objects |
| ROS2 dispatcher | `ros2_dispatcher.py` | Publishes intents + actions; subscribes battery/diagnostics/camera |
| TTS | `tts_engine.py` | Piper TTS synthesis |
| LLM client | `llm_client.py` | Ollama (Mistral) for general questions |
| Task runner | `task_runner.py` | Executes named `.txt` task sequences |
| Web server | `web_server.py` | FastAPI control panel + docs search + camera feed |
| Docs engine | `docs_engine.py` | TF-IDF search over `.md` files + LLM streaming answers |
| WLED client | `wled_client.py` | LED bar (green=idle, magenta=listening) |
| TCP audio | `tcp_audio_client.py` | Receives PCM from ESP32 ReSpeaker over WiFi |
| Local mic | `local_mic_client.py` | USB mic input with on-device wake word |
| Console logger | `console_logger.py` | Coloured terminal output |

---

## Audio Input Modes

Set `audio_source` in `config/voice_config.yaml`:

| Mode | Value | Description |
|---|---|---|
| Local mic (default) | `local_mic` | USB mic via PipeWire/PulseAudio, wake word on-device |
| TCP audio | `tcp` | ReSpeaker Lite (XIAO ESP32-S3) sends PCM over WiFi to port 8765 |
| Push-to-talk | `push_to_talk` | Press Enter to speak — useful for testing |

---

## Configuration (`config/voice_config.yaml`)

```yaml
audio_source: "local_mic"      # local_mic | tcp | push_to_talk

stt:
  model: "small"               # tiny / base / small / medium
  device: "cpu"                # cpu or cuda

tts:
  model: "en_GB-alan-medium"   # British male (Jarvis-style)

llm:
  model: "mistral"
  host: "http://10.0.0.87:11434"
  max_tokens: 120

vad:
  silence_threshold_ms: 1500
  max_duration_ms: 5000

web:
  port: 8090

wled:
  enabled: true
  host: "10.0.0.85"

tasks:
  dir: "/home/pi/ros2_ws/src/robbie_control/tasks"
```

---

## Intent System

### `config/commands.txt` — Phrase matching

Simple commands resolved by exact phrase, substring, or keyword overlap.
Edit this file to add or change commands. No restart needed for testing —
use the web UI command box.

```ini
[stop]
phrases: stop, halt, freeze, emergency stop
say: stopping
flags: cancel_tasks
ros: empty:/voice/stop
ros: twist:/cmd_vel
ros: float64_array:/drive  values=0,0,0,0

[look right]
phrases: look right, turn your head right
say: looking right
ros: joint_traj:/head_controller/joint_trajectory  head_pan_joint=-1.2 head_tilt_joint=0.0
```

**ROS action types in commands.txt:**

| Type | Message | Example |
|---|---|---|
| `none` | — | No ROS publish |
| `empty:/topic` | `std_msgs/Empty` | Service trigger |
| `twist:/topic` | `geometry_msgs/Twist` (zeros) | Stop motion |
| `joint_traj:/topic joint=rad` | `trajectory_msgs/JointTrajectory` | Head / arm position |
| `float64_array:/topic values=v1,v2` | `std_msgs/Float64MultiArray` | Drive speeds |
| `srv_empty:/service` | `std_srvs/Empty` service call | Docking, calibration |

### `config/intents.yaml` — Regex patterns

Matched after commands.txt fails. Intents are checked in priority order.

| Intent | Example | Action |
|---|---|---|
| `goto` | "go to the kitchen" | Nav2 NavigateToPose to coordinates from `locations.yaml` |
| `query_datetime` | "what time is it" | Returns formatted time or date |
| `system_status` | "robot status" | Reads `/diagnostics` via dispatcher |
| `query_status` | "how's the battery" | Returns battery voltage / dock state |
| `run_task` | "run first nav" | Executes a named task file |
| `list_tasks` | "what tasks do you have" | Lists available task names |
| `general_question` | "what is your name" | Sent to Ollama LLM |

### `config/locations.yaml` — Named navigation targets

```yaml
locations:
  kitchen:
    x: 2.16
    y: 4.53
    yaw_deg: 137.33
    aliases: []
  tv:
    x: -1.35
    y: 3.17
    yaw_deg: 87.89
    aliases: ["television", "lounge", "living room"]
```

Add locations by running Nav2, driving to the target, reading `/odom`, and adding an entry.

---

## Task Sequences (`tasks/`)

Task files are plain text, one step per line:

```
NAV:tv                          navigate to named location (from locations.yaml)
TASK:MakeSound:I have arrived   speak text via TTS
TASK:Wait                       pause 5 seconds
TASK:Dock                       call /start_docking service
```

Run a task by voice ("run first nav") or from the web UI Tasks panel.

---

## Web Interface (port 8090)

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Single-page control panel |
| `/api/command` | POST | Inject text command into pipeline |
| `/api/speak` | POST | Speak text directly via TTS |
| `/api/listen` | POST | Capture spoken answer (no wake word), returns transcript |
| `/api/stop_all` | POST | Immediately stop all robot motion |
| `/api/shutdown` | POST | Shutdown the Raspberry Pi |
| `/api/tasks` | GET | List available task names |
| `/camera/snapshot` | GET | Latest OAK-D JPEG frame |
| `/api/docs/search?q=` | GET (SSE) | Search docs, stream LLM answer |
| `/ws` | WebSocket | Real-time event stream + commands |

See `web_interface_FSD.md` for full WebSocket event/command protocol.

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/voice/intent` | `std_msgs/String` (JSON) | Publish | Classified intent with params |
| `/voice/tts_text` | `std_msgs/String` | Publish | TTS response text |
| `/voice/stop` | `std_msgs/Empty` | Publish | Emergency stop signal |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Velocity commands (stop = zeros) |
| `/drive` | `std_msgs/Float64MultiArray` | Publish | Wheel drive speeds |
| `/head_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Publish | Head pan/tilt |
| `/voice/speak` | `std_msgs/String` | Subscribe | External nodes trigger TTS |
| `/battery_voltage` | `std_msgs/Float32` | Subscribe | Battery voltage (from base_driver) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Subscribe | System health |
| `/oak/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | Subscribe | Camera feed for web UI |

**External TTS trigger** — any node can make Robbie speak by publishing to `/voice/speak`:
```bash
ros2 topic pub --once /voice/speak std_msgs/String '{data: "Hello world"}'
```

---

## Models

| Model | Location | Downloaded by |
|---|---|---|
| Piper TTS | `models/piper/en_GB-alan-medium.onnx` | Manual |
| openwakeword `hey_jarvis` | `~/.local/lib/python*/site-packages/openwakeword/resources/models/` | Auto on first run |
| Faster-Whisper `small` | `~/.cache/huggingface/hub/` | Auto on first run |
| Ollama Mistral | Ollama server at `10.0.0.87:11434` | `ollama pull mistral` |

---

## Setup

```bash
cd ~/ros2_ws/src/robbie_control

# Create venv
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
# openwakeword requires separate install (tflite-runtime unavailable on Python 3.12)
pip install openwakeword==0.6.0 --no-deps

# Download Piper voice model
mkdir -p models/piper
# Copy en_GB-alan-medium.onnx + .json into models/piper/
```

---

## Testing

```bash
# Test audio capture
python3 test_audio_capture.py

# Test TTS + speaker
python3 test_speak.py

# Test TCP connection to ESP32 satellite
python3 test_connection.py
```

---

## File Map

```
robbie_control/
├── robbie_voice_server.py    ← main entry point
├── ros2_dispatcher.py        ← ROS2 publisher/subscriber bridge (spin-restart loop)
├── web_server.py             ← FastAPI control panel (port 8090)
├── docs_engine.py            ← TF-IDF doc search + streaming LLM answers
├── intent_classifier.py      ← commands.txt + intents.yaml classification
├── command_loader.py         ← parses commands.txt into RosAction objects
├── task_runner.py            ← NAV/TASK/WAIT sequence execution
├── stt_engine.py             ← Faster-Whisper wrapper
├── stop_detector.py          ← fast-path stop keyword detection
├── tts_engine.py             ← Piper TTS wrapper
├── llm_client.py             ← Ollama client
├── tcp_audio_client.py       ← ReSpeaker ESP32 audio receiver
├── local_mic_client.py       ← USB mic + local wake word
├── wled_client.py            ← LED bar control
├── console_logger.py         ← coloured terminal output
├── config/
│   ├── voice_config.yaml     ← all runtime settings
│   ├── commands.txt          ← phrase-matched commands + ROS actions
│   ├── intents.yaml          ← regex intent patterns
│   ├── locations.yaml        ← named Nav2 targets (x, y, yaw)
│   ├── head_positions.yaml   ← named head poses
│   └── backstory.yaml        ← Robbie persona data (for LLM system prompt)
├── tasks/
│   ├── first_nav.txt         ← NAV/TASK sequence files
│   └── ...
├── models/
│   └── piper/                ← Piper TTS .onnx voice models
├── data/
│   └── docs_history.json     ← Docs tab query history
└── venv/                     ← Python virtual environment
```
