# Robbie Voice Control — External Integration FSD

**Purpose:** Reference for developers building programs that interact with Robbie's
voice pipeline from a separate computer.  Written for the meet-and-greet use case.

**Robot IP:** `10.0.0.x` (replace with actual)
**Voice server port:** `8090`
**WLED bar IP:** `10.0.0.85`

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        Robbie (Pi / i7)                         │
│                                                                 │
│  USB Mic ──► openwakeword ──► Faster-Whisper ──► Intent         │
│                 (hey_jarvis)      (STT)          Classifier      │
│                                                     │           │
│                                              ROS 2 Topics       │
│                                              /voice/intent etc  │
│                                                     │           │
│  Speaker ◄── Piper TTS ◄── response text ◄──────────┘           │
│                                                                 │
│  WLED bar ◄── wled_client  (green=idle, magenta=listening)      │
│                                                                 │
│  Web server :8090  ◄─────────────────────── External program   │
│    POST /api/speak                                              │
│    POST /api/listen                                             │
│    POST /api/command                                            │
│    POST /api/stop_all                                           │
│    WS   /ws                                                     │
└─────────────────────────────────────────────────────────────────┘
```

The voice server runs continuously.  External programs communicate with it
over HTTP/WebSocket on port 8090.  No ROS knowledge is required for the
meet-and-greet program — all interaction is plain HTTP.

---

## 2. HTTP API Reference

All endpoints accept and return JSON.  No authentication.

---

### 2.1  POST `/api/speak`  — Make Robbie say something

Sends text directly to Piper TTS and plays it through the robot's speaker.
Bypasses intent classification entirely.

**Request body**
```json
{ "text": "Hello, welcome to the lab!" }
```

**Response**
```json
{ "status": "ok" }
```

The call returns **after** the audio has finished playing (synchronous).

**Example (curl)**
```bash
curl -X POST http://10.0.0.x:8090/api/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello, welcome to the lab!"}'
```

**Example (Python — stdlib only)**
```python
import urllib.request, json

def speak(text, host="10.0.0.x", port=8090):
    payload = json.dumps({"text": text}).encode()
    req = urllib.request.Request(
        f"http://{host}:{port}/api/speak",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=15) as r:
        return json.loads(r.read())

speak("Hello, I am Robbie!")
```

---

### 2.2  POST `/api/listen`  — Capture a spoken answer

Immediately starts recording from the microphone **without** requiring the
wake word.  WLED bar turns **magenta** while listening, returns to **green**
once speech ends.

The call **blocks** until the user has spoken and Whisper has transcribed it,
then returns the transcript.

**Request body** (all fields optional)
```json
{ "timeout": 10.0 }
```

| Field     | Type  | Default | Description                           |
|-----------|-------|---------|---------------------------------------|
| `timeout` | float | `10.0`  | Max seconds to wait for speech        |

**Response**
```json
{ "transcript": "my name is john" }
```

Returns `{ "transcript": "" }` if nothing was heard before the timeout.

**Example (curl)**
```bash
curl -X POST http://10.0.0.x:8090/api/listen \
  -H "Content-Type: application/json" \
  -d '{"timeout": 10.0}'
```

**Example (Python)**
```python
def listen(timeout=10.0, host="10.0.0.x", port=8090):
    payload = json.dumps({"timeout": timeout}).encode()
    req = urllib.request.Request(
        f"http://{host}:{port}/api/listen",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout + 5) as r:
        return json.loads(r.read()).get("transcript", "")

answer = listen(10.0)
print(f"User said: {answer}")
```

---

### 2.3  POST `/api/command`  — Inject a text command

Passes text through the **full pipeline** — intent classification, ROS
dispatch, and TTS response — as if the user had spoken it after the wake word.
Use this if you want Robbie to react to a phrase the way a voice command would.

**Request body**
```json
{ "text": "go to the reception area" }
```

**Response**
```json
{ "status": "ok" }
```

Returns immediately (fire-and-forget).  The pipeline runs asynchronously.

---

### 2.4  POST `/api/tts_mute`  — Silence the speaker

```json
{ "muted": true }
```

Useful during demos where you want pipeline events to appear in the web UI
without audio playing.

---

### 2.5  POST `/api/stop_all`  — Emergency stop

No body required. Immediately publishes stop signals:
- `Empty` to `/voice/stop`
- zero `Twist` to `/cmd_vel`
- zero `Float64MultiArray` to `/drive`

```bash
curl -X POST http://10.0.0.x:8090/api/stop_all
```

---

## 3. WebSocket Event Stream  `ws://10.0.0.x:8090/ws`

Connect to receive real-time pipeline events as JSON frames.  Useful for
building a display panel or logging session activity.

On connect, the server replays the last 100 buffered events so new connections
see recent history.

### Event types

| `type`       | When fired                        | Key fields                              |
|--------------|-----------------------------------|-----------------------------------------|
| `status`     | State changes                     | `state`: listening / recording / processing / speaking |
| `transcript` | Speech recognised                 | `text`, `source` (voice / web / listen) |
| `intent`     | Intent classified                 | `name`, `params`, `response`            |
| `tts`        | TTS about to play                 | `text`, `muted`                         |
| `log`        | Server log line                   | `level`, `msg`                          |
| `tts_mute`   | Mute state changed                | `muted`                                 |
| `task_update`| Task runner step progress         | `task`, `step`, `running`               |
| `battery`    | Battery voltage update            | `voltage` (float, volts)                |
| `cmd_output` | Streaming ROS2 command output     | `cmd_id`, `text`                        |
| `cmd_done`   | ROS2 command subprocess finished  | `cmd_id`                                |

### Send commands over WebSocket

```json
{ "type": "command",      "text": "stop" }
{ "type": "run_task",     "name": "first_nav" }
{ "type": "cancel_task"                       }
{ "type": "set_tts_mute", "muted": true       }
```

---

## 4. WLED LED Bar States

| Colour       | Meaning                                     |
|--------------|---------------------------------------------|
| Green (50%)  | Idle — server ready, listening for wake word |
| Magenta      | Actively capturing a voice response          |

The WLED changes are driven by the voice server automatically.
Your program does **not** need to control WLED directly — calling
`/api/listen` handles it.

If you want to drive WLED directly for your own states (e.g. a greeting
animation) the WLED JSON API is:

```bash
# Set solid blue
curl -X POST http://10.0.0.85/json/state \
  -H "Content-Type: application/json" \
  -d '{"on": true, "bri": 200, "seg": [{"col": [[0, 0, 255]]}]}'
```

Key fields: `on` (bool), `bri` (0-255), `seg[0].col[0]` = `[R, G, B]`.

---

## 5. Voice Pipeline Detail

### 5.1 Audio Input Modes

| Mode | Config `audio_source` | Description |
|---|---|---|
| TCP (default) | `tcp` | ReSpeaker Lite (XIAO ESP32-S3) sends PCM over TCP |
| Local mic | `local_mic` | USB mic directly on robot, openwakeword on-device |
| Push-to-talk | `push_to_talk` | Press ENTER to speak (testing/debug) |

### 5.2 STT — Faster-Whisper

- Model: `small` (on robot CPU; `tiny` for speed, `medium` for accuracy)
- Language: English, VAD filter enabled
- Output: lowercase string, leading/trailing whitespace stripped
- Audio: 16-bit PCM, 16 kHz, mono
- Typical latency: 0.5–2 s depending on utterance length

### 5.3 Stop Detector (fast-path)

A second STT instance (`tiny` model) checks the first ~0.5 s of audio for stop
keywords before the full utterance finishes.  If triggered, publishes stop
immediately and skips full STT.

Keywords configurable in `config/voice_config.yaml` → `stop_detector.keywords`.

### 5.4 Wake Word — openwakeword

- Model: `hey_jarvis`
- Threshold: `0.85` (TCP mode: `0.5`)
- The normal wake word pipeline is **bypassed** when you call `/api/listen`

### 5.5 TTS — Piper

- Voice: `en_GB-alan-medium` (British male, Jarvis-style)
- Output sample rate: 22050 Hz
- Synthesis time: ~200 ms for a short sentence on CPU

### 5.6 Intent Classification

After STT, text is classified in this order:

1. **commands.txt** — plain phrase matching (exact → substring → keyword overlap)
2. **intents.yaml** — regex patterns, sorted by priority

**Built-in intents relevant to meet-and-greet:**

| Intent           | Example utterance                    | Notes                              |
|------------------|--------------------------------------|------------------------------------|
| `goto`           | "go to reception"                    | Navigates to a named location      |
| `query_status`   | "how are you"                        | Returns a status response          |
| `query_datetime` | "what time is it"                    | Reads current time/date            |
| `general_question` | "what is your name"               | Sent to Ollama LLM                 |
| `system_status`  | "are there any errors"               | Reads /diagnostics via dispatcher  |
| `run_task`       | "run first nav"                      | Starts a named task                |
| `unknown`        | anything unmatched                   | "I don't know that command"        |

Commands from `commands.txt`:

| Command      | Phrases                              | ROS action                          |
|--------------|--------------------------------------|-------------------------------------|
| `stop`       | stop, halt, freeze                   | `/voice/stop`, zero `/cmd_vel`      |
| `look right` | look right, turn your head right     | `/head_controller/joint_trajectory` |
| `look left`  | look left, turn your head left       | same                                |
| `look up/down/forward` | look up / down / forward   | same                                |
| `dock`       | dock, go charge                      | `/start_docking` service            |
| `undock`     | undock                               | `/undock` service                   |

---

## 6. Meet and Greet — Recommended Program Structure

The meet-and-greet program runs on a **separate computer** and drives the
voice server over HTTP.  A typical session:

```
1.  Trigger (button press, presence sensor, face detection, etc.)
2.  Robbie speaks a greeting          →  POST /api/speak
3.  Robbie asks visitor's name        →  POST /api/speak
4.  Listen for name                   →  POST /api/listen   (WLED → magenta)
5.  Robbie greets by name             →  POST /api/speak
6.  Ask a follow-up question          →  POST /api/speak
7.  Listen for answer                 →  POST /api/listen
8.  Respond / route to department     →  POST /api/speak
9.  Navigate to destination           →  POST /api/command  ("go to reception")
```

### Minimal Python skeleton

```python
#!/usr/bin/env python3
"""Robbie meet-and-greet — skeleton."""

import urllib.request
import json
import time

HOST = "10.0.0.x"
PORT = 8090


def speak(text):
    _post("/api/speak", {"text": text})


def listen(timeout=10.0):
    return _post("/api/listen", {"timeout": timeout}).get("transcript", "")


def command(text):
    _post("/api/command", {"text": text})


def _post(path, body):
    payload = json.dumps(body).encode()
    req = urllib.request.Request(
        f"http://{HOST}:{PORT}{path}",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=20) as r:
        return json.loads(r.read())


# ── Main greeting flow ──────────────────────────────────────────

speak("Hello! Welcome to the lab. I am Robbie.")
time.sleep(0.5)

speak("May I ask your name?")
name = listen(timeout=8.0)

if name:
    speak(f"Nice to meet you, {name}.")
else:
    name = "visitor"
    speak("Nice to meet you.")

time.sleep(0.3)
speak("Who are you here to see today?")
answer = listen(timeout=10.0)

if answer:
    speak(f"Let me take you to {answer}.")
    command(f"go to {answer}")
else:
    speak("Please ask at the front desk and someone will help you.")
```

---

## 7. Timing Notes

| Operation              | Typical duration    |
|------------------------|---------------------|
| `POST /api/speak`      | 0.3 – 2 s (blocks until audio done) |
| `POST /api/listen`     | 1 – 10 s (blocks until speech ends + STT) |
| STT transcription      | 0.5 – 2 s after speech ends |
| Stop detector fast-path | ~0.6 s from wake word to stop action |
| WLED colour change     | < 50 ms             |
| Wake word detection    | continuous background process |

Set `urllib.request.urlopen` timeout to `timeout + 5` when calling
`/api/listen` to give enough headroom for STT after the audio capture.

Note: `/api/listen` acquires `_listen_lock` — only one listen session at a time.

---

## 8. Running the Voice Server

```bash
# On the robot
cd /home/pi/ros2_ws/src/robbie_control
source venv/bin/activate
python3 -m robbie_control.robbie_voice_server \
    --config config/voice_config.yaml

# Or use the start script (also launches base driver + nav)
~/robbie_start
```

Web UI: `http://robot-ip:8090`

Config file: `robbie_control/config/voice_config.yaml`

Key config options for meet-and-greet:

```yaml
stt:
  model: "small"      # tiny = faster, small = more accurate
  device: "cpu"       # cpu on robot, cuda on GPU desktop

tts:
  model: "en_GB-alan-medium"   # British male Jarvis voice

wled:
  enabled: true
  host: "10.0.0.85"

web:
  port: 8090
```

---

## 9. Troubleshooting

| Symptom | Check |
|---------|-------|
| Connection refused on port 8090 | Voice server not running |
| `/api/speak` returns quickly but no audio | Check TTS mute state (`/api/tts_mute {"muted": false}`) |
| `/api/listen` returns empty transcript | No mic audio — check mic device in config, or background noise too quiet |
| WLED not changing | Ping `10.0.0.85` — check WLED is on same network segment |
| TTS plays but wrong voice | Check `tts.model` in `voice_config.yaml` and that the `.onnx` file exists in `models/piper/` |
| `/api/listen` hangs past timeout | Increase `timeout` in request body; only one listen at a time (`_listen_lock`) |
| `/api/command` fires intent but no navigation | Destination must exist in `config/locations.yaml` |
| Battery always shows `--.-V` in web UI | `/battery_voltage` not published — base_driver.py not running |
