# Robbie Web Interface — Functional Specification (As-Built)

**Date**: 2026-03-12
**Status**: Implemented
**Port**: 8090

---

## 1. Purpose

Browser-based control panel running alongside the voice pipeline. Serves three goals:

1. **Testing** — type commands directly, bypassing wake word and STT
2. **Monitoring** — real-time pipeline log, status, battery, and diagnostics
3. **Documentation** — LLM-powered docs search with live diagnostics and runnable ROS2 commands

Accessible from phone, tablet, or laptop on the same network. No install required.

---

## 2. Architecture

Everything runs on the robot. No rosbridge required — all ROS2 data is fed through
`ROS2Dispatcher` which subscribes to topics in a background thread and pushes state
to the web layer via WebSocket events.

```
ROBOT (Pi / i7)
  ├── robbie_voice_server.py
  │     └── WebServer (FastAPI, port 8090)
  │           - serves single-page HTML
  │           - WebSocket: /ws  (pipeline events, commands, cmd streaming)
  │           - REST: /api/*
  │
  └── ROS2Dispatcher (background rclpy spin thread)
        - subscribes /battery_voltage, /diagnostics, /oak/rgb/image_raw/compressed
        - spin thread auto-restarts on exception (_spinning flag + retry loop)

Browser (phone / laptop)
  └── ws://robot-ip:8090/ws  ─── all data, no rosbridge needed
```

---

## 3. HTTP API Reference

All endpoints accept/return JSON unless noted. No authentication.

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Single-page HTML |
| `/api/command` | POST | Inject text command (full pipeline: intent → ROS2 → TTS) |
| `/api/speak` | POST | Speak text directly via TTS, bypassing intent classification |
| `/api/listen` | POST | Capture a spoken answer (no wake word required), returns transcript |
| `/api/tts_mute` | POST | Set TTS mute state `{"muted": true/false}` |
| `/api/stop_all` | POST | Stop all robot motion immediately (publishes stop to /voice/stop, /cmd_vel, /drive) |
| `/api/shutdown` | POST | Shutdown the Raspberry Pi (`sudo shutdown -h now`) |
| `/api/tasks` | GET | List available task names |
| `/camera/snapshot` | GET | Latest OAK-D JPEG frame (polled by JS at ~2 Hz), 204 if no signal |
| `/api/docs/search?q=` | GET (SSE) | Search docs + stream LLM answer tokens; live /diagnostics for fault queries |
| `/api/docs/history` | GET | Last 20 query/answer entries |
| `/ws` | WebSocket | Bidirectional: events out + commands + cmd execution in |

### POST `/api/command`
```json
{ "text": "go to the kitchen" }
→ { "status": "ok" }
```
Fire-and-forget. Full pipeline runs asynchronously.

### POST `/api/speak`
```json
{ "text": "Hello, welcome!" }
→ { "status": "ok" }
```
Blocks until audio finishes playing (synchronous).

### POST `/api/listen`
```json
{ "timeout": 10.0 }
→ { "transcript": "my name is john" }
```
Starts recording without wake word. WLED → magenta while listening. Blocks until
Whisper finishes (timeout + STT latency). Returns `""` on silence.

### POST `/api/stop_all`
No body. Calls `dispatcher.publish_stop()` — publishes Empty to `/voice/stop`,
zero Twist to `/cmd_vel`, zero Float64MultiArray to `/drive`.

### GET `/api/docs/search` (SSE)
Streams JSON-encoded tokens:
```
data: {"token": "The base driver..."}\n\n
data: {"diag": "base_driver: OK\nbattery: 12.4V"}\n\n   ← fault queries only
data: {"sources": ["robbie_bot/fsd.md", ...]}\n\n
data: {"commands": [{"id": "health", "label": "ros2 topic echo /health_summary", ...}]}\n\n
data: [DONE]\n\n
```
Recognized ROS2 shortcut queries (`list nodes`, `list topics`, `list controllers`, etc.)
skip LLM and return direct `ros2` command output.

---

## 4. WebSocket Protocol

Connect: `ws://robot-ip:8090/ws`

On connect: server replays last 100 buffered events so new tabs see history.

### Events — server → browser

| `type` | Key fields | Meaning |
|---|---|---|
| `status` | `state` | Pipeline state: `listening / recording / processing / speaking / disconnected` |
| `transcript` | `text`, `source` | STT result or injected text (`voice / web / listen`) |
| `intent` | `name`, `params`, `response` | Classified intent |
| `tts` | `text`, `muted` | TTS about to play (or suppressed) |
| `tts_mute` | `muted` | Mute state changed (sync all clients) |
| `log` | `level`, `msg` | Server log line |
| `task_update` | `task`, `step`, `running` | Task runner step progress |
| `battery` | `voltage` | Battery voltage push (on connect if cached, then every 10 s) |
| `cmd_output` | `cmd_id`, `text` | Streaming subprocess output (Docs tab) |
| `cmd_done` | `cmd_id` | Subprocess finished |

### Commands — browser → server

```json
{ "type": "command",      "text": "stop" }
{ "type": "run_task",     "name": "first_nav" }
{ "type": "cancel_task"                       }
{ "type": "set_tts_mute", "muted": true       }
{ "type": "run_cmd",      "cmd_id": "health"  }   ← executes ROS2 command from Docs tab
{ "type": "stop_cmd",     "cmd_id": "health"  }   ← kills the subprocess
```

---

## 5. Frontend Layout

Single-page HTML, vanilla JS, no framework. Two tabs: **Control** and **Docs**.

### Header (all tabs)
```
🤖 ROBBIE   [● LISTENING]   [⏹ Stop All] [⏻ Shutdown] [🔇 Silent ─●]
```

### Info bar (Control tab)
```
HH:MM:SS  |  📍 Perth, 24°C  |  🔋 12.4V
```
Battery colour coding: `#3fb950` (≥11.5V), `#e3b341` (10.5–11.5V low), `#f85149` (<10.5V critical).

### Control tab layout
```
┌──────────────────────────────────┐  ┌─────────────────┐
│ Command input + Send button      │  │ 📷 Camera        │
├──────────────────────────────────┤  │ (JPEG polled 2Hz)│
│ Last Interaction                 │  ├─────────────────┤
│  Heard:    "look right"          │  │ ▶ Tasks          │
│  Intent:   look                  │  │ first_nav        │
│  Response: "looking right"       │  │ meet_greet       │
├──────────────────────────────────┤  │ [✗ Cancel]       │
│ Live Log (300px scrollable)      │  │                  │
│  19:36:50 HEAR  "look right"     │  │                  │
│  19:36:50 INTENT look dir=right  │  │                  │
└──────────────────────────────────┘  └─────────────────┘
```

### Docs tab layout
```
┌─ History ─┐  ┌──────────────────────────────────────────┐
│ prev query │  │ [ Ask anything...              ] [Ask]   │
│ prev query │  │                                          │
│ ...        │  │ ⚡ Live Diagnostics (fault queries only) │
│            │  │                                          │
│            │  │ LLM answer streamed here...              │
│            │  │                                          │
│            │  │ Sources: fsd.md, CLAUDE.md              │
│            │  │ [▶ list topics] [▶ list controllers]    │
│            │  │                                          │
│            │  │ $ ros2 topic list                       │
│            │  │ /battery_voltage...  [✗ Stop]            │
└────────────┘  └──────────────────────────────────────────┘
```

---

## 6. Status Badge Colours

| State | Colour | Meaning |
|---|---|---|
| `disconnected` | Amber | WebSocket not connected |
| `listening` | Green | Idle, waiting for wake word |
| `recording` | Red | Capturing speech |
| `processing` | Blue | STT + intent running |
| `speaking` | Purple | TTS active |

---

## 7. TTS Mute

Server-side flag (`WebServer._tts_muted`). When muted:
- `_speak()` in voice server skips `send_tts_audio()` but always emits `tts` event
- Browser shows response text with a 🔇 badge
- Toggle broadcasts `tts_mute` event to sync all open tabs
- Silent toggle in header uses a CSS slider

---

## 8. Camera Feed

- Source: `/oak/rgb/image_raw/compressed` (OAK-D Lite)
- JS polls `/camera/snapshot` every 500 ms using `<img>` element
- Server returns 204 (No Content) if no frame available → "No signal" placeholder shown
- Frame cached in `ROS2Dispatcher._latest_camera_frame` (set from rclpy spin thread)

---

## 9. Task Runner Integration

- `GET /api/tasks` — returns list of `.yaml` task files in `tasks/` dir
- Tasks visible in right sidebar; click to run via WebSocket `run_task` message
- Running task highlighted green with step description
- `task_update` event updates sidebar in real time
- Cancel button sends `cancel_task` WebSocket message

---

## 10. Docs Engine (`docs_engine.py`)

- Indexes all `.md` and `CLAUDE.md` files in the workspace at startup
- Vector search using TF-IDF + cosine similarity
- LLM answer streamed via SSE from Ollama (same model as voice server)
- Recognized ROS2 shortcut queries bypass LLM entirely (whitelist of safe read-only commands)
- Fault queries (`error`, `not working`, `why`, etc.) pull live `/diagnostics` output first
- Relevant commands extracted from answer text and shown as runnable buttons
- History persisted to `data/docs_history.json`

---

## 11. ROS2 Dispatcher (`ros2_dispatcher.py`)

Key design points:
- `rclpy.spin()` runs in a daemon background thread (`ros2_spin`)
- **Spin restart loop**: if `rclpy.spin()` raises, logs warning and retries after 1 s (prevents silent death of all callbacks)
- `_spinning` flag ensures clean shutdown without spurious restart
- Subscriptions: `/battery_voltage` (Float32), `/diagnostics` (DiagnosticArray), `/oak/rgb/image_raw/compressed` (CompressedImage), `/voice/speak` (String → TTS callback)
- Dynamic publishers created on first use (keyed by `(msg_type, topic)`)
- `navigate_to(x, y, yaw_deg)` sends Nav2 `NavigateToPose` action goal in a daemon thread

---

## 12. Running the Voice Server

```bash
# On the robot
cd /home/pi/ros2_ws/src/robbie_control
source venv/bin/activate
python3 -m robbie_control.robbie_voice_server \
    --config config/voice_config.yaml

# Or use the system start script
~/robbie_start
```

Web UI: `http://robot-ip:8090`

Key config (`config/voice_config.yaml`):
```yaml
web:
  enabled: true
  host: "0.0.0.0"
  port: 8090

wled:
  enabled: true
  host: "10.0.0.85"
```

---

## 13. Dependencies

**Python** (in `requirements.txt`):
```
fastapi>=0.111.0
uvicorn>=0.29.0
```

**System**: none beyond what voice server already needs.

---

## 14. Troubleshooting

| Symptom | Check |
|---|---|
| Connection refused on port 8090 | Voice server not running; check `~/robbie_start` |
| Battery shows `--.-V` | `/battery_voltage` not published — is `base_driver.py` running? |
| Camera shows "No signal" | OAK-D not publishing `/oak/rgb/image_raw/compressed` |
| Docs tab answers are stale | `docs_engine.py` indexes at startup; restart server after adding `.md` files |
| Stop All doesn't stop motion | ROS2 dispatcher spin may have failed — check server logs |
| `/api/listen` hangs | Check mic device; `listen_for_answer` acquires `_listen_lock` (one at a time) |
