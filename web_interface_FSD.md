# Robbie Web Interface — Functional Specification

**Date**: 2026-02-18
**Status**: Planning / pre-implementation
**Author**: Peter + Claude Code

---

## 1. Purpose

Add a browser-based control panel to the voice control server that runs alongside the existing pipeline. The web interface serves three goals:

1. **Testing** — type commands directly, bypassing wake word and STT, to test intent/ROS dispatch without speaking
2. **Monitoring** — real-time pipeline log, status, errors visible from any device on the network
3. **Situational awareness** — nav map with robot position (phase 2)

Accessible from phone, tablet or laptop on the same network. No install required on the client.

---

## 2. Non-Goals (out of scope for v1)

- No authentication (LAN-only, trusted network assumed)
- No video stream (separate concern)
- No config editing via UI (use YAML files)
- No React/Vue/Node build chain (single HTML file served by FastAPI)

---

## 3. Architecture — Hybrid (rosbridge + FastAPI), both on robot

Everything runs on the **robot i7**. SLAM and navigation run locally — the web UI only subscribes to topics and does not affect control loop timing.

```
ROBOT i7
  ├── robbie_voice_server.py
  │     └── FastAPI WebServer (port 8080)
  │           - serves index.html
  │           - WS: pipeline status, log stream, TTS mute
  │           - POST /api/command: text injection
  │
  └── rosbridge_suite (port 9090)
        - /map  (throttled to 0.2 Hz — enough for display, low WiFi cost)
        - /odom or /amcl_pose  (5 Hz — robot position)
        - /voice/intent, /voice/tts_text  (already ROS topics)

WiFi (one link, shared)
  └── Browser (phone / laptop / desktop)
        ├── ws://robot-ip:8080  — voice events, commands, log
        └── ws://robot-ip:9090  — ROS topics (map, pose)
```

### Why on the robot (not desktop)

- **No audio over WiFi** — mic is USB-direct on robot, audio never leaves the machine
- **ROS 2 publishing is local** — cmd_vel, /head/position published on-machine, zero network hop
- **/map is published on robot** — rosbridge subscribes locally, no extra DDS hop
- **Nav timing unaffected** — web UI is read-only subscriber; ROS 2 pub/sub is decoupled
- **Self-contained** — robot works without desktop being on

### WiFi traffic estimate

| Source | Rate | Notes |
|---|---|---|
| FastAPI voice events | ~2 KB/s | JSON events, negligible |
| rosbridge /map | ~50–200 KB per update × 0.2 Hz = ~10–40 KB/s | throttle in rosbridge config |
| rosbridge /odom | ~0.5 KB/s | tiny pose messages |
| Existing nav/SLAM topics to desktop | already present | not added by web UI |

Map throttling config in rosbridge launch:
```xml
<param name="max_message_size" value="10000000"/>
<param name="fragment_timeout" value="600"/>
<!-- throttle /map to 0.2 Hz via roslibjs on client side -->
```

### Starting rosbridge

```bash
# Install once on robot
sudo apt install ros-jazzy-rosbridge-suite

# Start (add to launch file alongside voice server)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## 4. Components

### 4.1 WebServer (`robbie_control/web_server.py`)

FastAPI app sharing the existing asyncio event loop:

```python
class WebServer:
    def __init__(self, host="0.0.0.0", port=8080):
        self._tts_muted = False      # server-side TTS mute flag
        self._clients = set()        # connected WebSocket clients
        self._log_buffer = deque(maxlen=100)  # last 100 lines for new connections

    async def start(self, voice_server):
        # voice_server ref for handle_text_command() and tts_muted property

    async def broadcast(self, event: dict):
        # push JSON event to all connected WS clients
```

Endpoints:

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Serve the single-page HTML |
| `/api/command` | POST | Inject text command into pipeline |
| `/api/tts_mute` | POST | Set TTS mute state `{"muted": true/false}` |
| `/ws` | WebSocket | Bidirectional: events out, commands + settings in |

### 4.2 Event Bus

`robbie_voice_server.py` gains a `publish_event(event: dict)` method:

```python
# Events pushed to web clients:
{"type": "status",     "state": "listening"}           # idle, watching for wake word
{"type": "status",     "state": "recording"}           # capturing speech
{"type": "status",     "state": "processing"}          # STT + intent running
{"type": "status",     "state": "speaking"}            # TTS playing (if not muted)
{"type": "transcript", "text": "look right"}           # STT result
{"type": "intent",     "name": "look", "params": {...}, "response": "looking right"}
{"type": "tts",        "text": "looking right",        # what robot said/would say
                       "muted": false}
{"type": "log",        "level": "info",  "msg": "..."}
{"type": "log",        "level": "error", "msg": "..."}
{"type": "tts_mute",   "muted": true}                  # mute state changed (sync all clients)
```

### 4.3 TTS Mute Toggle

A toggle in the browser that suppresses audio playback server-side — the robot stays silent but the response text is still shown on screen.

**Server side:**
- `WebServer._tts_muted` bool, defaulting to `False`
- Exposed as a property on the voice server: `voice_server.tts_muted`
- Before calling `send_tts_audio()`, the pipeline checks this flag:
  ```python
  if not self._web_server or not self._web_server.tts_muted:
      await self._esphome.send_tts_audio(audio)
  # Always publish event so UI shows the text regardless
  await self.publish_event({"type": "tts", "text": tts_text, "muted": self._web_server.tts_muted})
  ```
- When browser sends mute toggle, server broadcasts `{"type": "tts_mute", "muted": true}` to all clients so every browser stays in sync

**Client side:**
- Toggle switch (checkbox styled as a slider) labelled **"Silent mode"**
- When muted: TTS response text shown in a highlighted box; speaker icon shows crossed-out
- When unmuted: normal behaviour

### 4.4 Text Command Injection

New method in `RobbieVoiceServer`:

```python
async def handle_text_command(self, text: str):
    """Inject text directly into pipeline, bypassing wake word and STT."""
    await self.publish_event({"type": "transcript", "text": text, "source": "web"})
    # then same path as post-STT:
    # classify → dispatch ROS 2 → TTS (respects mute flag)
```

### 4.5 Frontend (embedded single-page HTML)

Vanilla JS, no framework. Responsive. Connects to both WebSockets on page load.

**Layout:**

```
┌─────────────────────────────────────────────────┐
│  🤖 ROBBIE          [● LISTENING]               │
│                                      [🔇 Silent] │ ← toggle switch
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─ Type a Command ────────────────────────┐   │
│  │  [ look left                   ] [Send] │   │
│  └─────────────────────────────────────────┘   │
│                                                 │
│  ┌─ Last Interaction ──────────────────────┐   │
│  │  Heard:    "look right"                 │   │
│  │  Intent:   look → pan=-1.2  tilt=0.0    │   │
│  │  Response: "looking right"   🔇 (muted) │   │ ← shows muted badge if silent
│  └─────────────────────────────────────────┘   │
│                                                 │
│  ┌─ Live Log ──────────────────────────────┐   │
│  │  19:36:50  HEAR   "look right."         │   │
│  │  19:36:50  INTENT look dir=right        │   │
│  │  19:36:50  TTS    "looking right" 🔇    │   │
│  │  19:36:51  [listening]                  │   │
│  └─────────────────────────────────────────┘   │
│                                                 │
│  ┌─ Nav Map (rosbridge / phase 2) ─────────┐   │
│  │                                         │   │
│  │   [canvas — occupancy grid + robot]     │   │
│  │                                         │   │
│  └─────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```

**Status indicator colours:**
- Grey: idle / listening for wake word
- Amber: recording speech
- Blue: processing (STT / intent)
- Green: speaking (TTS active)
- Green + 🔇: processing complete, TTS muted (text shown only)
- Red: error

---

## 5. Nav Map Viewer (Phase 2 — via rosbridge)

The browser uses `roslibjs` + `nav2djs` (standard ROS JS libraries, loaded from CDN) to connect to rosbridge on port 9090 and subscribe directly to:

- `/map` (`nav_msgs/OccupancyGrid`) — occupancy grid, rendered on canvas
- `/amcl_pose` or `/odom` — robot position + heading (blue arrow overlay)
- `/move_base_simple/goal` — current nav goal (red dot)

No custom server-side serialization needed — rosbridge handles it.

```html
<script src="https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/nav2djs/build/nav2d.min.js"></script>
```

---

## 6. Config

Add to `robbie_control/config/voice_config.yaml`:

```yaml
web:
  enabled: true
  host: "0.0.0.0"        # listen on all interfaces
  port: 8080
  tts_muted_default: false   # start with audio on
  rosbridge_port: 9090    # for frontend JS to connect to
```

---

## 7. Dependencies

**Python** — add to `requirements.txt`:
```
fastapi>=0.111.0
uvicorn>=0.29.0
```

**System** — install once on robot:
```bash
sudo apt install ros-jazzy-rosbridge-suite
```

**Browser** — loaded from CDN, no install:
- `roslibjs` — ROS WebSocket client
- `nav2djs` — map rendering (phase 2 only)

---

## 8. Implementation Phases

### Phase 1 — Text bypass + live log + TTS mute (MVP)
**Goal**: Test intent/ROS from browser without speaking. Silence robot on demand.

1. Create `robbie_control/web_server.py` — FastAPI + WebSocket + TTS mute flag
2. Add `web_server` startup to `robbie_voice_server.py`
3. Add `publish_event()` calls at key pipeline steps
4. Add TTS mute check before `send_tts_audio()` in pipeline
5. Add `handle_text_command()` to `RobbieVoiceServer`
6. Embed single-page HTML: command input, silent toggle, status badge, log stream
7. Add `web:` section to `voice_config.yaml`

Deliverable: `http://robot-ip:8080` — type commands, see log, toggle silent mode.

### Phase 2 — Nav map (rosbridge)
**Goal**: Live map with robot position from any browser.

1. Install `ros-jazzy-rosbridge-suite`
2. Add rosbridge to launch file alongside voice server
3. Add map canvas + roslibjs + nav2djs to the HTML page
4. Subscribe to `/map` + `/amcl_pose` via rosbridge WebSocket

Deliverable: Map panel visible below the log, robot position updates in real time.

### Phase 3 — Quick-command buttons + history (nice to have)
- Quick-tap buttons: Stop, Dock, Look Forward, Go Home
- Last N interactions shown on page load (from server-side buffer)
- Mobile-optimised large tap targets

---

## 9. Integration Points in Existing Code

| File | Change |
|---|---|
| `robbie_voice_server.py` | Start WebServer; add `publish_event()` calls; add `handle_text_command()`; check `tts_muted` before audio playback |
| `config/voice_config.yaml` | Add `web:` section |
| `requirements.txt` | Add fastapi, uvicorn |

No changes needed to: `local_mic_client.py`, `stt_engine.py`, `intent_classifier.py`, `tts_engine.py`, `ros2_dispatcher.py`

---

## 10. Example Flows

### Text command (silent mode on)
```
User types "go forward" + clicks Send
  → POST /api/command {"text": "go forward"}
  → handle_text_command("go forward")
    → publish_event({"type":"transcript", "text":"go forward", "source":"web"})
    → intent_classifier.classify() → drive intent
    → dispatcher.publish(intent)          ← ROS 2 publishes normally
    → tts.synthesize("moving forward")    ← generates audio
    → tts_muted == True → skip send_tts_audio()
    → publish_event({"type":"tts", "text":"moving forward", "muted":true})
  → Browser shows: Response: "moving forward" 🔇
```

### Silent mode toggle
```
User clicks Silent toggle ON
  → WS message: {"type":"set_tts_mute", "muted": true}
  → server._tts_muted = True
  → broadcast to all clients: {"type":"tts_mute", "muted": true}
  → all browser tabs update their toggle to ON
```

---

## 11. Open Questions

1. **uvicorn loop sharing**: share the asyncio loop using `config.setup()` + `asyncio.create_task(server.serve())`. Needs testing with sounddevice callbacks running simultaneously.
2. **Map data size**: `nav_msgs/OccupancyGrid` for a large map can be several MB. rosbridge streams it natively — may need to throttle updates (e.g. 1 Hz for map, 5 Hz for pose).
3. **Multiple clients + mute**: mute state is server-global. If two browsers are open and one mutes, both reflect the muted state. This is intentional — the robot either speaks or doesn't.
4. **Wake word while silent**: microphone and wake word still active. User can still say "hey Jarvis" — the response text appears but no audio plays.
