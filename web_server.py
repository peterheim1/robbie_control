"""Web interface for Robbie voice control server.

Provides:
  GET  /             — single-page HTML control panel
  POST /api/command  — inject text command (bypass wake word + STT)
  POST /api/tts_mute — set TTS mute state  {"muted": true/false}
  WS   /ws           — real-time event stream + command input
"""

import asyncio
import json
import logging
from collections import deque
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Single-page HTML (embedded — no separate file to deploy)
# ---------------------------------------------------------------------------

_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robbie</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:#0d1117;color:#e6edf3;font-family:'Courier New',monospace;font-size:14px}
    header{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;
           padding:12px 16px;background:#161b22;border-bottom:1px solid #30363d}
    .page-wrap{display:flex;gap:0;align-items:flex-start}
    .right-sidebar{width:300px;flex-shrink:0;display:flex;flex-direction:column;
                   border-left:1px solid #30363d;min-height:calc(100vh - 90px)}
    .cam-panel{border-bottom:1px solid #30363d}
    .cam-panel .panel-hdr{padding:6px 12px;font-size:11px;color:#8b949e;text-transform:uppercase;
                          letter-spacing:1px;background:#161b22;border-bottom:1px solid #30363d}
    .cam-feed{width:100%;display:block;background:#000;min-height:169px;object-fit:contain}
    .cam-none{display:flex;align-items:center;justify-content:center;height:169px;
              color:#6e7681;font-size:12px;background:#0d1117}
    .tasks-panel{flex:1;background:#161b22;padding:10px 0}
    .tasks-panel .panel-hdr{padding:6px 12px;font-size:11px;color:#8b949e;text-transform:uppercase;
                            letter-spacing:1px;border-bottom:1px solid #30363d;margin-bottom:8px}
    .task-btn{display:block;width:100%;text-align:left;background:none;border:none;
              color:#c9d1d9;font-family:inherit;font-size:13px;padding:6px 12px;cursor:pointer;
              border-left:3px solid transparent;transition:background .15s}
    .task-btn:hover{background:#21262d;color:#58a6ff}
    .task-btn.running{border-left-color:#3fb950;color:#3fb950;background:#0d2a13}
    .task-step{font-size:11px;color:#6e7681;padding:3px 12px 6px 15px;word-break:break-word;line-height:1.4}
    .task-cancel{display:none;width:calc(100% - 24px);margin:4px 12px;background:#3d1f1f;border:1px solid #f85149;
                 color:#f85149;font-family:inherit;font-size:12px;padding:4px 8px;border-radius:4px;cursor:pointer}
    .task-cancel:hover{background:#f85149;color:#fff}
    .title{font-size:18px;font-weight:bold;color:#58a6ff}
    .status-badge{padding:4px 12px;border-radius:12px;font-size:12px;background:#21262d;color:#8b949e}
    .status-badge.listening{background:#1f2d1f;color:#3fb950}
    .status-badge.recording{background:#3d1f1f;color:#f85149}
    .status-badge.processing{background:#1f2a3d;color:#58a6ff}
    .status-badge.speaking{background:#2d1f3d;color:#bc8cff}
    .status-badge.disconnected{background:#3d2600;color:#e3b341}
    .mute-wrap{display:flex;align-items:center;gap:8px;cursor:pointer}
    .mute-label{font-size:12px;color:#8b949e;user-select:none}
    .toggle{position:relative;width:40px;height:20px;flex-shrink:0}
    .toggle input{opacity:0;width:0;height:0}
    .slider{position:absolute;cursor:pointer;inset:0;background:#21262d;border-radius:20px;transition:.3s}
    .slider:before{position:absolute;content:"";height:14px;width:14px;left:3px;bottom:3px;
                   background:#8b949e;border-radius:50%;transition:.3s}
    input:checked+.slider{background:#f85149}
    input:checked+.slider:before{transform:translateX(20px);background:#fff}
    .main-col{flex:1;min-width:0;padding:16px;display:flex;flex-direction:column;gap:14px}
    .banner{color:#f85149;text-align:center;padding:6px;font-size:12px;display:none}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden}
    .card-hdr{padding:6px 12px;background:#21262d;font-size:11px;color:#8b949e;
              text-transform:uppercase;letter-spacing:1px}
    .card-body{padding:12px}
    .cmd-row{display:flex;gap:8px}
    .cmd-input{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:6px;
               color:#e6edf3;font-family:inherit;font-size:14px;padding:8px 12px}
    .cmd-input:focus{outline:none;border-color:#58a6ff}
    .send-btn{background:#238636;color:#fff;border:none;border-radius:6px;
              padding:8px 16px;cursor:pointer;font-family:inherit;font-size:14px}
    .send-btn:hover{background:#2ea043}
    .grid{display:grid;grid-template-columns:80px 1fr;gap:5px 12px;align-items:start}
    .lbl{color:#8b949e;font-size:12px;padding-top:1px}
    .val{color:#e6edf3;word-break:break-word}
    .val.muted{color:#6e6e6e}
    .badge{font-size:11px;margin-left:6px}
    .badge.muted{color:#f85149}
    .badge.web{color:#58a6ff}
    .log-box{font-family:'Courier New',monospace;font-size:12px;height:300px;overflow-y:auto;
             background:#0d1117;padding:8px;line-height:1.7}
    .ll{white-space:pre-wrap;word-break:break-all}
    .ll.err{color:#f85149} .ll.warn{color:#e3b341} .ll.info{color:#6e7681}
    .ll.hear{color:#3fb950} .ll.intent{color:#58a6ff}
    .ll.tts{color:#bc8cff} .ll.tts-muted{color:#6e4496}
    .infobar{display:flex;align-items:center;gap:16px;flex-wrap:wrap;
             padding:5px 16px;background:#0d1117;border-bottom:1px solid #21262d;
             font-size:12px;color:#8b949e}
    .sep{color:#30363d}
  </style>
</head>
<body>
<header>
  <div class="title">&#x1F916; ROBBIE</div>
  <div id="status" class="status-badge disconnected">&#x25CF; CONNECTING</div>
  <label class="mute-wrap" title="Silent mode — text shown, robot stays quiet">
    <span class="mute-label">&#x1F507; Silent</span>
    <div class="toggle">
      <input type="checkbox" id="muteToggle" onchange="toggleMute()">
      <span class="slider"></span>
    </div>
  </label>
</header>
<div class="infobar">
  <span id="clock">--:--:--</span>
  <span class="sep">|</span>
  <span id="weather">fetching weather...</span>
</div>
<div class="page-wrap">
<div class="main-col">
  <div id="banner" class="banner">&#x26A0; Disconnected — reconnecting...</div>

  <div class="card">
    <div class="card-hdr">Command</div>
    <div class="card-body">
      <div class="cmd-row">
        <input id="cmdInput" class="cmd-input" type="text"
               placeholder="type a command and press Enter..."
               onkeydown="if(event.key==='Enter')sendCmd()">
        <button class="send-btn" onclick="sendCmd()">Send</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-hdr">Last Interaction</div>
    <div class="card-body">
      <div class="grid">
        <span class="lbl">Heard</span>   <span id="lHeard"    class="val">&mdash;</span>
        <span class="lbl">Intent</span>  <span id="lIntent"   class="val">&mdash;</span>
        <span class="lbl">Response</span><span id="lResponse" class="val">&mdash;</span>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-hdr">Live Log</div>
    <div id="log" class="log-box"></div>
  </div>
</div>

<div class="right-sidebar">
  <div class="cam-panel">
    <div class="panel-hdr">&#x1F4F7; Camera</div>
    <img id="camFeed" class="cam-feed"
         src="/camera/stream"
         alt=""
         onerror="camError()"
         onload="document.getElementById('camNone').style.display='none'">
    <div id="camNone" class="cam-none" style="display:none">No signal</div>
  </div>
  <div class="tasks-panel" id="tasksPanel">
    <div class="panel-hdr">&#x25B6; Tasks</div>
    <div id="taskList"><!-- populated by /api/tasks --></div>
    <button class="task-cancel" id="cancelBtn" onclick="cancelTask()">&#x25FC; Cancel</button>
  </div>
</div>
</div>
<script>
  let ws, reconnTimer;

  function connect() {
    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.onopen = () => {
      document.getElementById('banner').style.display = 'none';
    };
    ws.onclose = () => {
      document.getElementById('banner').style.display = 'block';
      setStatus('disconnected');
      reconnTimer = setTimeout(connect, 3000);
    };
    ws.onmessage = (e) => {
      const d = JSON.parse(e.data);
      switch (d.type) {
        case 'status':
          setStatus(d.state);
          if (d.state === 'listening') addLog('· listening', 'info');
          break;
        case 'transcript': {
          const src = d.source === 'web' ? '<span class="badge web">[web]</span>' : '';
          document.getElementById('lHeard').innerHTML = '"' + esc(d.text) + '"' + src;
          addLog('HEAR  "' + d.text + '"' + (d.source === 'web' ? ' [web]' : ''), 'hear');
          break;
        }
        case 'intent': {
          const p = Object.entries(d.params || {}).map(([k,v]) => k + '=' + v).join('  ');
          document.getElementById('lIntent').textContent = d.name + (p ? '  ' + p : '');
          addLog('INTENT ' + d.name + (p ? '  ' + p : ''), 'intent');
          break;
        }
        case 'tts': {
          const badge = d.muted ? '<span class="badge muted">&#x1F507; muted</span>' : '';
          const el = document.getElementById('lResponse');
          el.innerHTML = '"' + esc(d.text) + '"' + badge;
          el.className = d.muted ? 'val muted' : 'val';
          addLog('TTS   "' + d.text + '"' + (d.muted ? ' [muted]' : ''), d.muted ? 'tts-muted' : 'tts');
          break;
        }
        case 'log':
          addLog(d.msg, d.level || 'info');
          break;
        case 'tts_mute':
          document.getElementById('muteToggle').checked = d.muted;
          break;
        case 'task_update':
          handleTaskUpdate(d);
          break;
      }
    };
  }

  function setStatus(state) {
    const el = document.getElementById('status');
    const labels = {
      listening: '&#x25CF; LISTENING',
      recording: '&#x25CF; RECORDING',
      processing: '&#x25CC; PROCESSING',
      speaking:   '&#x25B6; SPEAKING',
      disconnected: '&#x25CF; DISCONNECTED',
    };
    el.innerHTML = labels[state] || state.toUpperCase();
    el.className = 'status-badge ' + state;
  }

  function addLog(msg, cls) {
    const box = document.getElementById('log');
    const line = document.createElement('div');
    const ts = new Date().toTimeString().slice(0, 8);
    line.className = 'll ' + (cls || 'info');
    line.textContent = ts + '  ' + msg;
    box.appendChild(line);
    box.scrollTop = box.scrollHeight;
    while (box.children.length > 200) box.removeChild(box.firstChild);
  }

  function sendCmd() {
    const inp = document.getElementById('cmdInput');
    const text = inp.value.trim();
    if (!text || !ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify({type: 'command', text}));
    inp.value = '';
  }

  function toggleMute() {
    const muted = document.getElementById('muteToggle').checked;
    if (ws && ws.readyState === 1)
      ws.send(JSON.stringify({type: 'set_tts_mute', muted}));
  }

  function esc(s) {
    return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
  }

  function updateClock() {
    const n = new Date();
    const t = n.toTimeString().slice(0, 8);
    const d = n.toLocaleDateString('en-GB', {weekday:'short', day:'numeric', month:'short'});
    document.getElementById('clock').textContent = t + '  \u00b7  ' + d;
  }
  updateClock();
  setInterval(updateClock, 1000);

  async function fetchWeather() {
    try {
      const r = await fetch('https://wttr.in/?format=%t+%C&lang=en');
      const txt = r.ok ? (await r.text()).trim() : '';
      document.getElementById('weather').textContent = txt || '';
    } catch(e) {
      document.getElementById('weather').textContent = '';
    }
  }
  fetchWeather();
  setInterval(fetchWeather, 600000);

  connect();

  // ---- Tasks panel ----
  let runningTask = null;

  async function loadTasks() {
    try {
      const r = await fetch('/api/tasks');
      if (!r.ok) return;
      const data = await r.json();
      renderTasks(data.tasks || []);
    } catch(e) {}
  }

  function renderTasks(tasks) {
    const el = document.getElementById('taskList');
    el.innerHTML = '';
    if (!tasks.length) {
      el.innerHTML = '<div style="padding:8px 12px;color:#6e7681;font-size:12px">No tasks found</div>';
      return;
    }
    tasks.forEach(name => {
      const btn = document.createElement('button');
      btn.className = 'task-btn';
      btn.id = 'task-' + name;
      btn.textContent = name;
      btn.onclick = () => runTask(name);
      el.appendChild(btn);
      const step = document.createElement('div');
      step.className = 'task-step';
      step.id = 'step-' + name;
      el.appendChild(step);
    });
  }

  function runTask(name) {
    if (ws && ws.readyState === 1)
      ws.send(JSON.stringify({type: 'run_task', name}));
  }

  function cancelTask() {
    if (ws && ws.readyState === 1)
      ws.send(JSON.stringify({type: 'cancel_task'}));
  }

  function handleTaskUpdate(d) {
    const {task, step, running} = d;
    // Clear previous running state
    if (runningTask && runningTask !== task) {
      const prev = document.getElementById('task-' + runningTask);
      const prevStep = document.getElementById('step-' + runningTask);
      if (prev) prev.classList.remove('running');
      if (prevStep) prevStep.textContent = '';
    }
    runningTask = running ? task : null;
    const btn = document.getElementById('task-' + task);
    const stepEl = document.getElementById('step-' + task);
    if (btn) btn.classList.toggle('running', running);
    if (stepEl) stepEl.textContent = running ? step : '';
    const cancelBtn = document.getElementById('cancelBtn');
    if (cancelBtn) cancelBtn.style.display = running ? 'block' : 'none';
    addLog((running ? 'TASK  ' : 'TASK  ') + task + '  ' + step, 'intent');
  }

  loadTasks();

  // ---- Camera feed ----
  function camError() {
    const img = document.getElementById('camFeed');
    const none = document.getElementById('camNone');
    img.style.display = 'none';
    none.style.display = 'flex';
    // retry after 5s
    setTimeout(() => {
      img.style.display = 'block';
      none.style.display = 'none';
      img.src = '/camera/stream?' + Date.now();
    }, 5000);
  }
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# WebServer class
# ---------------------------------------------------------------------------

class WebServer:
    """FastAPI/WebSocket server for the Robbie control panel."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self._host = host
        self._port = port
        self._tts_muted: bool = False
        self._clients: set = set()
        self._log_buffer: deque = deque(maxlen=100)
        self._voice_server = None

    @property
    def tts_muted(self) -> bool:
        return self._tts_muted

    async def broadcast(self, event: dict):
        """Push a JSON event to all connected WebSocket clients."""
        self._log_buffer.append(event)
        dead = set()
        msg = json.dumps(event)
        for ws in self._clients:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    async def start(self, voice_server):
        """Start the web server on the current asyncio event loop."""
        try:
            from fastapi import FastAPI, WebSocket, WebSocketDisconnect
            from fastapi.responses import HTMLResponse, StreamingResponse
            import uvicorn
        except ImportError:
            logger.error(
                "fastapi/uvicorn not installed — web interface disabled. "
                "Run: pip install fastapi uvicorn"
            )
            return

        self._voice_server = voice_server
        app = FastAPI()

        @app.get("/", response_class=HTMLResponse)
        async def index():
            return _HTML

        @app.post("/api/speak")
        async def api_speak(body: dict[str, Any]):
            """Speak text directly via TTS, bypassing intent classification."""
            text = body.get("text", "").strip()
            if text and self._voice_server:
                await self._voice_server._speak(text)
            return {"status": "ok"}

        @app.post("/api/command")
        async def api_command(body: dict[str, Any]):
            text = body.get("text", "").strip()
            if text and self._voice_server:
                asyncio.create_task(self._voice_server.handle_text_command(text))
            return {"status": "ok"}

        @app.get("/camera/stream")
        async def camera_stream():
            async def generate():
                while True:
                    dispatcher = (self._voice_server and
                                  getattr(self._voice_server, '_dispatcher', None))
                    frame = dispatcher.get_latest_camera_frame() if dispatcher else None
                    if frame:
                        yield (b"--frame\r\n"
                               b"Content-Type: image/jpeg\r\n\r\n" +
                               frame + b"\r\n")
                    await asyncio.sleep(0.05)  # ~20 fps cap
            return StreamingResponse(
                generate(),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/api/tasks")
        async def api_tasks():
            tasks = []
            if self._voice_server and self._voice_server._task_runner:
                tasks = self._voice_server._task_runner.list_tasks()
            return {"tasks": tasks}

        @app.post("/api/listen")
        async def api_listen(body: dict[str, Any] = {}):
            """Trigger a no-wake-word listen session and return the transcript.

            External programs POST here (optionally with {"timeout": 10.0}) and
            block until the user has spoken.  The WLED bar turns magenta while
            listening and green when done.

            Returns: {"transcript": "<spoken text>"}
            """
            if not self._voice_server:
                return {"transcript": "", "error": "voice server not ready"}
            timeout = float(body.get("timeout", 10.0))
            transcript = await self._voice_server.listen_for_answer(timeout=timeout)
            return {"transcript": transcript}

        @app.post("/api/tts_mute")
        async def api_tts_mute(body: dict[str, Any]):
            self._tts_muted = bool(body.get("muted", False))
            await self.broadcast({"type": "tts_mute", "muted": self._tts_muted})
            return {"status": "ok", "muted": self._tts_muted}

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            self._clients.add(websocket)
            # Replay buffered events to new client so log is populated on load
            for event in self._log_buffer:
                try:
                    await websocket.send_text(json.dumps(event))
                except Exception:
                    break
            # Send current mute state
            try:
                await websocket.send_text(
                    json.dumps({"type": "tts_mute", "muted": self._tts_muted})
                )
            except Exception:
                pass
            try:
                while True:
                    data = await websocket.receive_text()
                    try:
                        msg = json.loads(data)
                    except json.JSONDecodeError:
                        continue
                    if msg.get("type") == "command":
                        text = msg.get("text", "").strip()
                        if text and self._voice_server:
                            asyncio.create_task(
                                self._voice_server.handle_text_command(text)
                            )
                    elif msg.get("type") == "run_task":
                        name = msg.get("name", "").strip()
                        tr = (self._voice_server and
                              self._voice_server._task_runner)
                        if name and tr:
                            asyncio.create_task(tr.run_task(name))
                    elif msg.get("type") == "cancel_task":
                        tr = (self._voice_server and
                              self._voice_server._task_runner)
                        if tr:
                            tr.cancel()
                    elif msg.get("type") == "set_tts_mute":
                        self._tts_muted = bool(msg.get("muted", False))
                        await self.broadcast(
                            {"type": "tts_mute", "muted": self._tts_muted}
                        )
            except WebSocketDisconnect:
                pass
            except Exception as e:
                logger.debug(f"WebSocket error: {e}")
            finally:
                self._clients.discard(websocket)

        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            loop="none",
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.install_signal_handlers = lambda: None  # don't override main handlers
        logger.info(f"Web interface at http://{self._host}:{self._port}")
        asyncio.create_task(server.serve())
