"""Web interface for Robbie voice control server.

Provides:
  GET  /                   — single-page HTML control panel (Control + Docs tabs)
  POST /api/command        — inject text command (bypass wake word + STT)
  POST /api/tts_mute       — set TTS mute state  {"muted": true/false}
  GET  /api/docs/search    — SSE stream: doc search + LLM answer
  GET  /api/docs/history   — last 20 query/answer entries
  POST /api/ros2/query     — run a safe read-only ROS2 command
  WS   /ws                 — real-time event stream + command + run_cmd input
"""

import asyncio
import json
import logging
import subprocess
from collections import deque
from typing import Any

from robbie_control.docs_engine import DocsEngine

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
    /* ── Tabs ── */
    .tab-bar{display:flex;background:#161b22;border-bottom:1px solid #30363d;padding:0 16px}
    .tab-btn{background:none;border:none;color:#8b949e;font-family:inherit;font-size:13px;
             padding:10px 16px;cursor:pointer;border-bottom:2px solid transparent;transition:.15s}
    .tab-btn:hover{color:#e6edf3}
    .tab-btn.active{color:#58a6ff;border-bottom-color:#58a6ff}
    .tab-pane{display:none}
    .tab-pane.active{display:block}
    /* ── Docs tab ── */
    .docs-layout{display:flex;height:calc(100vh - 112px)}
    .docs-sidebar{width:220px;flex-shrink:0;border-right:1px solid #30363d;
                  background:#161b22;overflow-y:auto;padding:8px 0}
    .docs-sidebar-hdr{padding:6px 12px;font-size:11px;color:#8b949e;
                      text-transform:uppercase;letter-spacing:1px;
                      border-bottom:1px solid #30363d;margin-bottom:4px}
    .docs-hist-item{padding:6px 12px;font-size:12px;color:#8b949e;cursor:pointer;
                    border-left:3px solid transparent;white-space:nowrap;
                    overflow:hidden;text-overflow:ellipsis}
    .docs-hist-item:hover{background:#21262d;color:#e6edf3;border-left-color:#58a6ff}
    .docs-main{flex:1;min-width:0;display:flex;flex-direction:column;
               padding:16px;gap:12px;overflow-y:auto}
    .docs-search-row{display:flex;gap:8px;flex-shrink:0}
    .docs-input{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:6px;
                color:#e6edf3;font-family:inherit;font-size:14px;padding:8px 12px}
    .docs-input:focus{outline:none;border-color:#58a6ff}
    .docs-answer{background:#161b22;border:1px solid #30363d;border-radius:8px;
                 padding:14px;font-size:13px;line-height:1.8;overflow-y:auto;
                 min-height:160px;flex:1;word-break:break-word}
    .docs-answer code{background:#0d1117;border-radius:4px;padding:2px 6px;
                      font-family:'Courier New',monospace;font-size:12px}
    .docs-answer pre{background:#0d1117;border-radius:6px;padding:10px;margin:8px 0;
                     overflow-x:auto;font-family:'Courier New',monospace;font-size:12px}
    .docs-sources{margin-top:10px;border-top:1px solid #30363d;padding-top:8px}
    .docs-source{font-size:11px;color:#6e7681;margin:2px 0}
    .docs-cmds{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px;
               padding-top:8px;border-top:1px solid #30363d}
    .run-btn{background:#1f2d1f;border:1px solid #3fb950;color:#3fb950;
             font-family:inherit;font-size:12px;padding:5px 12px;
             border-radius:4px;cursor:pointer;transition:.15s}
    .run-btn:hover{background:#3fb950;color:#000}
    .docs-diag{background:#0d1117;border:1px solid #21262d;border-radius:6px;
               padding:10px;font-size:11px;color:#8b949e;margin-bottom:10px;
               font-family:'Courier New',monospace;white-space:pre-wrap}
    .docs-diag-hdr{color:#e3b341;font-size:11px;text-transform:uppercase;
                   letter-spacing:1px;margin-bottom:6px}
    .cmd-result{background:#0d1117;border:1px solid #30363d;border-radius:6px;
                margin-top:8px;overflow:hidden}
    .cmd-result-hdr{display:flex;align-items:center;justify-content:space-between;
                    padding:6px 10px;background:#161b22;font-size:11px;color:#8b949e}
    .cmd-result-body{padding:8px 10px;font-family:'Courier New',monospace;font-size:11px;
                     max-height:180px;overflow-y:auto;white-space:pre-wrap;color:#c9d1d9}
    .stop-btn{background:#3d1f1f;border:1px solid #f85149;color:#f85149;
              font-family:inherit;font-size:11px;padding:2px 8px;
              border-radius:3px;cursor:pointer}
    .stop-btn:hover{background:#f85149;color:#fff}
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
<div class="tab-bar">
  <button class="tab-btn active" data-tab="control" onclick="showTab('control')">&#x2699; Control</button>
  <button class="tab-btn" data-tab="docs" onclick="showTab('docs')">&#x1F4DA; Docs</button>
</div>
<div id="tab-control" class="tab-pane active">
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
    <img id="camFeed" class="cam-feed" alt="" style="display:none">
    <div id="camNone" class="cam-none">No signal</div>
  </div>
  <div class="tasks-panel" id="tasksPanel">
    <div class="panel-hdr">&#x25B6; Tasks</div>
    <div id="taskList"><!-- populated by /api/tasks --></div>
    <button class="task-cancel" id="cancelBtn" onclick="cancelTask()">&#x25FC; Cancel</button>
  </div>
</div>
</div>
</div>
</div>
<div id="tab-docs" class="tab-pane">
  <div class="docs-layout">
    <div class="docs-sidebar">
      <div class="docs-sidebar-hdr">History</div>
      <div id="docsHistory"></div>
    </div>
    <div class="docs-main">
      <div class="docs-search-row">
        <input id="docsInput" class="docs-input" type="text"
               placeholder="Ask about Robbie, or &#x27;list nodes&#x27;, &#x27;list topics&#x27;&#x2026;"
               onkeydown="if(event.key===&#x27;Enter&#x27;)docsSearch()">
        <button class="send-btn" onclick="docsSearch()">Ask</button>
      </div>
      <div id="docsDiagWrap" style="display:none">
        <div class="docs-diag-hdr">&#x1F4E1; Live Diagnostics</div>
        <div id="docsDiagContent" class="docs-diag"></div>
      </div>
      <div id="docsAnswer" class="docs-answer" style="color:#6e7681">Ask anything about Robbie&#x2026;</div>
      <div id="docsSources" class="docs-sources" style="display:none"></div>
      <div id="docsCmds" class="docs-cmds" style="display:none"></div>
      <div id="cmdResults"></div>
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
        case 'cmd_output': {
          const body = document.getElementById('cmdbody-' + d.cmd_id);
          if (body) { body.textContent += d.text; body.scrollTop = body.scrollHeight; }
          break;
        }
        case 'cmd_done': {
          const btn = document.getElementById('stopbtn-' + d.cmd_id);
          if (btn) btn.style.display = 'none';
          break;
        }
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

  // ---- Camera feed (snapshot polling) ----
  (function startCamPoll() {
    const img  = document.getElementById('camFeed');
    const none = document.getElementById('camNone');
    let pending = false;
    setInterval(() => {
      if (pending) return;
      pending = true;
      const url = '/camera/snapshot?' + Date.now();
      fetch(url)
        .then(r => {
          if (r.status === 204) throw new Error('no frame');
          return r.blob();
        })
        .then(blob => {
          const old = img.src;
          img.src = URL.createObjectURL(blob);
          if (old && old.startsWith('blob:')) URL.revokeObjectURL(old);
          img.style.display = 'block';
          none.style.display = 'none';
        })
        .catch(() => {
          img.style.display = 'none';
          none.style.display = 'flex';
        })
        .finally(() => { pending = false; });
    }, 100);  // 10 fps poll
  })();

  // ---- Tab switching ----
  function showTab(name) {
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById('tab-' + name).classList.add('active');
    document.querySelector('[data-tab="' + name + '"]').classList.add('active');
    if (name === 'docs') loadDocsHistory();
  }

  // ---- Docs search ----
  let docsAbort = null;

  async function docsSearch() {
    const inp = document.getElementById('docsInput');
    const q = inp.value.trim();
    if (!q) return;
    const ansEl     = document.getElementById('docsAnswer');
    const srcEl     = document.getElementById('docsSources');
    const cmdsEl    = document.getElementById('docsCmds');
    const diagWrap  = document.getElementById('docsDiagWrap');
    const diagCont  = document.getElementById('docsDiagContent');
    ansEl.style.color = '';
    ansEl.innerHTML = '<span style="color:#8b949e">&#x23F3; Searching&#x2026;</span>';
    srcEl.style.display = 'none';
    cmdsEl.style.display = 'none';
    diagWrap.style.display = 'none';
    document.getElementById('cmdResults').innerHTML = '';
    if (docsAbort) docsAbort.abort();
    docsAbort = new AbortController();
    try {
      const resp = await fetch('/api/docs/search?q=' + encodeURIComponent(q),
                               {signal: docsAbort.signal});
      if (!resp.ok) { ansEl.textContent = '\u26a0 Error: ' + resp.status; return; }
      const reader = resp.body.getReader();
      const dec = new TextDecoder();
      let buf = '', answer = '';
      ansEl.innerHTML = '';
      while (true) {
        const {done, value} = await reader.read();
        if (done) break;
        buf += dec.decode(value, {stream: true});
        const lines = buf.split('\n');
        buf = lines.pop();
        for (const line of lines) {
          if (!line.startsWith('data: ')) continue;
          const raw = line.slice(6);
          if (raw === '[DONE]') continue;
          try {
            const ev = JSON.parse(raw);
            if (ev.token !== undefined) {
              answer += ev.token;
              ansEl.innerHTML = mdRender(answer);
            } else if (ev.diag !== undefined) {
              diagCont.textContent = ev.diag;
              diagWrap.style.display = 'block';
            } else if (ev.sources !== undefined) {
              srcEl.innerHTML = ev.sources.map(s =>
                '<div class="docs-source">&#x1F4C4; ' + esc(s) + '</div>').join('');
              srcEl.style.display = ev.sources.length ? 'block' : 'none';
            } else if (ev.commands !== undefined) {
              cmdsEl.innerHTML = ev.commands.map(c =>
                '<button class="run-btn" onclick="runCmd(\'' +
                c.id.replace(/\\/g,'\\\\').replace(/'/g,"\\'") + '\',\'' +
                c.label.replace(/\\/g,'\\\\').replace(/'/g,"\\'") + '\')">' +
                esc(c.label) + '</button>').join('');
              cmdsEl.style.display = ev.commands.length ? 'flex' : 'none';
            }
          } catch(e) {}
        }
      }
      loadDocsHistory();
    } catch(e) {
      if (e.name !== 'AbortError')
        document.getElementById('docsAnswer').textContent = '\u26a0 ' + e.message;
    }
  }

  // ---- History sidebar ----
  async function loadDocsHistory() {
    try {
      const r = await fetch('/api/docs/history');
      if (!r.ok) return;
      const data = await r.json();
      const el = document.getElementById('docsHistory');
      el.innerHTML = '';
      (data.entries || []).forEach(entry => {
        const div = document.createElement('div');
        div.className = 'docs-hist-item';
        div.title = entry.query;
        div.textContent = entry.query;
        div.onclick = () => {
          document.getElementById('docsInput').value = entry.query;
          docsSearch();
        };
        el.appendChild(div);
      });
    } catch(e) {}
  }

  // ---- Command execution ----
  function runCmd(id, label) {
    if (!ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify({type: 'run_cmd', cmd_id: id}));
    const res = document.getElementById('cmdResults');
    let box = document.getElementById('cmdbox-' + id);
    if (!box) {
      box = document.createElement('div');
      box.className = 'cmd-result';
      box.id = 'cmdbox-' + id;
      const safeId = id.replace(/\\/g,'\\\\').replace(/'/g,"\\'");
      box.innerHTML =
        '<div class="cmd-result-hdr"><span>' + esc(label) + '</span>' +
        '<button class="stop-btn" id="stopbtn-' + id +
        '" onclick="stopCmd(\'' + safeId + '\')">&#x25FC; Stop</button></div>' +
        '<div class="cmd-result-body" id="cmdbody-' + id + '"></div>';
      res.appendChild(box);
    } else {
      const b = document.getElementById('cmdbody-' + id);
      if (b) b.textContent = '';
      const s = document.getElementById('stopbtn-' + id);
      if (s) s.style.display = 'inline-block';
    }
  }

  function stopCmd(id) {
    if (ws && ws.readyState === 1)
      ws.send(JSON.stringify({type: 'stop_cmd', cmd_id: id}));
  }

  // ---- Minimal markdown renderer ----
  function mdRender(text) {
    return text
      .replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')
      .replace(/```[\\w]*\\n?([\\s\\S]*?)```/g,'<pre>$1</pre>')
      .replace(/`([^`]+)`/g,'<code>$1</code>')
      .replace(/\\*\\*([^*]+)\\*\\*/g,'<strong>$1</strong>')
      .replace(/\\*([^*]+)\\*/g,'<em>$1</em>')
      .replace(/^### (.+)$/gm,'<h3 style="color:#e6edf3;margin:8px 0 4px">$1</h3>')
      .replace(/^## (.+)$/gm,'<h2 style="color:#e6edf3;margin:10px 0 4px">$1</h2>')
      .replace(/^# (.+)$/gm,'<h1 style="color:#e6edf3;margin:12px 0 4px">$1</h1>')
      .replace(/^[-*] (.+)$/gm,'\u2022 $1')
      .replace(/\n/g,'<br>');
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
        self._docs = DocsEngine()
        self._cmd_procs: dict = {}  # cmd_id -> asyncio subprocess

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

    async def _run_cmd_stream(self, ws, cmd_id: str, cmd_def: dict):
        """Run an approved command and stream output back via WebSocket."""
        _ROS2_SETUP = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/pi/ros2_ws/install/setup.bash && "
        )
        full_cmd = _ROS2_SETUP + cmd_def["cmd"]
        is_bg = cmd_def.get("background", False)

        async def _send(payload: dict):
            try:
                await ws.send_text(json.dumps(payload))
            except Exception:
                pass

        try:
            proc = await asyncio.create_subprocess_shell(
                full_cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                executable="/bin/bash",
            )
            self._cmd_procs[cmd_id] = proc

            if is_bg:
                while True:
                    try:
                        line = await asyncio.wait_for(proc.stdout.readline(), 0.5)
                    except asyncio.TimeoutError:
                        if proc.returncode is not None:
                            break
                        continue
                    if not line:
                        break
                    await _send({"type": "cmd_output", "cmd_id": cmd_id,
                                 "text": line.decode(errors="replace")})
                await proc.wait()
            else:
                try:
                    out, _ = await asyncio.wait_for(proc.communicate(), timeout=30)
                    await _send({"type": "cmd_output", "cmd_id": cmd_id,
                                 "text": out.decode(errors="replace") or "(no output)"})
                except asyncio.TimeoutError:
                    await _send({"type": "cmd_output", "cmd_id": cmd_id,
                                 "text": "\n(timed out after 30 s)"})
        except Exception as e:
            await _send({"type": "cmd_output", "cmd_id": cmd_id,
                         "text": f"\n(error: {e})"})
        finally:
            self._cmd_procs.pop(cmd_id, None)
            await _send({"type": "cmd_done", "cmd_id": cmd_id})

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
        self._docs.startup()
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

        @app.get("/camera/snapshot")
        async def camera_snapshot():
            """Return the latest camera frame as a single JPEG (polled by JS)."""
            from fastapi.responses import Response
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            frame = dispatcher.get_latest_camera_frame() if dispatcher else None
            if not frame:
                return Response(status_code=204)  # No Content — JS treats as no signal
            return Response(
                content=frame,
                media_type="image/jpeg",
                headers={"Cache-Control": "no-store"},
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

        @app.get("/api/docs/search")
        async def api_docs_search(q: str = ""):
            """SSE stream: search docs, pull live diagnostics, stream LLM answer."""
            from fastapi.responses import StreamingResponse as SR

            async def generate():
                query = q.strip()
                if not query:
                    yield "data: [DONE]\n\n"
                    return

                # Direct ROS2 read-only shortcut
                ros2_cmd = self._docs.detect_ros2_query(query)
                if ros2_cmd:
                    result = await self._docs.run_ros2_query(ros2_cmd)
                    token = f"```\n{result}\n```"
                    yield f"data: {json.dumps({'token': token})}\n\n"
                    yield "data: [DONE]\n\n"
                    return

                # Pull live diagnostics for fault queries
                diag = ""
                if self._docs.is_fault_query(query):
                    diag = await self._docs.get_diagnostics()
                    if diag:
                        yield f"data: {json.dumps({'diag': diag})}\n\n"

                chunks = self._docs.search(query)
                sources = list(dict.fromkeys(c.path for c in chunks))

                # Stream LLM tokens
                answer_parts: list[str] = []
                async for token in self._docs.stream_answer(query, chunks, diag):
                    answer_parts.append(token)
                    yield f"data: {json.dumps({'token': token})}\n\n"

                answer = "".join(answer_parts)
                if sources:
                    yield f"data: {json.dumps({'sources': sources})}\n\n"
                cmds = self._docs.relevant_commands(query, answer)
                if cmds:
                    yield f"data: {json.dumps({'commands': cmds})}\n\n"
                self._docs.save_history(query, answer, sources)
                yield "data: [DONE]\n\n"

            return SR(
                generate(),
                media_type="text/event-stream",
                headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
            )

        @app.get("/api/docs/history")
        async def api_docs_history():
            return {"entries": self._docs.load_history(limit=20)}

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
                    elif msg.get("type") == "run_cmd":
                        cmd_id = msg.get("cmd_id", "").strip()
                        cmd_def = self._docs.get_command(cmd_id)
                        if cmd_def:
                            asyncio.create_task(
                                self._run_cmd_stream(websocket, cmd_id, cmd_def)
                            )
                    elif msg.get("type") == "stop_cmd":
                        cmd_id = msg.get("cmd_id", "").strip()
                        proc = self._cmd_procs.get(cmd_id)
                        if proc:
                            try:
                                proc.terminate()
                            except Exception:
                                pass
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
