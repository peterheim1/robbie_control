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
import os
import pathlib
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
    .tasks-panel{background:#161b22;padding:4px 0}
    .tasks-panel .panel-hdr{padding:4px 12px;font-size:11px;color:#8b949e;text-transform:uppercase;
                            letter-spacing:1px;border-bottom:1px solid #30363d;margin-bottom:4px}
    .task-btn{display:block;width:100%;text-align:left;background:none;border:none;
              color:#c9d1d9;font-family:inherit;font-size:12px;padding:4px 12px;cursor:pointer;
              border-left:3px solid transparent;transition:background .15s}
    .task-btn:hover{background:#21262d;color:#58a6ff}
    .task-btn.running{border-left-color:#3fb950;color:#3fb950;background:#0d2a13}
    .task-step{font-size:10px;color:#6e7681;padding:2px 12px 4px 15px;word-break:break-word;line-height:1.3}
    .task-cancel{display:none;width:calc(100% - 24px);margin:3px 12px;background:#3d1f1f;border:1px solid #f85149;
                 color:#f85149;font-family:inherit;font-size:12px;padding:3px 8px;border-radius:4px;cursor:pointer}
    .task-cancel:hover{background:#f85149;color:#fff}
    .remap-panel{background:#161b22;border-top:1px solid #30363d;padding:4px 0}
    .remap-panel .panel-hdr{padding:4px 12px;font-size:11px;color:#8b949e;text-transform:uppercase;
                             letter-spacing:1px;border-bottom:1px solid #30363d;margin-bottom:4px}
    .remap-body{padding:4px 12px}
    .remap-btn{display:block;width:100%;background:#1f2d1f;border:1px solid #3fb950;color:#3fb950;
               font-family:inherit;font-size:12px;padding:4px 8px;border-radius:4px;cursor:pointer;margin-bottom:3px}
    .remap-btn:hover{background:#3fb950;color:#0d1117}
    .remap-btn.active{background:#3fb950;color:#0d1117}
    .remap-save-btn{display:block;width:100%;background:#1f2a3d;border:1px solid #58a6ff;color:#58a6ff;
                    font-family:inherit;font-size:12px;padding:4px 8px;border-radius:4px;cursor:pointer;margin-bottom:3px}
    .remap-save-btn:hover:not(:disabled){background:#58a6ff;color:#0d1117}
    .remap-save-btn:disabled{opacity:0.4;cursor:not-allowed}
    .remap-status{font-size:11px;color:#6e7681;line-height:1.4}
    .head-panel{background:#161b22;border-top:1px solid #30363d;padding:4px 0}
    .head-panel .panel-hdr{padding:4px 12px;font-size:11px;color:#8b949e;text-transform:uppercase;
                            letter-spacing:1px;border-bottom:1px solid #30363d;margin-bottom:4px}
    .head-body{padding:4px 12px;display:flex;flex-direction:column;align-items:center;gap:4px}
    .head-tilt-lbl{font-size:11px;color:#8b949e}
    .head-tilt-slider{writing-mode:vertical-lr;direction:rtl;height:100px;
                      accent-color:#58a6ff;cursor:pointer}
    .head-tilt-ctr{font-size:11px;color:#8b949e}
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
    .docs-source{font-size:11px;color:#58a6ff;margin:2px 0;cursor:pointer;text-decoration:underline}
    .docs-source:hover{color:#79c0ff}
    .doc-viewer{background:#161b22;border:1px solid #30363d;border-radius:6px;
                margin-top:10px;padding:12px;display:none;max-height:400px;overflow-y:auto}
    .doc-viewer-hdr{display:flex;justify-content:space-between;align-items:center;
                    margin-bottom:8px;padding-bottom:8px;border-bottom:1px solid #30363d}
    .doc-viewer-title{font-size:12px;color:#8b949e;font-family:'Courier New',monospace}
    .doc-viewer-close{background:none;border:none;color:#6e7681;cursor:pointer;
                      font-size:16px;padding:0 4px;line-height:1}
    .doc-viewer-close:hover{color:#f85149}
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
    .hdr-right{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
    .danger-btn{background:#3d1f1f;border:1px solid #f85149;color:#f85149;
                font-family:inherit;font-size:12px;padding:5px 12px;
                border-radius:4px;cursor:pointer;transition:.15s;font-weight:bold}
    .danger-btn:hover{background:#f85149;color:#fff}
    .shutdown-btn{background:#2d1500;border:1px solid #e3b341;color:#e3b341;
                  font-family:inherit;font-size:12px;padding:5px 12px;
                  border-radius:4px;cursor:pointer;transition:.15s;font-weight:bold}
    .shutdown-btn:hover{background:#e3b341;color:#000}
    #batteryInfo{color:#3fb950}
    #batteryInfo.low{color:#e3b341}
    #batteryInfo.critical{color:#f85149}
    /* ── Logs tab ── */
    .logs-layout{display:flex;flex-direction:column;height:calc(100vh - 112px);padding:16px;gap:12px}
    .logs-toolbar{display:flex;align-items:center;gap:12px;flex-shrink:0}
    .logs-status{font-size:12px;color:#8b949e}
    .logs-report{background:#161b22;border:1px solid #30363d;border-radius:8px;
                 padding:14px;font-size:13px;line-height:1.8;overflow-y:auto;
                 flex:1;word-break:break-word}
    .logs-report code{background:#0d1117;border-radius:4px;padding:2px 6px;
                      font-family:'Courier New',monospace;font-size:12px}
    .logs-report pre{background:#0d1117;border-radius:6px;padding:10px;margin:8px 0;
                     overflow-x:auto;font-family:'Courier New',monospace;font-size:12px}
    .log-error{color:#f85149} .log-warn{color:#e3b341} .log-ok{color:#3fb950}
    /* ── Tooltip ── */
    [data-tip]{position:relative;cursor:default}
    [data-tip]:hover::after{content:attr(data-tip);position:absolute;z-index:999;
      bottom:calc(100% + 6px);left:50%;transform:translateX(-50%);
      background:#1c2128;color:#c9d1d9;border:1px solid #444c56;
      border-radius:6px;padding:6px 10px;font-size:11px;white-space:pre-wrap;
      max-width:260px;line-height:1.5;pointer-events:none;
      box-shadow:0 4px 12px rgba(0,0,0,.5)}
    [data-tip]:hover::before{content:"";position:absolute;z-index:999;
      bottom:calc(100% + 1px);left:50%;transform:translateX(-50%);
      border:5px solid transparent;border-top-color:#444c56;pointer-events:none}
    /* ── Task context menu ── */
    #taskCtxMenu{position:fixed;z-index:1000;background:#1c2128;border:1px solid #444c56;
      border-radius:6px;padding:4px 0;box-shadow:0 4px 16px rgba(0,0,0,.6);display:none}
    .ctx-item{padding:7px 16px;font-size:13px;color:#c9d1d9;cursor:pointer;
      font-family:'Courier New',monospace;white-space:nowrap}
    .ctx-item:hover{background:#2d333b;color:#58a6ff}
    /* ── Task editor modal ── */
    #taskEditorOverlay{display:none;position:fixed;inset:0;z-index:2000;
      background:rgba(0,0,0,.65);align-items:center;justify-content:center}
    #taskEditorOverlay.open{display:flex}
    #taskEditorBox{background:#161b22;border:1px solid #30363d;border-radius:10px;
      width:min(600px,96vw);max-height:80vh;display:flex;flex-direction:column;
      box-shadow:0 8px 32px rgba(0,0,0,.7)}
    .task-ed-hdr{display:flex;justify-content:space-between;align-items:center;
      padding:10px 16px;border-bottom:1px solid #30363d;background:#21262d;
      border-radius:10px 10px 0 0}
    .task-ed-title{color:#e6edf3;font-size:14px;font-weight:bold}
    .task-ed-close{background:none;border:none;color:#6e7681;font-size:18px;
      cursor:pointer;padding:0 4px;line-height:1}
    .task-ed-close:hover{color:#f85149}
    .task-ed-body{padding:12px;flex:1;overflow:auto}
    .task-ed-help{font-size:11px;color:#6e7681;margin-bottom:8px;line-height:1.5}
    #taskEditorArea{width:100%;background:#0d1117;color:#c9d1d9;border:1px solid #30363d;
      border-radius:6px;font-family:'Courier New',monospace;font-size:13px;
      padding:10px;resize:vertical;min-height:200px;outline:none;line-height:1.6}
    #taskEditorArea:focus{border-color:#58a6ff}
    .task-ed-footer{display:flex;justify-content:flex-end;gap:8px;
      padding:10px 16px;border-top:1px solid #30363d;background:#161b22;
      border-radius:0 0 10px 10px}
    .task-ed-save{background:#238636;color:#fff;border:none;border-radius:6px;
      padding:7px 20px;font-family:inherit;font-size:13px;cursor:pointer}
    .task-ed-save:hover{background:#2ea043}
    .task-ed-cancel{background:none;border:1px solid #30363d;color:#8b949e;
      border-radius:6px;padding:7px 16px;font-family:inherit;font-size:13px;cursor:pointer}
    .task-ed-cancel:hover{border-color:#8b949e;color:#c9d1d9}
    /* ── Diagnostics tab ── */
    .diag-layout{display:flex;flex-direction:column;height:calc(100vh - 112px);padding:16px;gap:12px}
    .diag-toolbar{display:flex;align-items:center;gap:12px;flex-shrink:0}
    .diag-updated{font-size:12px;color:#6e7681}
    .diag-table{background:#161b22;border:1px solid #30363d;border-radius:8px;
                overflow:hidden;flex:1;overflow-y:auto}
    .diag-row{display:grid;grid-template-columns:90px 160px 1fr;gap:0;
              border-bottom:1px solid #21262d;padding:7px 12px;align-items:center}
    .diag-row:last-child{border-bottom:none}
    .diag-badge{display:inline-block;font-size:11px;font-weight:bold;padding:2px 7px;
                border-radius:10px;text-align:center;letter-spacing:.5px}
    .diag-ok  .diag-badge{background:#0d2a13;color:#3fb950;border:1px solid #3fb950}
    .diag-warn .diag-badge{background:#2d2000;color:#e3b341;border:1px solid #e3b341}
    .diag-err  .diag-badge{background:#3d1f1f;color:#f85149;border:1px solid #f85149}
    .diag-stale .diag-badge{background:#1c2128;color:#6e7681;border:1px solid #444c56}
    .diag-name{font-size:13px;color:#c9d1d9;padding:0 12px}
    .diag-msg{font-size:12px;color:#8b949e;word-break:break-word;line-height:1.4}
    .diag-err  .diag-msg{color:#e07070}
    .diag-warn .diag-msg{color:#c9a227}
    .diag-empty{padding:20px;text-align:center;color:#6e7681;font-size:13px}
    /* ── Joints tab ── */
    .joints-layout{display:flex;flex-direction:column;height:calc(100vh - 112px);padding:16px;gap:12px}
    .joints-toolbar{display:flex;align-items:center;gap:16px;flex-shrink:0;flex-wrap:wrap}
    .joints-groups{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;overflow-y:auto;flex:1;align-content:start}
    .joints-group{background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden}
    .joints-group-hdr{padding:6px 12px;background:#21262d;font-size:11px;color:#8b949e;
                      text-transform:uppercase;letter-spacing:1px}
    .joint-row{display:grid;grid-template-columns:72px 1fr 40px 40px;gap:6px;align-items:center;
               padding:5px 10px;border-bottom:1px solid #21262d}
    .joint-row:last-child{border-bottom:none}
    .joint-lbl{font-size:12px;color:#c9d1d9;white-space:nowrap}
    .joint-range{width:100%;accent-color:#58a6ff;cursor:pointer}
    .joint-cmd{font-size:11px;color:#58a6ff;text-align:right;font-family:'Courier New',monospace;min-width:36px}
    .joint-actual{font-size:11px;color:#3fb950;text-align:right;font-family:'Courier New',monospace;min-width:36px}
    .joints-speed-row{display:flex;align-items:center;gap:8px;font-size:12px;color:#8b949e}
    .joints-speed-row input{accent-color:#58a6ff;width:80px}
    .joints-status{font-size:12px;color:#6e7681}
    /* ── Joints toolbar buttons ── */
    .zero-btn,.pose-action-btn,.mirror-btn,.save-pose-btn,.torque-off-btn,.torque-on-btn{
      font-family:inherit;font-size:12px;padding:5px 12px;
      border-radius:4px;cursor:pointer;border:1px solid;white-space:nowrap}
    .zero-btn{background:#21262d;border-color:#6e7681;color:#8b949e}
    .zero-btn:hover{background:#6e7681;color:#0d1117}
    .pose-action-btn{background:#1f2a3d;border-color:#58a6ff;color:#58a6ff}
    .pose-action-btn:hover{background:#58a6ff;color:#0d1117}
    .mirror-btn{background:#1f2d1f;border-color:#3fb950;color:#3fb950}
    .mirror-btn:hover{background:#3fb950;color:#0d1117}
    .save-pose-btn{background:#2d1f3d;border-color:#bc8cff;color:#bc8cff}
    .save-pose-btn:hover{background:#bc8cff;color:#0d1117}
    .torque-off-btn{background:#2d1f1f;border-color:#f0883e;color:#f0883e}
    .torque-off-btn:hover{background:#f0883e;color:#0d1117}
    .torque-on-btn{background:#1f2d1f;border-color:#3fb950;color:#3fb950}
    .torque-on-btn:hover{background:#3fb950;color:#0d1117}
    /* ── Poses panel ── */
    .poses-beh-row{display:flex;gap:10px;flex-shrink:0}
    .poses-panel{background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden;flex:1;min-width:0}
    .poses-panel-hdr{padding:6px 12px;background:#21262d;font-size:11px;color:#8b949e;
                     text-transform:uppercase;letter-spacing:1px;display:flex;align-items:center;gap:8px}
    .poses-list{display:flex;flex-wrap:wrap;gap:8px;padding:10px 12px;min-height:46px}
    .poses-empty{padding:2px 0;color:#6e7681;font-size:12px}
    .posesel-row{display:flex;align-items:center;gap:6px;padding:8px 10px}
    .posesel-select{flex:1;min-width:0;max-width:220px;background:#0d1117;border:1px solid #30363d;
      border-radius:4px;color:#c9d1d9;font-family:inherit;font-size:12px;
      padding:4px 6px;outline:none;cursor:pointer;height:28px}
    .posesel-select:focus{border-color:#58a6ff}
    .posesel-select:disabled{opacity:0.5;cursor:default}
    .posesel-btn{flex-shrink:0;background:#21262d;border:1px solid #30363d;color:#8b949e;padding:5px 11px;font-size:12px;
      font-family:inherit;font-size:11px;padding:3px 8px;border-radius:4px;cursor:pointer}
    .posesel-btn:hover{border-color:#58a6ff;color:#58a6ff}
    .posesel-btn:disabled{opacity:0.4;cursor:default}
    .posesel-del-btn{color:#f85149 !important;border-color:#f85149 !important}
    .posesel-del-btn:hover{background:#f85149 !important;color:#0d1117 !important}
    .posesel-prog{padding:2px 12px 6px;font-size:11px;color:#3fb950}
    .pose-item{display:flex;align-items:center;gap:6px;background:#0d1117;
               border:1px solid #30363d;border-radius:6px;padding:4px 10px}
    .pose-item-name{font-size:12px;color:#c9d1d9;min-width:60px}
    .pose-preview-btn{background:none;border:1px solid #30363d;color:#8b949e;font-family:inherit;
                      font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer}
    .pose-preview-btn:hover{border-color:#58a6ff;color:#58a6ff}
    .pose-run-btn{background:#1f2d1f;border:1px solid #3fb950;color:#3fb950;font-family:inherit;
                  font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer}
    .pose-run-btn:hover{background:#3fb950;color:#0d1117}
    .pose-edit-btn{background:none;border:1px solid #bc8cff;color:#bc8cff;font-family:inherit;
                   font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer}
    .pose-edit-btn:hover{background:#bc8cff;color:#0d1117}
    /* ── Save pose modal ── */
    /* ── Pose editor modal ── */
    #poseEditorOverlay{display:none;position:fixed;inset:0;z-index:2000;
      background:rgba(0,0,0,.65);align-items:center;justify-content:center}
    #poseEditorOverlay.open{display:flex}
    #poseEditorBox{background:#161b22;border:1px solid #30363d;border-radius:10px;
      width:min(380px,96vw);max-height:88vh;display:flex;flex-direction:column;
      box-shadow:0 8px 32px rgba(0,0,0,.7)}
    .pe-hdr{display:flex;justify-content:space-between;align-items:center;
      padding:10px 16px;border-bottom:1px solid #30363d;background:#21262d;
      border-radius:10px 10px 0 0;flex-shrink:0}
    .pe-title{color:#e6edf3;font-size:14px;font-weight:bold}
    .pe-close{background:none;border:none;color:#6e7681;font-size:18px;cursor:pointer;padding:0 4px;line-height:1}
    .pe-close:hover{color:#f85149}
    .pe-body{padding:12px 16px;overflow-y:auto;display:flex;flex-direction:column;gap:8px}
    .pe-meta{display:flex;flex-direction:column;gap:6px;padding-bottom:10px;border-bottom:1px solid #21262d}
    .pe-row{display:flex;align-items:center;gap:8px}
    .pe-meta-label{font-size:11px;color:#6e7681;width:80px;flex-shrink:0}
    .pe-input{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:5px;
      color:#e6edf3;font-family:inherit;font-size:12px;padding:5px 8px;outline:none}
    .pe-mt-input{width:54px;background:#0d1117;border:1px solid #30363d;border-radius:5px;
      color:#e6edf3;font-family:inherit;font-size:12px;padding:5px 8px;text-align:right;outline:none}
    .pe-joints-hdr{font-size:10px;color:#6e7681;text-transform:uppercase;letter-spacing:1px;
      display:flex;gap:8px;padding:0 0 2px}
    .pe-joint-row{display:flex;align-items:center;gap:8px;padding:2px 0}
    .pe-joint-name{font-size:12px;color:#8b949e;flex:1;font-family:'Courier New',monospace;white-space:nowrap}
    .pe-joint-val{width:64px;background:#0d1117;border:1px solid #30363d;border-radius:4px;
      color:#e6edf3;font-family:'Courier New',monospace;font-size:12px;
      padding:3px 6px;text-align:right;outline:none;flex-shrink:0}
    .pe-joint-val:focus{border-color:#bc8cff}
    .pe-unit{font-size:11px;color:#6e7681;width:10px;flex-shrink:0}
    .pe-footer{display:flex;justify-content:flex-end;gap:8px;padding:10px 16px;
      border-top:1px solid #30363d;background:#21262d;border-radius:0 0 10px 10px;flex-shrink:0}
    .pe-cancel{background:none;border:1px solid #30363d;color:#6e7681;font-family:inherit;
      font-size:12px;padding:5px 14px;border-radius:4px;cursor:pointer}
    .pe-cancel:hover{border-color:#8b949e;color:#e6edf3}
    .pe-save{background:#2d1f3d;border:1px solid #bc8cff;color:#bc8cff;font-family:inherit;
      font-size:12px;padding:5px 14px;border-radius:4px;cursor:pointer}
    .pe-save:hover{background:#bc8cff;color:#0d1117}
    /* ── Behaviours panel ── */
    .behaviours-panel{background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden;flex:1;min-width:0}
    .behaviours-panel-hdr{padding:6px 12px;background:#21262d;font-size:11px;color:#8b949e;
      text-transform:uppercase;letter-spacing:1px;display:flex;align-items:center;gap:8px}
    .behaviours-list{display:flex;flex-wrap:wrap;gap:8px;padding:10px 12px;min-height:46px}
    .behaviours-empty{padding:2px 0;color:#6e7681;font-size:12px}
    .behaviour-item{display:flex;align-items:center;gap:6px;background:#0d1117;
      border:1px solid #30363d;border-radius:6px;padding:4px 10px;transition:border-color .2s}
    .behaviour-item.running{border-color:#3fb950;box-shadow:0 0 0 1px #3fb95030}
    .behaviour-item-name{font-size:12px;color:#c9d1d9;min-width:80px}
    .beh-run-btn{background:#1f2d1f;border:1px solid #3fb950;color:#3fb950;font-family:inherit;
      font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer}
    .beh-run-btn:hover:not(:disabled){background:#3fb950;color:#0d1117}
    .beh-run-btn:disabled{opacity:.4;cursor:default}
    .beh-stop-btn{background:#2d1f1f;border:1px solid #f85149;color:#f85149;font-family:inherit;
      font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer}
    .beh-stop-btn:hover{background:#f85149;color:#0d1117}
    .beh-new-btn{background:none;border:1px solid #30363d;color:#8b949e;font-family:inherit;
      font-size:11px;padding:2px 8px;border-radius:3px;cursor:pointer;margin-left:auto}
    .beh-new-btn:hover{border-color:#58a6ff;color:#58a6ff}
    /* ── Behaviour editor modal ── */
    #behEditorOverlay{display:none;position:fixed;inset:0;z-index:2000;
      background:rgba(0,0,0,.65);align-items:center;justify-content:center}
    #behEditorOverlay.open{display:flex}
    #behEditorBox{background:#161b22;border:1px solid #30363d;border-radius:10px;
      width:min(520px,96vw);max-height:88vh;display:flex;flex-direction:column;
      box-shadow:0 8px 32px rgba(0,0,0,.7)}
    .beh-ed-hdr{display:flex;justify-content:space-between;align-items:center;
      padding:10px 16px;border-bottom:1px solid #30363d;background:#21262d;
      border-radius:10px 10px 0 0;flex-shrink:0}
    .beh-ed-title{color:#e6edf3;font-size:14px;font-weight:bold}
    .beh-ed-close{background:none;border:none;color:#6e7681;font-size:18px;cursor:pointer;padding:0 4px;line-height:1}
    .beh-ed-close:hover{color:#f85149}
    .beh-ed-body{padding:12px 16px;overflow-y:auto;display:flex;flex-direction:column;gap:10px}
    .beh-ed-meta{display:flex;flex-direction:column;gap:6px;padding-bottom:10px;border-bottom:1px solid #21262d}
    .beh-ed-row{display:flex;align-items:center;gap:8px}
    .beh-ed-lbl{font-size:11px;color:#6e7681;width:80px;flex-shrink:0}
    .beh-ed-input{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:5px;
      color:#e6edf3;font-family:inherit;font-size:12px;padding:5px 8px;outline:none}
    .beh-ed-input:focus{border-color:#58a6ff}
    .beh-steps-hdr{display:flex;justify-content:space-between;align-items:center}
    .beh-steps-lbl{font-size:11px;color:#6e7681;text-transform:uppercase;letter-spacing:1px}
    .beh-add-btn{background:none;border:1px solid #30363d;color:#58a6ff;font-family:inherit;
      font-size:11px;padding:2px 10px;border-radius:3px;cursor:pointer}
    .beh-add-btn:hover{background:#1f2a3d;border-color:#58a6ff}
    #behEditorSteps{display:flex;flex-direction:column;gap:4px}
    .beh-step-row{display:flex;align-items:center;gap:5px;padding:4px 6px;
      background:#0d1117;border:1px solid #21262d;border-radius:5px}
    .beh-step-num{font-size:10px;color:#484f58;width:14px;flex-shrink:0;text-align:right}
    .beh-step-type{width:74px;background:#0d1117;border:1px solid #21262d;border-radius:3px;
      color:#8b949e;font-family:inherit;font-size:10px;padding:2px 3px;outline:none;flex-shrink:0}
    .beh-step-type:focus{border-color:#58a6ff}
    .beh-step-row.beh-type-behaviour{border-color:#7c3aed;background:#130d1e}
    .beh-step-sub-sel{flex:1;min-width:0;background:#0d1117;border:1px solid #7c3aed;border-radius:3px;
      color:#c792ea;font-family:inherit;font-size:11px;padding:2px 4px;outline:none}
    .beh-step-sub-sel:focus{border-color:#c792ea}
    .beh-step-fields{display:flex;align-items:center;gap:5px;flex:1;min-width:0}
    .beh-step-pose{flex:1;min-width:0;background:#0d1117;border:1px solid #21262d;border-radius:3px;
      color:#c9d1d9;font-family:inherit;font-size:11px;padding:2px 4px;outline:none}
    .beh-step-pose:focus{border-color:#58a6ff}
    .beh-step-mt{width:44px;background:#0d1117;border:1px solid #21262d;border-radius:3px;
      color:#e6edf3;font-family:'Courier New',monospace;font-size:11px;
      padding:2px 4px;text-align:right;outline:none;flex-shrink:0}
    .beh-step-mt:focus{border-color:#58a6ff}
    .beh-step-rep{width:32px;background:#0d1117;border:1px solid #21262d;border-radius:3px;
      color:#e6edf3;font-family:'Courier New',monospace;font-size:11px;
      padding:2px 4px;text-align:right;outline:none;flex-shrink:0}
    .beh-step-rep:focus{border-color:#58a6ff}
    .beh-step-speak-text{flex:1;min-width:0;background:#0d1117;border:1px solid #21262d;
      border-radius:3px;color:#c9d1d9;font-family:inherit;font-size:11px;padding:2px 4px;outline:none}
    .beh-step-speak-text:focus{border-color:#58a6ff}
    .beh-step-num-input{width:44px;background:#0d1117;border:1px solid #21262d;border-radius:3px;
      color:#e6edf3;font-family:'Courier New',monospace;font-size:11px;
      padding:2px 4px;text-align:right;outline:none;flex-shrink:0}
    .beh-step-num-input:focus{border-color:#58a6ff}
    .beh-step-lbl{font-size:10px;color:#484f58;flex-shrink:0}
    .beh-step-ord{background:none;border:none;color:#484f58;cursor:pointer;font-size:11px;
      padding:0 2px;line-height:1;flex-shrink:0}
    .beh-step-ord:hover:not(:disabled){color:#8b949e}
    .beh-step-ord:disabled{opacity:.2;cursor:default}
    .beh-step-del{background:none;border:none;color:#484f58;cursor:pointer;font-size:13px;
      padding:0 2px;line-height:1;flex-shrink:0}
    .beh-step-del:hover{color:#f85149}
    .beh-ed-footer{display:flex;justify-content:flex-end;gap:8px;padding:10px 16px;
      border-top:1px solid #30363d;background:#21262d;border-radius:0 0 10px 10px;flex-shrink:0}
    .beh-ed-cancel{background:none;border:1px solid #30363d;color:#6e7681;font-family:inherit;
      font-size:12px;padding:5px 14px;border-radius:4px;cursor:pointer}
    .beh-ed-cancel:hover{border-color:#8b949e;color:#e6edf3}
    .beh-ed-save{background:#1f2d1f;border:1px solid #3fb950;color:#3fb950;font-family:inherit;
      font-size:12px;padding:5px 14px;border-radius:4px;cursor:pointer}
    .beh-ed-save:hover{background:#3fb950;color:#0d1117}
    #savePoseOverlay{display:none;position:fixed;inset:0;z-index:2000;
      background:rgba(0,0,0,.65);align-items:center;justify-content:center}
    #savePoseOverlay.open{display:flex}
    #savePoseBox{background:#161b22;border:1px solid #30363d;border-radius:10px;
      width:min(400px,96vw);display:flex;flex-direction:column;
      box-shadow:0 8px 32px rgba(0,0,0,.7)}
    .sp-hdr{display:flex;justify-content:space-between;align-items:center;
      padding:10px 16px;border-bottom:1px solid #30363d;background:#21262d;
      border-radius:10px 10px 0 0}
    .sp-title{color:#e6edf3;font-size:14px;font-weight:bold}
    .sp-close{background:none;border:none;color:#6e7681;font-size:18px;
      cursor:pointer;padding:0 4px;line-height:1}
    .sp-close:hover{color:#f85149}
    .sp-body{padding:16px;display:flex;flex-direction:column;gap:10px}
    .sp-label{font-size:12px;color:#8b949e}
    .sp-input{width:100%;background:#0d1117;border:1px solid #30363d;border-radius:6px;
      color:#e6edf3;font-family:inherit;font-size:13px;padding:7px 10px;outline:none}
    .sp-input:focus{border-color:#bc8cff}
    .sp-footer{display:flex;justify-content:flex-end;gap:8px;
      padding:10px 16px;border-top:1px solid #30363d}
    .sp-save{background:#238636;color:#fff;border:none;border-radius:6px;
      padding:7px 20px;font-family:inherit;font-size:13px;cursor:pointer}
    .sp-save:hover{background:#2ea043}
    .sp-cancel{background:none;border:1px solid #30363d;color:#8b949e;
      border-radius:6px;padding:7px 16px;font-family:inherit;font-size:13px;cursor:pointer}
    .sp-cancel:hover{border-color:#8b949e;color:#c9d1d9}
  </style>
</head>
<body>
<header>
  <div class="title">&#x1F916; ROBBIE</div>
  <div id="status" class="status-badge disconnected"
       data-tip="Pipeline state&#10;listening  — idle, waiting for wake word&#10;recording  — capturing speech&#10;processing — STT + intent running&#10;speaking   — TTS active&#10;disconnected — WebSocket lost">&#x25CF; CONNECTING</div>
  <div class="hdr-right">
    <button class="danger-btn" onclick="stopAll()"
            data-tip="Stop all motion immediately&#10;Zeros cmd_vel, drive motors&#10;and voice pipeline">&#x23F9; Stop All</button>
    <button class="shutdown-btn" onclick="shutdownPc()"
            data-tip="Shut down the Raspberry Pi&#10;(requires confirmation)">&#x23FB; Shutdown</button>
    <label class="mute-wrap" title="Silent mode — text shown, robot stays quiet">
      <span class="mute-label">&#x1F507; Silent</span>
      <div class="toggle">
        <input type="checkbox" id="muteToggle" onchange="toggleMute()">
        <span class="slider"></span>
      </div>
    </label>
  </div>
</header>
<div class="tab-bar">
  <button class="tab-btn active" data-tab="control" onclick="showTab('control')">&#x2699; Control</button>
  <button class="tab-btn" data-tab="docs" onclick="showTab('docs')">&#x1F4DA; Docs</button>
  <button class="tab-btn" data-tab="diag" onclick="showTab('diag')">&#x2665; Diagnostics</button>
  <button class="tab-btn" data-tab="joints" onclick="showTab('joints')">&#x1F9BE; Joints</button>
  <button class="tab-btn" data-tab="logs" onclick="showTab('logs')">&#x1F4CB; Logs</button>
</div>
<div id="tab-control" class="tab-pane active">
<div class="infobar">
  <span id="clock">--:--:--</span>
  <span class="sep">|</span>
  <span id="batteryInfo" data-tip="Battery voltage&#10;Green  ≥11.5V  (good)&#10;Yellow 10.5–11.5V (low)&#10;Red    &lt;10.5V  (critical)">&#x1F50B; --.-V</span>
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
        <span class="lbl" data-tip="What Robbie heard&#10;(STT transcript or web injection)">Heard</span>
        <span id="lHeard" class="val">&mdash;</span>
        <span class="lbl" data-tip="Classified intent + parameters&#10;Determined by the intent classifier">Intent</span>
        <span id="lIntent" class="val">&mdash;</span>
        <span class="lbl" data-tip="Text sent to TTS&#10;🔇 badge = muted, not spoken">Response</span>
        <span id="lResponse" class="val">&mdash;</span>
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
  <div class="remap-panel">
    <div class="panel-hdr">&#x1F5FA; Map Update</div>
    <div class="remap-body">
      <button class="remap-btn" id="remapStartBtn" onclick="remapToggle()">&#x25B6; Start Remap</button>
      <button class="remap-save-btn" id="remapSaveBtn" onclick="remapSave()" disabled>&#x1F4BE; Save Map</button>
      <div class="remap-status" id="remapStatus">Drive robot around changed area, then save.</div>
    </div>
  </div>
  <div class="head-panel">
    <div class="panel-hdr">&#x1F4CF; Head Tilt</div>
    <div class="head-body">
      <span class="head-tilt-lbl">Up</span>
      <input type="range" class="head-tilt-slider" id="headTiltSlider"
             min="-0.6" max="0.6" step="0.01" value="0"
             oninput="setHeadTilt(this.value)">
      <span class="head-tilt-lbl">Down</span>
      <div class="head-tilt-ctr"><span id="headTiltVal">0.00</span> rad</div>
    </div>
  </div>
</div>
</div>
</div>
</div>
<div id="tab-docs" class="tab-pane">
  <div class="docs-layout">
    <div class="docs-sidebar">
      <div class="docs-sidebar-hdr">Blog</div>
      <a class="docs-hist-item" href="/blog/latest" target="_blank"
         style="display:block;text-decoration:none;color:#58a6ff">
        &#x1F4DD; Development Update
      </a>
      <div class="docs-sidebar-hdr" style="margin-top:8px">History</div>
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
      <div id="docViewer" class="doc-viewer">
        <div class="doc-viewer-hdr">
          <span id="docViewerTitle" class="doc-viewer-title"></span>
          <button class="doc-viewer-close" onclick="closeDocViewer()" title="Close">&#x2715;</button>
        </div>
        <div id="docViewerContent" class="docs-answer"></div>
      </div>
      <div id="cmdResults"></div>
    </div>
  </div>
</div>
<div id="tab-diag" class="tab-pane">
  <div class="diag-layout">
    <div class="diag-toolbar">
      <span class="diag-updated" id="diagUpdated">Waiting for data&hellip;</span>
    </div>
    <div class="diag-table" id="diagTable">
      <div class="diag-empty">Open this tab to start polling diagnostics&hellip;</div>
    </div>
  </div>
</div>
<div id="tab-joints" class="tab-pane">
  <div class="joints-layout">
    <div class="joints-toolbar">
      <button class="zero-btn" onclick="allToZero()" data-tip="Send all joints to 0°">&#x25A0; Zero</button>
      <button class="pose-action-btn" onclick="homeJoints()" data-tip="Send all joints to the home pose">&#x1F3E0; Home</button>
      <button class="mirror-btn" onclick="mirrorArm('right_to_left')" data-tip="Copy right arm to left (mirrored)">&#x21C6; R&#x2192;L</button>
      <button class="mirror-btn" onclick="mirrorArm('left_to_right')" data-tip="Copy left arm to right (mirrored)">&#x21C6; L&#x2192;R</button>
      <button class="save-pose-btn" onclick="openSavePose()" data-tip="Save all joint positions as a named YAML pose">&#x1F4BE; Save Pose</button>
      <button id="torqueBtn" class="torque-on-btn" onclick="toggleTorque()" data-tip="Toggle servo torque on/off">&#x26A1; Torque: ON</button>
      <div class="joints-speed-row">
        <span>Move time</span>
        <input type="range" id="jointSpeed" min="0.2" max="3.0" step="0.1" value="0.5"
               oninput="document.getElementById('jointSpeedVal').textContent=parseFloat(this.value).toFixed(1)+'s'">
        <span id="jointSpeedVal">0.5s</span>
      </div>
      <span class="joints-status" id="jointsStatus"></span>
    </div>
    <div class="joints-groups" id="jointsGroups"></div>
    <div class="poses-beh-row">
      <div class="poses-panel">
        <div class="poses-panel-hdr">&#x1F3AF; Saved Poses
          <span id="posesCount" style="font-weight:normal;color:#6e7681;font-size:10px"></span>
          <button class="beh-new-btn" onclick="openSavePose()" style="margin-left:auto">+ New</button>
        </div>
        <div class="posesel-row">
          <select id="poseSelect" class="posesel-select">
            <option value="">Loading&hellip;</option>
          </select>
          <button class="posesel-btn" onclick="runSelectedPose()">&#x25B6; Run</button>
          <button class="posesel-btn" onclick="previewSelectedPose()">&#x1F441;</button>
          <button class="posesel-btn" onclick="editSelectedPose()">&#x270F; Edit</button>
          <button class="posesel-btn posesel-del-btn" onclick="deleteSelectedPose()" title="Delete pose">&#x1F5D1;</button>
        </div>
      </div>
      <div class="behaviours-panel">
        <div class="behaviours-panel-hdr">&#x1F3AC; Behaviours
          <span id="behavioursCount" style="font-weight:normal;color:#6e7681;font-size:10px"></span>
          <button class="beh-new-btn" onclick="openBehaviourEditor(null)">+ New</button>
        </div>
        <div class="posesel-row">
          <select id="behSelect" class="posesel-select">
            <option value="">Loading&hellip;</option>
          </select>
          <button class="posesel-btn" id="behRunBtn" onclick="runSelectedBehaviour()">&#x25B6; Run</button>
          <button class="posesel-btn" id="behStopBtn" onclick="stopBehaviour()" style="display:none;color:#f85149;border-color:#f85149">&#x25FC; Stop</button>
          <button class="posesel-btn" onclick="editSelectedBehaviour()">&#x270F; Edit</button>
          <button class="posesel-btn posesel-del-btn" onclick="deleteSelectedBehaviour()" title="Delete behaviour">&#x1F5D1;</button>
        </div>
        <div class="posesel-prog" id="behProgress" style="display:none"></div>
      </div>
    </div>
  </div>
</div>
<div id="tab-logs" class="tab-pane">
  <div class="logs-layout">
    <div class="logs-toolbar">
      <button class="send-btn" onclick="generateLogsReport()">&#x1F50D; Generate Report</button>
      <span id="logsStatus" class="logs-status"></span>
    </div>
    <div id="logsReport" class="logs-report" style="color:#6e7681">
      Click Generate Report to analyse the latest ROS2 session logs&hellip;
    </div>
  </div>
</div>
<!-- Save pose modal -->
<div id="savePoseOverlay" onclick="if(event.target===this)closeSavePose()">
  <div id="savePoseBox">
    <div class="sp-hdr">
      <span class="sp-title">&#x1F4BE; Save Pose</span>
      <button class="sp-close" onclick="closeSavePose()">&#x2715;</button>
    </div>
    <div class="sp-body">
      <label class="sp-label">Pose name</label>
      <input id="spName" class="sp-input" type="text" placeholder="e.g. wave_ready" maxlength="40"
             onkeydown="if(event.key==='Enter')confirmSavePose()">
      <label class="sp-label">Description (optional)</label>
      <input id="spDesc" class="sp-input" type="text" placeholder="e.g. Right arm raised"
             onkeydown="if(event.key==='Enter')confirmSavePose()">
    </div>
    <div class="sp-footer">
      <button class="sp-cancel" onclick="closeSavePose()">Cancel</button>
      <button class="sp-save" onclick="confirmSavePose()">&#x1F4BE; Save</button>
    </div>
  </div>
</div>
<!-- Pose editor modal -->
<div id="poseEditorOverlay" onclick="if(event.target===this)closePoseEditor()">
  <div id="poseEditorBox">
    <div class="pe-hdr">
      <span class="pe-title">&#x270F; Edit Pose: <span id="peTitle"></span></span>
      <button class="pe-close" onclick="closePoseEditor()">&#x2715;</button>
    </div>
    <div class="pe-body">
      <div class="pe-meta">
        <div class="pe-row">
          <span class="pe-meta-label">Name</span>
          <input id="peName" class="pe-input" type="text" placeholder="pose name" maxlength="40">
        </div>
        <div class="pe-row">
          <span class="pe-meta-label">Description</span>
          <input id="peDesc" class="pe-input" type="text" placeholder="optional">
        </div>
        <div class="pe-row">
          <span class="pe-meta-label">Move time</span>
          <input id="peMoveTime" class="pe-mt-input" type="number" min="0.1" max="10" step="0.1" value="1.0">
          <span style="font-size:11px;color:#6e7681">s</span>
        </div>
      </div>
      <div id="peJoints"></div>
    </div>
    <div class="pe-footer">
      <button class="pe-cancel" onclick="closePoseEditor()">Cancel</button>
      <button class="pe-save" onclick="savePoseEditor()">&#x1F4BE; Save</button>
    </div>
  </div>
</div>
<!-- Behaviour editor modal -->
<div id="behEditorOverlay" onclick="if(event.target===this)closeBehaviourEditor()">
  <div id="behEditorBox">
    <div class="beh-ed-hdr">
      <span class="beh-ed-title" id="behEditorTitle">New Behaviour</span>
      <button class="beh-ed-close" onclick="closeBehaviourEditor()">&#x2715;</button>
    </div>
    <div class="beh-ed-body">
      <div class="beh-ed-meta">
        <div class="beh-ed-row">
          <span class="beh-ed-lbl">Name</span>
          <input id="behName" class="beh-ed-input" type="text" placeholder="e.g. wave" maxlength="40"
                 onkeydown="if(event.key==='Enter')document.getElementById('behDesc').focus()">
        </div>
        <div class="beh-ed-row">
          <span class="beh-ed-lbl">Description</span>
          <input id="behDesc" class="beh-ed-input" type="text" placeholder="optional">
        </div>
      </div>
      <div class="beh-steps-hdr">
        <span class="beh-steps-lbl">Steps</span>
        <button class="beh-add-btn" onclick="addBehStep()">+ Add Step</button>
      </div>
      <div id="behEditorSteps"></div>
      <div id="behNoSteps" style="font-size:12px;color:#6e7681;padding:4px 0">No steps — click Add Step.</div>
    </div>
    <div class="beh-ed-footer">
      <button class="beh-ed-cancel" onclick="closeBehaviourEditor()">Cancel</button>
      <button class="beh-ed-save" onclick="saveBehaviourEditor()">&#x1F4BE; Save</button>
    </div>
  </div>
</div>
<!-- Task context menu -->
<div id="taskCtxMenu">
  <div class="ctx-item" onclick="taskCtxView()">&#x1F4C4; View / Edit</div>
  <div class="ctx-item" onclick="taskCtxRun()">&#x25B6; Run</div>
</div>
<!-- Task editor modal -->
<div id="taskEditorOverlay" onclick="if(event.target===this)closeTaskEditor()">
  <div id="taskEditorBox">
    <div class="task-ed-hdr">
      <span id="taskEditorTitle" class="task-ed-title"></span>
      <button class="task-ed-close" onclick="closeTaskEditor()">&#x2715;</button>
    </div>
    <div class="task-ed-body">
      <div class="task-ed-help">One step per line &nbsp;&#x2014;&nbsp; format: <code>STEP_TYPE:arg1:arg2</code><br>
        e.g. <code>MakeSound:hello</code> &nbsp; <code>NavigateTo:kitchen</code> &nbsp; <code>Wait:2.0</code></div>
      <textarea id="taskEditorArea" rows="14" spellcheck="false"></textarea>
    </div>
    <div class="task-ed-footer">
      <button class="task-ed-cancel" onclick="closeTaskEditor()">Cancel</button>
      <button class="task-ed-save" onclick="saveTask()">&#x1F4BE; Save</button>
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
        case 'battery_voltage': {
          const v = d.voltage;
          const el = document.getElementById('batteryInfo');
          el.textContent = '🔋 ' + v.toFixed(1) + 'V';
          el.className = v >= 11.5 ? '' : v >= 10.5 ? 'low' : 'critical';
          break;
        }
        case 'task_update':
          handleTaskUpdate(d);
          break;
        case 'behaviour_progress':
          handleBehaviourProgress(d);
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

  function stopAll() {
    fetch('/api/stop_all', {method: 'POST'});
    addLog('STOP ALL sent', 'warn');
  }

  function shutdownPc() {
    if (!confirm('Shutdown the Raspberry Pi now?')) return;
    fetch('/api/shutdown', {method: 'POST'});
    addLog('Shutdown command sent', 'warn');
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
      btn.setAttribute('data-tip', 'Left-click to run\\nRight-click to view / edit');
      btn.onclick = () => runTask(name);
      btn.oncontextmenu = (e) => { e.preventDefault(); showTaskCtxMenu(e, name); };
      el.appendChild(btn);
      const step = document.createElement('div');
      step.className = 'task-step';
      step.id = 'step-' + name;
      el.appendChild(step);
    });
  }

  // ---- Task context menu ----
  let _ctxTask = null;

  function showTaskCtxMenu(e, name) {
    _ctxTask = name;
    const menu = document.getElementById('taskCtxMenu');
    menu.style.display = 'block';
    // Position near cursor but keep within viewport
    const x = Math.min(e.clientX, window.innerWidth  - menu.offsetWidth  - 8);
    const y = Math.min(e.clientY, window.innerHeight - menu.offsetHeight - 8);
    menu.style.left = x + 'px';
    menu.style.top  = y + 'px';
  }

  function hideTaskCtxMenu() {
    document.getElementById('taskCtxMenu').style.display = 'none';
  }

  document.addEventListener('click',   hideTaskCtxMenu);
  document.addEventListener('keydown', e => { if(e.key==='Escape') { hideTaskCtxMenu(); closeTaskEditor(); closeSavePose(); } });

  function taskCtxRun()  { hideTaskCtxMenu(); if(_ctxTask) runTask(_ctxTask); }
  function taskCtxView() { hideTaskCtxMenu(); if(_ctxTask) openTaskEditor(_ctxTask); }

  // ---- Task editor ----
  let _editingTask = null;

  async function openTaskEditor(name) {
    _editingTask = name;
    document.getElementById('taskEditorTitle').textContent = '\u270F\ufe0f  ' + name + '.txt';
    document.getElementById('taskEditorArea').value = 'Loading\u2026';
    document.getElementById('taskEditorOverlay').classList.add('open');
    try {
      const r = await fetch('/api/tasks/get?name=' + encodeURIComponent(name));
      const d = await r.json();
      document.getElementById('taskEditorArea').value = d.content || '';
    } catch(e) {
      document.getElementById('taskEditorArea').value = '# error loading file: ' + e;
    }
  }

  function closeTaskEditor() {
    document.getElementById('taskEditorOverlay').classList.remove('open');
    _editingTask = null;
  }

  async function saveTask() {
    if (!_editingTask) return;
    const content = document.getElementById('taskEditorArea').value;
    try {
      const r = await fetch('/api/tasks/save', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({name: _editingTask, content}),
      });
      const d = await r.json();
      if (d.status === 'ok') {
        closeTaskEditor();
        loadTasks();
      } else {
        alert('Save failed: ' + (d.error || 'unknown'));
      }
    } catch(e) {
      alert('Save failed: ' + e);
    }
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
  let _diagTimer   = null;
  let _jointsTimer = null;

  function showTab(name) {
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById('tab-' + name).classList.add('active');
    document.querySelector('[data-tab="' + name + '"]').classList.add('active');
    if (name === 'docs') loadDocsHistory();
    if (name === 'diag')   { fetchDiag(); if (!_diagTimer) _diagTimer = setInterval(fetchDiag, 2000); }
    else                   { clearInterval(_diagTimer);   _diagTimer   = null; }
    if (name === 'joints') { buildJointsUI(); fetchJointStates(); loadPoses(); loadBehaviours(); updateTorqueBtn(); if (!_jointsTimer) _jointsTimer = setInterval(fetchJointStates, 300); }
    else                   { clearInterval(_jointsTimer); _jointsTimer = null; }
  }

  // ---- Diagnostics polling ----
  async function fetchDiag() {
    try {
      const r = await fetch('/api/diagnostics');
      if (!r.ok) return;
      const d = await r.json();
      renderDiag(d.statuses || []);
    } catch(e) {}
  }

  function renderDiag(statuses) {
    const table = document.getElementById('diagTable');
    const upd   = document.getElementById('diagUpdated');
    if (!statuses.length) {
      table.innerHTML = '<div class="diag-empty">No diagnostic data — is health_monitor running?</div>';
      return;
    }
    const LEVEL_ORDER = [2, 1, 3, 0];
    const sorted = [...statuses].sort((a, b) => {
      return LEVEL_ORDER.indexOf(a.level) - LEVEL_ORDER.indexOf(b.level);
    });
    const LABELS = {0:'OK', 1:'WARN', 2:'ERROR', 3:'STALE'};
    const CSS    = {0:'diag-ok', 1:'diag-warn', 2:'diag-err', 3:'diag-stale'};
    table.innerHTML = sorted.map(s => {
      const cls   = CSS[s.level] || 'diag-stale';
      const label = LABELS[s.level] || 'STALE';
      const name  = esc(s.name);
      const msg   = esc(s.message);
      return '<div class="diag-row ' + cls + '">' +
               '<span class="diag-badge">' + label + '</span>' +
               '<span class="diag-name">' + name + '</span>' +
               '<span class="diag-msg">'  + msg  + '</span>' +
             '</div>';
    }).join('');
    const t = new Date().toTimeString().slice(0,8);
    upd.textContent = 'Updated ' + t;
  }

  // ---- Joint sliders ----
  const _JOINT_CFG = [
    { key: 'head',      label: 'Head',      joints: [
      {name:'head_yaw_joint',          lbl:'Yaw',        min:-60, max:60},
      {name:'head_roll',               lbl:'Roll',       min:-45, max:45},
      {name:'head_pitch',              lbl:'Pitch',      min:-35, max:35},
    ]},
    { key: 'right_arm', label: 'Right Arm', joints: [
      {name:'right_yaw_joint',         lbl:'Yaw',        min:-60, max:60},
      {name:'right_lift_joint',        lbl:'Lift',       min:-90, max:90},
      {name:'right_rotate_joint',      lbl:'Rotate',     min:-90, max:90},
      {name:'right_elbow_joint',       lbl:'Elbow',      min:-90, max:90},
      {name:'right_wrist_yaw_joint',   lbl:'Wrist Yaw',  min:-90, max:90},
      {name:'right_wrist_pitch_joint', lbl:'Wrist Pitch',min:-90, max:90},
    ]},
    { key: 'left_arm',  label: 'Left Arm',  joints: [
      {name:'left_yaw_joint',          lbl:'Yaw',        min:-60, max:60},
      {name:'left_lift_joint',         lbl:'Lift',       min:-90, max:90},
      {name:'left_rotate_joint',       lbl:'Rotate',     min:-90, max:90},
      {name:'left_elbow_joint',        lbl:'Elbow',      min:-90, max:90},
      {name:'left_wrist_yaw_joint',    lbl:'Wrist Yaw',  min:-90, max:90},
      {name:'left_wrist_pitch_joint',  lbl:'Wrist Pitch',min:-90, max:90},
    ]},
  ];
  const _jDebounce = {};
  let   _jBuilt    = false;

  async function buildJointsUI() {
    if (_jBuilt) return;
    _jBuilt = true;
    let limits = {};
    try {
      const r = await fetch('/api/joints/config');
      if (r.ok) { const d = await r.json(); limits = d.limits || {}; }
    } catch(e) {}
    const wrap = document.getElementById('jointsGroups');
    _JOINT_CFG.forEach(grp => {
      const col = document.createElement('div');
      col.className = 'joints-group';
      const hdr = document.createElement('div');
      hdr.className = 'joints-group-hdr';
      hdr.textContent = grp.label;
      col.appendChild(hdr);
      grp.joints.forEach(j => {
        const lim = limits[j.name] || {min: j.min, max: j.max};
        const row = document.createElement('div');
        row.className = 'joint-row';

        const lbl = document.createElement('span');
        lbl.className = 'joint-lbl';
        lbl.textContent = j.lbl;

        const inp = document.createElement('input');
        inp.type = 'range';
        inp.className = 'joint-range';
        inp.id = 'jr-' + j.name;
        inp.min = lim.min;  inp.max = lim.max;  inp.step = 1;  inp.value = 0;
        inp.addEventListener('input', (function(name){ return function() {
          onJointSlider(name, +this.value);
        }; })(j.name));

        const cmd = document.createElement('span');
        cmd.className = 'joint-cmd';
        cmd.id = 'jc-' + j.name;
        cmd.textContent = '0°';

        const act = document.createElement('span');
        act.className = 'joint-actual';
        act.id = 'ja-' + j.name;
        act.textContent = '—';

        row.appendChild(lbl); row.appendChild(inp);
        row.appendChild(cmd); row.appendChild(act);
        col.appendChild(row);
      });
      wrap.appendChild(col);
    });
  }

  function onJointSlider(name, val) {
    document.getElementById('jc-' + name).textContent = val + '°';
    clearTimeout(_jDebounce[name]);
    _jDebounce[name] = setTimeout(() => {
      const mt = parseFloat(document.getElementById('jointSpeed').value);
      fetch('/api/joints/command', {
        method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({joint: name, deg: val, move_time: mt})
      }).catch(() => {});
    }, 150);
  }

  async function fetchJointStates() {
    try {
      const r = await fetch('/api/joints/states');
      if (!r.ok) return;
      const d = await r.json();
      const states = d.states || {};
      Object.entries(states).forEach(([name, deg]) => {
        const el = document.getElementById('ja-' + name);
        if (el) el.textContent = deg.toFixed(1) + '°';
      });
      const s = document.getElementById('jointsStatus');
      if (s) s.textContent = 'Actual · ' + new Date().toTimeString().slice(0,8);
    } catch(e) {}
  }

  function allToZero() {
    const mt = parseFloat(document.getElementById('jointSpeed').value);
    _JOINT_CFG.forEach(grp => grp.joints.forEach(j => {
      const s = document.getElementById('jr-' + j.name);
      const c = document.getElementById('jc-' + j.name);
      if (s) s.value = 0;
      if (c) c.textContent = '0°';
    }));
    fetch('/api/joints/zero', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({move_time: mt})
    }).catch(() => {});
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
        const lines = buf.split('\\n');
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
                '<div class="docs-source" onclick="openDocViewer(' + JSON.stringify(s) + ')">'
                + '&#x1F4C4; ' + esc(s) + '</div>').join('');
              srcEl.style.display = ev.sources.length ? 'block' : 'none';
            } else if (ev.commands !== undefined) {
              cmdsEl.innerHTML = ev.commands.map(c =>
                '<button class="run-btn" data-id="' + c.id +
                '" data-label="' + esc(c.label) +
                '" onclick="runCmd(this.dataset.id,this.dataset.label)">' +
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

  // ---- Doc viewer ----
  async function openDocViewer(path) {
    const viewer  = document.getElementById('docViewer');
    const title   = document.getElementById('docViewerTitle');
    const content = document.getElementById('docViewerContent');
    title.textContent = path;
    content.innerHTML = '<span style="color:#8b949e">Loading\u2026</span>';
    viewer.style.display = 'block';
    try {
      const r = await fetch('/api/docs/file?path=' + encodeURIComponent(path));
      const d = await r.json();
      if (d.error) { content.textContent = '\u26a0 ' + d.error; return; }
      content.innerHTML = mdRender(d.content);
    } catch(e) {
      content.textContent = '\u26a0 ' + e;
    }
  }

  function closeDocViewer() {
    document.getElementById('docViewer').style.display = 'none';
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
      box.innerHTML =
        '<div class="cmd-result-hdr"><span>' + esc(label) + '</span>' +
        '<button class="stop-btn" id="stopbtn-' + id +
        '" data-id="' + id + '" onclick="stopCmd(this.dataset.id)">&#x25FC; Stop</button></div>' +
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

  // ---- Logs report ----
  let logsAbort = null;

  async function generateLogsReport() {
    const reportEl = document.getElementById('logsReport');
    const statusEl = document.getElementById('logsStatus');
    reportEl.innerHTML = '<span style="color:#8b949e">&#x23F3; Reading logs&hellip;</span>';
    statusEl.textContent = '';
    if (logsAbort) logsAbort.abort();
    logsAbort = new AbortController();
    try {
      const resp = await fetch('/api/logs/report', {signal: logsAbort.signal});
      if (!resp.ok) { reportEl.textContent = '\u26a0 Error: ' + resp.status; return; }
      const reader = resp.body.getReader();
      const dec = new TextDecoder();
      let buf = '', report = '';
      reportEl.innerHTML = '';
      while (true) {
        const {done, value} = await reader.read();
        if (done) break;
        buf += dec.decode(value, {stream: true});
        const lines = buf.split('\\n');
        buf = lines.pop();
        for (const line of lines) {
          if (!line.startsWith('data: ')) continue;
          const raw = line.slice(6);
          if (raw === '[DONE]') { statusEl.textContent = '\u2713 Done'; break; }
          try {
            const ev = JSON.parse(raw);
            if (ev.status !== undefined) {
              statusEl.textContent = ev.status;
            } else if (ev.token !== undefined) {
              report += ev.token;
              reportEl.innerHTML = mdRender(report);
              reportEl.scrollTop = reportEl.scrollHeight;
            }
          } catch(e) {}
        }
      }
    } catch(e) {
      if (e.name !== 'AbortError')
        reportEl.textContent = '\u26a0 ' + e;
    }
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
      .replace(/\\n/g,'<br>');
  }

  // ---- Pose functions ----
  async function homeJoints() {
    await runPose('home');
  }

  async function mirrorArm(direction) {
    try {
      const mt = parseFloat(document.getElementById('jointSpeed').value);
      await fetch('/api/arm/mirror', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({direction, move_time: mt})
      });
      const st = document.getElementById('jointsStatus');
      const lbl = direction === 'right_to_left' ? 'Mirrored R→L' : 'Mirrored L→R';
      if (st) st.textContent = lbl + ' \xb7 ' + new Date().toTimeString().slice(0,8);
      setTimeout(fetchJointStates, 1000);
    } catch(e) {}
  }

  let _torqueOn = true;

  function updateTorqueBtn() {
    const btn = document.getElementById('torqueBtn');
    if (!btn) return;
    if (_torqueOn) {
      btn.textContent = '⚡ Torque: ON';
      btn.className = 'torque-on-btn';
      btn.setAttribute('data-tip', 'Torque is ON — click to disable (arms go limp for manual posing)');
    } else {
      btn.textContent = '⚡ Torque: OFF';
      btn.className = 'torque-off-btn';
      btn.setAttribute('data-tip', 'Torque is OFF — click to enable (servos resume position control)');
    }
  }

  async function toggleTorque() {
    const newState = !_torqueOn;
    try {
      const r = await fetch('/api/arm/torque', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({enable: newState})
      });
      if (r.ok) {
        _torqueOn = newState;
        updateTorqueBtn();
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Torque ' + (newState ? 'ON' : 'OFF') + ' \xb7 ' + new Date().toTimeString().slice(0,8);
      }
    } catch(e) {}
  }

  function openSavePose() {
    document.getElementById('spName').value = '';
    document.getElementById('spDesc').value = '';
    document.getElementById('savePoseOverlay').classList.add('open');
    setTimeout(() => document.getElementById('spName').focus(), 50);
  }

  function closeSavePose() {
    document.getElementById('savePoseOverlay').classList.remove('open');
  }

  async function confirmSavePose() {
    const name = document.getElementById('spName').value.trim();
    const desc = document.getElementById('spDesc').value.trim();
    if (!name) { document.getElementById('spName').focus(); return; }
    try {
      const r = await fetch('/api/poses/save', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({name, description: desc})
      });
      const d = await r.json();
      if (d.status === 'ok') {
        closeSavePose();
        loadPoses();
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Saved "' + name + '"';
      } else {
        alert('Save failed: ' + (d.error || 'unknown'));
      }
    } catch(e) { alert('Save failed: ' + e); }
  }

  async function loadPoses() {
    try {
      const r = await fetch('/api/poses');
      if (!r.ok) return;
      const d = await r.json();
      const items = d.items || (d.poses || []).map(n => ({name: n, description: ''}));
      const sel = document.getElementById('poseSelect');
      const cnt = document.getElementById('posesCount');
      if (!sel) return;
      const prev = sel.value;
      sel.innerHTML = '';
      if (!items.length) {
        sel.innerHTML = '<option value="">(no poses saved)</option>';
        if (cnt) cnt.textContent = '';
      } else {
        items.forEach(({name, description}) => {
          const opt = document.createElement('option');
          opt.value = name;
          opt.textContent = description ? name + ' — ' + description : name;
          sel.appendChild(opt);
        });
        if (prev && Array.from(sel.options).some(o => o.value === prev)) sel.value = prev;
        if (cnt) cnt.textContent = '(' + items.length + ')';
      }
      _behPoseList = items.map(i => i.name);
    } catch(e) {}
  }

  function runSelectedPose()     { const s = document.getElementById('poseSelect');     if (s && s.value) runPose(s.value); }
  function editSelectedPose()    { const s = document.getElementById('poseSelect');     if (s && s.value) openPoseEditor(s.value); }
  function previewSelectedPose() { const s = document.getElementById('poseSelect');     if (s && s.value) previewPose(s.value); }

  async function deleteSelectedPose() {
    const s = document.getElementById('poseSelect');
    if (!s || !s.value) return;
    const name = s.value;
    if (!confirm('Delete pose "' + name + '"?')) return;
    try {
      const r = await fetch('/api/poses/delete', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({name})
      });
      const d = await r.json();
      if (d.status === 'ok') {
        loadPoses();
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Deleted "' + name + '"';
      } else {
        alert('Delete failed: ' + (d.error || 'unknown'));
      }
    } catch(e) { alert('Delete error: ' + e); }
  }

  async function deleteSelectedBehaviour() {
    const s = document.getElementById('behSelect');
    if (!s || !s.value) return;
    const name = s.value;
    if (!confirm('Delete behaviour "' + name + '"?')) return;
    try {
      const r = await fetch('/api/behaviours/delete', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({name})
      });
      const d = await r.json();
      if (d.status === 'ok') {
        loadBehaviours();
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Deleted "' + name + '"';
      } else {
        alert('Delete failed: ' + (d.error || 'unknown'));
      }
    } catch(e) { alert('Delete error: ' + e); }
  }
  function runSelectedBehaviour(){ const s = document.getElementById('behSelect');      if (s && s.value) runBehaviour(s.value); }
  function editSelectedBehaviour(){ const s = document.getElementById('behSelect');     if (s && s.value) openBehaviourEditor(s.value); }

  async function previewPose(name) {
    try {
      const r = await fetch('/api/poses/get?name=' + encodeURIComponent(name));
      if (!r.ok) return;
      const d = await r.json();
      Object.entries(d.joints || {}).forEach(([jname, deg]) => {
        const s = document.getElementById('jr-' + jname);
        const c = document.getElementById('jc-' + jname);
        if (s) s.value = Math.round(deg);
        if (c) c.textContent = Math.round(deg) + '\xb0';
      });
      const st = document.getElementById('jointsStatus');
      if (st) st.textContent = 'Preview: ' + name;
    } catch(e) {}
  }

  async function runPose(name) {
    try {
      const mt = parseFloat(document.getElementById('jointSpeed').value);
      await fetch('/api/poses/run', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({name, move_time: mt})
      });
      const st = document.getElementById('jointsStatus');
      if (st) st.textContent = 'Running: ' + name;
      setTimeout(fetchJointStates, 1500);
    } catch(e) {}
  }

  const JOINT_ORDER = [
    'head_yaw_joint','head_roll','head_pitch',
    'right_yaw_joint','right_lift_joint','right_rotate_joint','right_elbow_joint',
    'right_wrist_yaw_joint','right_wrist_pitch_joint',
    'left_yaw_joint','left_lift_joint','left_rotate_joint','left_elbow_joint',
    'left_wrist_yaw_joint','left_wrist_pitch_joint',
  ];

  let _poseEditorName = '';

  async function openPoseEditor(name) {
    try {
      const r = await fetch('/api/poses/get?name=' + encodeURIComponent(name));
      if (!r.ok) return;
      const d = await r.json();
      _poseEditorName = d.name || name;
      document.getElementById('peTitle').textContent = _poseEditorName;
      document.getElementById('peName').value = _poseEditorName;
      document.getElementById('peDesc').value = d.description || '';
      document.getElementById('peMoveTime').value = parseFloat(d.move_time || 1.0).toFixed(1);
      const joints = d.joints || {};
      const ordered = JOINT_ORDER.filter(j => j in joints);
      const rest = Object.keys(joints).filter(j => !JOINT_ORDER.includes(j)).sort();
      const container = document.getElementById('peJoints');
      container.innerHTML = '';
      [...ordered, ...rest].forEach(jname => {
        const row = document.createElement('div');
        row.className = 'pe-joint-row';
        const label = document.createElement('span');
        label.className = 'pe-joint-name';
        label.textContent = jname;
        const inp = document.createElement('input');
        inp.type = 'number';
        inp.className = 'pe-joint-val';
        inp.id = 'pj-' + jname;
        inp.value = parseFloat(joints[jname]).toFixed(1);
        inp.step = '0.5';
        inp.min = '-180';
        inp.max = '360';
        const unit = document.createElement('span');
        unit.className = 'pe-unit';
        unit.textContent = '\xb0';
        row.appendChild(label);
        row.appendChild(inp);
        row.appendChild(unit);
        container.appendChild(row);
      });
      document.getElementById('poseEditorOverlay').classList.add('open');
    } catch(e) {}
  }

  function closePoseEditor() {
    document.getElementById('poseEditorOverlay').classList.remove('open');
  }

  async function savePoseEditor() {
    const newName = document.getElementById('peName').value.trim();
    const desc = document.getElementById('peDesc').value.trim();
    const mt = parseFloat(document.getElementById('peMoveTime').value) || 1.0;
    if (!newName) { document.getElementById('peName').focus(); return; }
    const joints = {};
    document.querySelectorAll('[id^="pj-"]').forEach(el => {
      joints[el.id.slice(3)] = parseFloat(el.value);
    });
    const payload = {name: newName, description: desc, move_time: mt, joints};
    if (newName !== _poseEditorName) payload.old_name = _poseEditorName;
    try {
      const r = await fetch('/api/poses/save', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify(payload)
      });
      const d = await r.json();
      if (d.status === 'ok') {
        closePoseEditor();
        loadPoses();
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Saved “' + name + '”';
      } else {
        alert('Save failed: ' + (d.error || 'unknown'));
      }
    } catch(e) { alert('Save error: ' + e); }
  }

  // ---- Behaviours ----
  let _behPoseList = [];
  let _behBehaviourList = [];
  let _behRunningName = '';

  async function loadBehaviours() {
    try {
      const r = await fetch('/api/behaviours');
      if (!r.ok) return;
      const d = await r.json();
      const items = d.items || (d.behaviours || []).map(n => ({name: n, description: ''}));
      const sel = document.getElementById('behSelect');
      const cnt = document.getElementById('behavioursCount');
      if (!sel) return;
      const prev = sel.value;
      sel.innerHTML = '';
      if (!items.length) {
        sel.innerHTML = '<option value="">(no behaviours saved)</option>';
        if (cnt) cnt.textContent = '';
      } else {
        items.forEach(({name, description}) => {
          const opt = document.createElement('option');
          opt.value = name;
          opt.textContent = description ? name + ' — ' + description : name;
          sel.appendChild(opt);
        });
        if (prev && Array.from(sel.options).some(o => o.value === prev)) sel.value = prev;
        if (cnt) cnt.textContent = '(' + items.length + ')';
      }
      _behBehaviourList = items.map(i => i.name);
      if (_behRunningName) applyBehRunningState(_behRunningName, true, 0, 0);
    } catch(e) {}
  }

  async function runBehaviour(name) {
    try {
      const r = await fetch('/api/behaviours/run', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({name})
      });
      const d = await r.json();
      if (d.status !== 'ok') {
        const st = document.getElementById('jointsStatus');
        if (st) st.textContent = 'Error: ' + (d.error || 'run failed');
      }
    } catch(e) {}
  }

  async function stopBehaviour() {
    await fetch('/api/behaviours/stop', {method:'POST', headers:{'Content-Type':'application/json'}, body:'{}'});
  }

  function applyBehRunningState(name, running, step, total) {
    const runBtn  = document.getElementById('behRunBtn');
    const stopBtn = document.getElementById('behStopBtn');
    const prog    = document.getElementById('behProgress');
    const sel     = document.getElementById('behSelect');
    if (running) {
      if (runBtn)  runBtn.style.display  = 'none';
      if (stopBtn) stopBtn.style.display = 'inline-block';
      if (prog)  { prog.style.display = 'block'; prog.textContent = name + ': step ' + step + '/' + total; }
      if (sel)     sel.disabled = true;
    } else {
      if (runBtn)  runBtn.style.display  = 'inline-block';
      if (stopBtn) stopBtn.style.display = 'none';
      if (prog)    prog.style.display    = 'none';
      if (sel)     sel.disabled = false;
    }
  }

  function handleBehaviourProgress(d) {
    _behRunningName = d.running ? d.name : '';
    applyBehRunningState(d.name, d.running, d.step || 0, d.total || 0);
    const st = document.getElementById('jointsStatus');
    if (st) {
      if (d.running) st.textContent = 'Behaviour “' + d.name + '” step ' + d.step + '/' + d.total;
      else if (d.done) st.textContent = 'Behaviour done: ' + d.name;
      else if (d.cancelled) st.textContent = 'Behaviour stopped';
    }
  }

  // ---- Behaviour editor ----
  let _behEditorOrigName = '';

  async function openBehaviourEditor(name) {
    try {
      const [rp, rb] = await Promise.all([fetch('/api/poses'), fetch('/api/behaviours')]);
      const pd = await rp.json(); _behPoseList = pd.poses || [];
      const bd = await rb.json(); _behBehaviourList = (bd.items || bd.behaviours || []).map(i => typeof i === 'string' ? i : i.name);
    } catch(e) { _behPoseList = []; _behBehaviourList = []; }
    if (name) {
      try {
        const r = await fetch('/api/behaviours/get?name=' + encodeURIComponent(name));
        const d = await r.json();
        _behEditorOrigName = d.name || name;
        document.getElementById('behEditorTitle').textContent = 'Edit: ' + _behEditorOrigName;
        document.getElementById('behName').value = _behEditorOrigName;
        document.getElementById('behDesc').value = d.description || '';
        renderBehSteps(d.steps || []);
      } catch(e) { return; }
    } else {
      _behEditorOrigName = '';
      document.getElementById('behEditorTitle').textContent = 'New Behaviour';
      document.getElementById('behName').value = '';
      document.getElementById('behDesc').value = '';
      renderBehSteps([]);
    }
    document.getElementById('behEditorOverlay').classList.add('open');
    setTimeout(() => document.getElementById('behName').focus(), 50);
  }

  function closeBehaviourEditor() {
    document.getElementById('behEditorOverlay').classList.remove('open');
  }

  function makeBehStepFields(step) {
    const fields = document.createElement('span');
    fields.className = 'beh-step-fields';
    const type = step.type || 'pose';
    if (type === 'wait') {
      const sec = document.createElement('input');
      sec.type = 'number'; sec.className = 'beh-step-num-input';
      sec.value = parseFloat(step.seconds || 1.0).toFixed(1);
      sec.step = '0.5'; sec.min = '0.1'; sec.max = '60';
      const lbl = document.createElement('span');
      lbl.className = 'beh-step-lbl'; lbl.textContent = 's';
      fields.appendChild(sec); fields.appendChild(lbl);
    } else if (type === 'speak') {
      const txt = document.createElement('input');
      txt.type = 'text'; txt.className = 'beh-step-speak-text';
      txt.placeholder = 'text to speak'; txt.value = step.text || '';
      fields.appendChild(txt);
    } else if (type === 'behaviour') {
      const sel = document.createElement('select');
      sel.className = 'beh-step-sub-sel';
      if (step.behaviour && !_behBehaviourList.includes(step.behaviour)) {
        const opt = document.createElement('option');
        opt.value = step.behaviour; opt.textContent = step.behaviour; sel.appendChild(opt);
      }
      _behBehaviourList.forEach(b => {
        const opt = document.createElement('option');
        opt.value = b; opt.textContent = b;
        if (b === step.behaviour) opt.selected = true;
        sel.appendChild(opt);
      });
      if (sel.options.length === 0) {
        const opt = document.createElement('option');
        opt.value = ''; opt.textContent = '(no behaviours)'; sel.appendChild(opt);
      }
      fields.appendChild(sel);
    } else {
      const sel = document.createElement('select');
      sel.className = 'beh-step-pose';
      if (step.pose && !_behPoseList.includes(step.pose)) {
        const opt = document.createElement('option');
        opt.value = step.pose; opt.textContent = step.pose; sel.appendChild(opt);
      }
      _behPoseList.forEach(p => {
        const opt = document.createElement('option');
        opt.value = p; opt.textContent = p;
        if (p === step.pose) opt.selected = true;
        sel.appendChild(opt);
      });
      const mt = document.createElement('input');
      mt.type = 'number'; mt.className = 'beh-step-mt';
      mt.value = parseFloat(step.move_time || 1.0).toFixed(1);
      mt.step = '0.1'; mt.min = '0.1'; mt.max = '10';
      const mtLbl = document.createElement('span');
      mtLbl.className = 'beh-step-lbl'; mtLbl.textContent = 's';
      const rep = document.createElement('input');
      rep.type = 'number'; rep.className = 'beh-step-rep';
      rep.value = step.repeat || 1; rep.step = '1'; rep.min = '1'; rep.max = '20';
      const repLbl = document.createElement('span');
      repLbl.className = 'beh-step-lbl'; repLbl.textContent = '\xd7';
      fields.appendChild(sel); fields.appendChild(mt); fields.appendChild(mtLbl);
      fields.appendChild(rep); fields.appendChild(repLbl);
    }
    return fields;
  }

  function renderBehSteps(steps) {
    const container = document.getElementById('behEditorSteps');
    const noSteps   = document.getElementById('behNoSteps');
    container.innerHTML = '';
    if (noSteps) noSteps.style.display = steps.length ? 'none' : 'block';
    steps.forEach((step, i) => {
      const row = document.createElement('div');
      row.className = 'beh-step-row';
      const num = document.createElement('span');
      num.className = 'beh-step-num';
      num.textContent = i + 1;
      const typeSel = document.createElement('select');
      typeSel.className = 'beh-step-type';
      const _stepTypes = [
        {v:'pose',l:'pose'},{v:'wait',l:'wait'},{v:'speak',l:'speak'},
        {v:'behaviour',l:'behaviour',c:'#c792ea'}
      ];
      _stepTypes.forEach(({v,l,c}) => {
        const opt = document.createElement('option');
        opt.value = v; opt.textContent = l;
        if (c) opt.style.color = c;
        if ((step.type || 'pose') === v) opt.selected = true;
        typeSel.appendChild(opt);
      });
      const _applyRowType = (t) => {
        row.classList.toggle('beh-type-behaviour', t === 'behaviour');
        typeSel.style.color = t === 'behaviour' ? '#c792ea' : '';
      };
      _applyRowType(step.type || 'pose');
      let fields = makeBehStepFields(step);
      typeSel.addEventListener('change', () => {
        const newFields = makeBehStepFields({type: typeSel.value});
        row.replaceChild(newFields, fields);
        fields = newFields;
        _applyRowType(typeSel.value);
      });
      const upBtn = document.createElement('button');
      upBtn.className = 'beh-step-ord'; upBtn.textContent = '↑'; upBtn.disabled = (i === 0);
      upBtn.addEventListener('click', () => moveBehStep(i, -1));
      const dnBtn = document.createElement('button');
      dnBtn.className = 'beh-step-ord'; dnBtn.textContent = '↓'; dnBtn.disabled = (i === steps.length - 1);
      dnBtn.addEventListener('click', () => moveBehStep(i, +1));
      const delBtn = document.createElement('button');
      delBtn.className = 'beh-step-del'; delBtn.innerHTML = '&#x2715;';
      delBtn.addEventListener('click', () => deleteBehStep(i));
      row.appendChild(num); row.appendChild(typeSel); row.appendChild(fields);
      row.appendChild(upBtn); row.appendChild(dnBtn); row.appendChild(delBtn);
      container.appendChild(row);
    });
  }

  function readBehStepsFromDOM() {
    const steps = [];
    document.querySelectorAll('.beh-step-row').forEach(row => {
      const type = row.querySelector('.beh-step-type').value;
      if (type === 'wait') {
        steps.push({type: 'wait',
          seconds: parseFloat(row.querySelector('.beh-step-num-input').value) || 1.0});
      } else if (type === 'speak') {
        steps.push({type: 'speak',
          text: (row.querySelector('.beh-step-speak-text').value || '').trim()});
      } else if (type === 'behaviour') {
        const sub = row.querySelector('.beh-step-sub-sel');
        steps.push({type: 'behaviour', behaviour: sub ? sub.value : ''});
      } else {
        steps.push({
          pose:      row.querySelector('.beh-step-pose').value,
          move_time: parseFloat(row.querySelector('.beh-step-mt').value) || 1.0,
          repeat:    parseInt(row.querySelector('.beh-step-rep').value) || 1,
        });
      }
    });
    return steps;
  }

  function moveBehStep(i, delta) {
    const steps = readBehStepsFromDOM();
    const j = i + delta;
    if (j < 0 || j >= steps.length) return;
    [steps[i], steps[j]] = [steps[j], steps[i]];
    renderBehSteps(steps);
  }

  function deleteBehStep(i) {
    const steps = readBehStepsFromDOM();
    steps.splice(i, 1);
    renderBehSteps(steps);
  }

  function addBehStep() {
    const steps = readBehStepsFromDOM();
    steps.push({type: 'pose', pose: _behPoseList[0] || '', move_time: 1.0, repeat: 1});
    renderBehSteps(steps);
  }

  async function saveBehaviourEditor() {
    const name = document.getElementById('behName').value.trim();
    const desc = document.getElementById('behDesc').value.trim();
    if (!name) { document.getElementById('behName').focus(); return; }
    const steps = readBehStepsFromDOM();
    const payload = {name, description: desc, steps};
    if (_behEditorOrigName && name !== _behEditorOrigName) payload.old_name = _behEditorOrigName;
    try {
      const r = await fetch('/api/behaviours/save', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify(payload)
      });
      const d = await r.json();
      if (d.status === 'ok') {
        closeBehaviourEditor();
        loadBehaviours();
      } else {
        alert('Save failed: ' + (d.error || 'unknown'));
      }
    } catch(e) { alert('Save error: ' + e); }
  }

  let _remapping = false;
  function remapToggle() {
    _remapping = !_remapping;
    const startBtn = document.getElementById('remapStartBtn');
    const saveBtn  = document.getElementById('remapSaveBtn');
    const status   = document.getElementById('remapStatus');
    if (_remapping) {
      startBtn.textContent = '\u23F9 Stop Remap';
      startBtn.classList.add('active');
      saveBtn.disabled = false;
      status.textContent = 'Remapping \u2014 drive around the changed area\u2026';
      status.style.color = '#3fb950';
    } else {
      startBtn.textContent = '\u25B6 Start Remap';
      startBtn.classList.remove('active');
      saveBtn.disabled = true;
      status.textContent = 'Drive robot around changed area, then save.';
      status.style.color = '#6e7681';
    }
  }
  function setHeadTilt(val) {
    document.getElementById('headTiltVal').textContent = parseFloat(val).toFixed(2);
    fetch('/api/head/tilt', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({tilt: parseFloat(val)})
    });
  }

  async function remapSave() {
    const saveBtn = document.getElementById('remapSaveBtn');
    const status  = document.getElementById('remapStatus');
    saveBtn.disabled = true;
    status.textContent = 'Saving map\u2026';
    status.style.color = '#e3b341';
    try {
      const r = await fetch('/api/slam/save_map', {method:'POST'});
      const d = await r.json();
      if (d.status === 'ok') {
        status.textContent = '\u2713 Map saved';
        status.style.color = '#3fb950';
      } else {
        status.textContent = '\u2717 Save failed: ' + (d.message || 'unknown error');
        status.style.color = '#f85149';
        saveBtn.disabled = false;
        return;
      }
    } catch(e) {
      status.textContent = '\u2717 ' + e;
      status.style.color = '#f85149';
      saveBtn.disabled = false;
      return;
    }
    setTimeout(() => {
      _remapping = false;
      document.getElementById('remapStartBtn').textContent = '\u25B6 Start Remap';
      document.getElementById('remapStartBtn').classList.remove('active');
      document.getElementById('remapSaveBtn').disabled = true;
      document.getElementById('remapStatus').textContent = 'Drive robot around changed area, then save.';
      document.getElementById('remapStatus').style.color = '#6e7681';
    }, 4000);
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
        self._uvicorn_server = None

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

    def _collect_ros_logs(self, max_chars: int = 12000) -> str:
        """Collect the latest ROS2 session logs into a single text blob."""
        import os
        import time

        log_dir = os.path.expanduser("~/.ros/log")
        parts: list[str] = []

        # 1. Launch log from the latest session directory
        launch_log = os.path.join(log_dir, "latest", "launch.log")
        if os.path.exists(launch_log):
            try:
                with open(launch_log) as f:
                    parts.append("=== launch.log ===\n" + f.read())
            except OSError:
                pass

        # 2. Individual node logs — files in log_dir modified in the last 4 hours,
        #    sorted newest-first so the most recent activity is included first when
        #    the combined output is trimmed to max_chars.
        cutoff = time.time() - 4 * 3600
        node_logs = sorted(
            [
                os.path.join(log_dir, fn)
                for fn in os.listdir(log_dir)
                if fn.endswith(".log") and
                os.path.isfile(os.path.join(log_dir, fn)) and
                os.path.getmtime(os.path.join(log_dir, fn)) >= cutoff
            ],
            key=os.path.getmtime,
            reverse=True,  # newest first — ensures recent logs survive truncation
        )

        for path in node_logs:
            try:
                with open(path) as f:
                    lines = f.readlines()
                # Keep last 50 lines (latest activity) + first 10 lines (startup)
                if len(lines) > 60:
                    excerpt = lines[-50:] + ["...\n"] + lines[:10]
                else:
                    excerpt = lines
                name = os.path.basename(path)
                parts.append(f"\n=== {name} ===\n" + "".join(excerpt))
            except OSError:
                pass

        combined = "\n".join(parts)
        if len(combined) > max_chars:
            combined = combined[:max_chars] + "\n...(truncated)"
        return combined

    def _parse_urdf_limits(self) -> dict:
        """Parse joint limits from thor_robot.urdf.xacro; return {name: {min, max}} in degrees."""
        import math
        import xml.etree.ElementTree as ET
        urdf = os.path.join(
            os.path.dirname(__file__),
            '../robbie_description/urdf/thor_robot.urdf.xacro',
        )
        try:
            root = ET.parse(urdf).getroot()
        except Exception as e:
            logger.warning(f"URDF parse failed: {e}")
            return {}
        out = {}
        for joint in root.iter('joint'):
            if joint.get('type') != 'revolute':
                continue
            name = joint.get('name')
            lim  = joint.find('limit')
            if not name or lim is None:
                continue
            try:
                out[name] = {
                    'min': round(math.degrees(float(lim.get('lower', 0)))),
                    'max': round(math.degrees(float(lim.get('upper', 0)))),
                }
            except (ValueError, TypeError):
                pass
        return out

    async def stop(self):
        """Gracefully stop the web server and kill any running subprocesses."""
        for proc in list(self._cmd_procs.values()):
            try:
                proc.kill()
            except Exception:
                pass
        self._cmd_procs.clear()
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True

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
            from fastapi.responses import HTMLResponse as HR
            return HR(_HTML, headers={"Cache-Control": "no-store"})

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

        @app.get("/api/tasks/get")
        async def api_tasks_get(name: str = ""):
            """Return the raw content of a task .txt file."""
            tr = (self._voice_server and
                  getattr(self._voice_server, '_task_runner', None))
            if not tr or not name:
                return {"error": "not available"}
            import re
            if not re.match(r'^[\w\-. ]+$', name):
                return {"error": "invalid name"}
            path = tr._tasks_dir / f"{name}.txt"
            if not path.exists():
                return {"error": "not found"}
            return {"content": path.read_text(encoding="utf-8", errors="replace")}

        @app.post("/api/tasks/save")
        async def api_tasks_save(body: dict[str, Any]):
            """Save edited content back to a task .txt file."""
            tr = (self._voice_server and
                  getattr(self._voice_server, '_task_runner', None))
            name    = str(body.get("name", "")).strip()
            content = str(body.get("content", ""))
            if not tr or not name:
                return {"status": "error", "error": "not available"}
            import re
            if not re.match(r'^[\w\-. ]+$', name):
                return {"status": "error", "error": "invalid name"}
            path = tr._tasks_dir / f"{name}.txt"
            # Only allow saving existing task files (no path traversal, no new files)
            if not path.exists():
                return {"status": "error", "error": "task not found"}
            path.write_text(content, encoding="utf-8")
            logger.info(f"Task saved via web editor: {name}.txt")
            return {"status": "ok"}

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

        @app.get("/blog/latest")
        async def blog_latest():
            """Serve the latest blog post HTML."""
            import glob as _glob
            from fastapi.responses import HTMLResponse
            pattern = os.path.join(
                os.path.dirname(__file__),
                "../robbie_bot/blog/*.html"
            )
            files = sorted(_glob.glob(pattern))
            if not files:
                return HTMLResponse("<p>No blog post found.</p>", status_code=404)
            with open(files[-1], "r") as f:
                return HTMLResponse(f.read())

        @app.get("/api/docs/history")
        async def api_docs_history():
            return {"entries": self._docs.load_history(limit=20)}

        @app.get("/api/docs/list")
        async def api_docs_list():
            """Return sorted list of all indexed document paths."""
            return {"docs": self._docs.list_docs()}

        @app.get("/api/docs/file")
        async def api_docs_file(path: str = ""):
            """Return the raw markdown content of a doc file by relative path."""
            if not path:
                return {"error": "no path specified"}
            content = self._docs.read_doc(path)
            if content is None:
                return {"error": "file not found or access denied"}
            return {"path": path, "content": content}

        @app.get("/api/logs/report")
        async def api_logs_report():
            """SSE stream: read latest ROS2 logs and stream an LLM diagnostic report."""
            from fastapi.responses import StreamingResponse as SR

            async def generate():
                yield f"data: {json.dumps({'status': 'Reading logs…'})}\n\n"

                log_text = await asyncio.to_thread(self._collect_ros_logs)
                if not log_text.strip():
                    yield f"data: {json.dumps({'token': 'No ROS2 log files found.'})}\n\n"
                    yield "data: [DONE]\n\n"
                    return

                yield f"data: {json.dumps({'status': 'Analysing with LLM…'})}\n\n"

                system_prompt = (
                    "/no_think "
                    "You are a robot systems diagnostics analyst. "
                    "Analyse the provided ROS2 log output and produce a concise report. "
                    "Use markdown with these sections: "
                    "## Nodes Launched, ## Errors, ## Warnings, ## Summary. "
                    "In Errors and Warnings list each unique issue once with a brief explanation. "
                    "In Summary give a one-paragraph overall health assessment."
                )
                messages = [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"ROS2 logs:\n\n{log_text}"},
                ]

                try:
                    import ollama
                    client = ollama.AsyncClient(host="http://10.0.0.87:11434")
                    async for chunk in await client.chat(
                        model="qwen3:4b",
                        messages=messages,
                        stream=True,
                        options={"num_predict": 1024},
                    ):
                        token = chunk.message.content
                        if token:
                            yield f"data: {json.dumps({'token': token})}\n\n"
                except Exception as e:
                    yield f"data: {json.dumps({'token': f'LLM error: {e}'})}\n\n"

                yield "data: [DONE]\n\n"

            return SR(
                generate(),
                media_type="text/event-stream",
                headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
            )

        @app.post("/api/head/tilt")
        async def api_head_tilt(body: dict[str, Any]):
            tilt = float(body.get("tilt", 0.0))
            tilt = max(-0.6, min(0.6, tilt))
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if dispatcher:
                dispatcher.publish_head(0.0, tilt)
            return {"status": "ok"}

        @app.post("/api/stop_all")
        async def api_stop_all():
            """Stop all robot motion immediately."""
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if dispatcher:
                dispatcher.publish_stop()
            return {"status": "ok"}

        @app.post("/api/shutdown")
        async def api_shutdown():
            """Shutdown the Raspberry Pi."""
            asyncio.create_task(asyncio.to_thread(
                subprocess.run, ["sudo", "shutdown", "-h", "now"]
            ))
            return {"status": "shutting down"}

        @app.post("/api/slam/save_map")
        async def api_slam_save_map():
            """Save the SLAM pose graph back to the map file."""
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error", "message": "dispatcher not available"}
            ok = await asyncio.to_thread(dispatcher.slam_serialize_map)
            if ok:
                return {"status": "ok"}
            return {"status": "error", "message": "service call failed or timed out"}

        @app.get("/api/diagnostics")
        async def api_diagnostics():
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"statuses": []}
            return {"statuses": dispatcher.get_all_diagnostics()}

        @app.get("/api/joints/config")
        async def api_joints_config():
            """Return joint limits in degrees parsed from the URDF."""
            return {"limits": await asyncio.to_thread(self._parse_urdf_limits)}

        @app.get("/api/joints/states")
        async def api_joints_states():
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"states": {}}
            return {"states": dispatcher.get_joint_states()}

        @app.post("/api/joints/command")
        async def api_joints_command(body: dict[str, Any]):
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error", "message": "dispatcher not ready"}
            joint = str(body.get("joint", "")).strip()
            deg   = float(body.get("deg", 0.0))
            mt    = max(0.1, min(5.0, float(body.get("move_time", 0.5))))
            ok = dispatcher.send_joint_position(joint, deg, mt)
            return {"status": "ok" if ok else "error"}

        @app.post("/api/joints/zero")
        async def api_joints_zero(body: dict[str, Any]):
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error"}
            mt = max(0.1, min(5.0, float(body.get("move_time", 1.0))))
            dispatcher.send_all_to_zero(mt)
            return {"status": "ok"}

        # ---- Pose endpoints ----
        _poses_dir = pathlib.Path(__file__).parent.parent / 'robbie_bot' / 'poses'
        _poses_dir.mkdir(parents=True, exist_ok=True)

        @app.get("/api/poses")
        async def api_poses():
            import yaml as _yaml
            items = []
            for p in sorted(_poses_dir.glob('*.yaml')):
                try:
                    data = _yaml.safe_load(p.read_text()) or {}
                    items.append({"name": p.stem, "description": data.get("description", "")})
                except Exception:
                    items.append({"name": p.stem, "description": ""})
            return {"poses": [i["name"] for i in items], "items": items}

        @app.get("/api/poses/get")
        async def api_poses_get(name: str = ""):
            import re, yaml
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"error": "invalid name"}
            path = _poses_dir / f"{name}.yaml"
            if not path.exists():
                return {"error": "not found"}
            try:
                data = yaml.safe_load(path.read_text())
                return {"name": data.get("name", name),
                        "joints": data.get("joints", {}),
                        "move_time": data.get("move_time", 1.0),
                        "description": data.get("description", "")}
            except Exception as e:
                return {"error": str(e)}

        @app.post("/api/poses/save")
        async def api_poses_save(body: dict[str, Any]):
            import re, yaml
            name = str(body.get("name", "")).strip()
            desc = str(body.get("description", "")).strip()
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name (alphanumeric, hyphens, spaces only)"}
            explicit_joints = body.get("joints")
            if explicit_joints and isinstance(explicit_joints, dict):
                joints_data = {k: round(float(v), 2) for k, v in explicit_joints.items()}
                move_time = max(0.1, min(10.0, float(body.get("move_time", 1.0))))
            else:
                dispatcher = (self._voice_server and
                              getattr(self._voice_server, '_dispatcher', None))
                if not dispatcher:
                    return {"status": "error", "error": "dispatcher not ready"}
                states = dispatcher.get_joint_states()
                if not states:
                    return {"status": "error", "error": "no joint states received yet"}
                joints_data = {k: round(v, 2) for k, v in sorted(states.items())}
                move_time = 1.0
            pose = {
                "name": name,
                "description": desc,
                "move_time": move_time,
                "joints": joints_data,
            }
            path = _poses_dir / f"{name}.yaml"
            path.write_text(yaml.dump(pose, default_flow_style=False, sort_keys=False))
            old_name = str(body.get("old_name", "")).strip()
            if old_name and old_name != name:
                old_path = _poses_dir / f"{old_name}.yaml"
                if old_path.exists():
                    old_path.unlink()
            logger.info(f"Pose saved: {path}")
            return {"status": "ok"}

        @app.post("/api/poses/run")
        async def api_poses_run(body: dict[str, Any]):
            import re, yaml
            name = str(body.get("name", "")).strip()
            mt   = max(0.1, min(10.0, float(body.get("move_time", 1.0))))
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name"}
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error", "error": "dispatcher not ready"}
            path = _poses_dir / f"{name}.yaml"
            if not path.exists():
                return {"status": "error", "error": "pose not found"}
            try:
                data = yaml.safe_load(path.read_text())
                joints = data.get("joints", {})
                file_mt = float(data.get("move_time", 1.0))
                dispatcher.send_all_joints(joints, mt if mt != 1.0 else file_mt)
                return {"status": "ok"}
            except Exception as e:
                logger.error(f"api_poses_run: {e}")
                return {"status": "error", "error": str(e)}

        @app.post("/api/poses/delete")
        async def api_poses_delete(body: dict[str, Any]):
            import re
            name = str(body.get("name", "")).strip()
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name"}
            path = _poses_dir / f"{name}.yaml"
            if not path.exists():
                return {"status": "error", "error": "not found"}
            path.unlink()
            logger.info(f"Pose deleted: {name}")
            return {"status": "ok"}

        @app.post("/api/arm/mirror")
        async def api_arm_mirror(body: dict[str, Any]):
            direction = str(body.get("direction", "")).strip()
            mt = max(0.1, min(5.0, float(body.get("move_time", 1.0))))
            if direction not in ("right_to_left", "left_to_right"):
                return {"status": "error", "error": "direction must be right_to_left or left_to_right"}
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error", "error": "dispatcher not ready"}
            dispatcher.mirror_arm(direction, mt)
            return {"status": "ok"}

        @app.post("/api/arm/torque")
        async def api_arm_torque(body: dict[str, Any]):
            enable = bool(body.get("enable", True))
            dispatcher = (self._voice_server and
                          getattr(self._voice_server, '_dispatcher', None))
            if not dispatcher:
                return {"status": "error", "error": "dispatcher not ready"}
            dispatcher.set_servo_torque(enable)
            return {"status": "ok", "torque": enable}

        # ---- Behaviour endpoints ----
        _beh_dir = pathlib.Path(__file__).parent.parent / 'robbie_bot' / 'behaviours'
        _beh_dir.mkdir(parents=True, exist_ok=True)
        _beh_state: dict = {"task": None}

        async def _run_behaviour_coro(name: str, steps: list, dispatcher):
            import yaml as _yaml

            async def _exec_step(step: dict, depth: int = 0):
                step_type = str(step.get("type", "pose"))
                if step_type == "wait":
                    await asyncio.sleep(max(0.05, float(step.get("seconds", 1.0))))
                elif step_type == "speak":
                    text = str(step.get("text", "")).strip()
                    if text and self._voice_server:
                        await self._voice_server._speak(text)
                elif step_type == "behaviour":
                    if depth >= 5:
                        logger.warning(f"Behaviour nesting too deep, skipping sub-behaviour")
                        return
                    sub_name = str(step.get("behaviour", "")).strip()
                    if not sub_name:
                        return
                    sub_path = _beh_dir / f"{sub_name}.yaml"
                    if not sub_path.exists():
                        logger.warning(f"Sub-behaviour '{sub_name}' not found")
                        return
                    sub_data = _yaml.safe_load(sub_path.read_text()) or {}
                    for sub_step in sub_data.get("steps", []):
                        if isinstance(sub_step, dict):
                            await _exec_step(sub_step, depth + 1)
                else:  # pose
                    pose_name = str(step.get("pose", "")).strip()
                    mt = max(0.1, float(step.get("move_time", 1.0)))
                    repeat = max(1, int(step.get("repeat", 1)))
                    path = _poses_dir / f"{pose_name}.yaml"
                    if not path.exists():
                        logger.warning(f"Pose '{pose_name}' not found, skipping")
                        return
                    data = _yaml.safe_load(path.read_text()) or {}
                    joints = data.get("joints", {})
                    for _ in range(repeat):
                        dispatcher.send_all_joints(joints, mt)
                        await asyncio.sleep(mt + 0.1)

            total = len(steps)
            try:
                for i, step in enumerate(steps):
                    if not isinstance(step, dict):
                        logger.warning(f"Behaviour '{name}' step {i+1}: not a dict, skipping")
                        continue
                    step_type = str(step.get("type", "pose"))
                    await self.broadcast({"type": "behaviour_progress", "name": name,
                                          "step": i + 1, "total": total, "running": True})
                    try:
                        await _exec_step(step)
                    except asyncio.CancelledError:
                        raise
                    except Exception as e:
                        logger.error(f"Behaviour '{name}' step {i+1} ({step_type}) error: {e}", exc_info=True)

                await self.broadcast({"type": "behaviour_progress", "name": name,
                                      "step": total, "total": total, "running": False, "done": True})
            except asyncio.CancelledError:
                await self.broadcast({"type": "behaviour_progress", "name": name,
                                      "running": False, "cancelled": True})
                raise
            except Exception as e:
                logger.error(f"Behaviour '{name}' runner fatal error: {e}", exc_info=True)
                await self.broadcast({"type": "behaviour_progress", "name": name,
                                      "running": False, "error": str(e)})

        @app.get("/api/behaviours")
        async def api_behaviours():
            import yaml as _yaml
            items = []
            for p in sorted(_beh_dir.glob('*.yaml')):
                try:
                    data = _yaml.safe_load(p.read_text()) or {}
                    items.append({"name": p.stem, "description": data.get("description", "")})
                except Exception:
                    items.append({"name": p.stem, "description": ""})
            return {"behaviours": [i["name"] for i in items], "items": items}

        @app.get("/api/behaviours/get")
        async def api_behaviours_get(name: str = ""):
            import re, yaml
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"error": "invalid name"}
            path = _beh_dir / f"{name}.yaml"
            if not path.exists():
                return {"error": "not found"}
            try:
                data = yaml.safe_load(path.read_text())
                return {"name": data.get("name", name),
                        "description": data.get("description", ""),
                        "steps": data.get("steps", [])}
            except Exception as e:
                return {"error": str(e)}

        @app.post("/api/behaviours/save")
        async def api_behaviours_save(body: dict[str, Any]):
            import re, yaml
            name = str(body.get("name", "")).strip()
            desc = str(body.get("description", "")).strip()
            steps_raw = body.get("steps", [])
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name"}
            if not isinstance(steps_raw, list):
                return {"status": "error", "error": "steps must be a list"}
            clean_steps = []
            for s in steps_raw:
                if not isinstance(s, dict):
                    continue
                step_type = str(s.get("type", "pose"))
                if step_type == "wait":
                    clean_steps.append({"type": "wait",
                                        "seconds": round(max(0.05, float(s.get("seconds", 1.0))), 2)})
                elif step_type == "speak":
                    text = str(s.get("text", "")).strip()
                    if not text:
                        continue
                    clean_steps.append({"type": "speak", "text": text})
                elif step_type == "behaviour":
                    sub_name = str(s.get("behaviour", "")).strip()
                    if not sub_name:
                        continue
                    clean_steps.append({"type": "behaviour", "behaviour": sub_name})
                else:  # pose
                    pose_name = str(s.get("pose", "")).strip()
                    if not pose_name:
                        continue
                    step: dict = {"pose": pose_name,
                                  "move_time": round(max(0.1, float(s.get("move_time", 1.0))), 2)}
                    repeat = max(1, int(s.get("repeat", 1)))
                    if repeat > 1:
                        step["repeat"] = repeat
                    clean_steps.append(step)
            beh = {"name": name, "description": desc, "steps": clean_steps}
            path = _beh_dir / f"{name}.yaml"
            path.write_text(yaml.dump(beh, default_flow_style=False, sort_keys=False))
            old_name = str(body.get("old_name", "")).strip()
            if old_name and old_name != name:
                old_path = _beh_dir / f"{old_name}.yaml"
                if old_path.exists():
                    old_path.unlink()
            logger.info(f"Behaviour saved: {path}")
            return {"status": "ok"}

        @app.post("/api/behaviours/delete")
        async def api_behaviours_delete(body: dict[str, Any]):
            import re
            name = str(body.get("name", "")).strip()
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name"}
            path = _beh_dir / f"{name}.yaml"
            if not path.exists():
                return {"status": "error", "error": "not found"}
            path.unlink()
            logger.info(f"Behaviour deleted: {name}")
            return {"status": "ok"}

        @app.post("/api/behaviours/run")
        async def api_behaviours_run(body: dict[str, Any]):
            import re, yaml
            name = str(body.get("name", "")).strip()
            if not name or not re.match(r'^[\w\- ]+$', name):
                return {"status": "error", "error": "invalid name"}
            task = _beh_state["task"]
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass
            disp = (self._voice_server and getattr(self._voice_server, '_dispatcher', None))
            if not disp:
                return {"status": "error", "error": "dispatcher not ready"}
            path = _beh_dir / f"{name}.yaml"
            if not path.exists():
                return {"status": "error", "error": "behaviour not found"}
            data = yaml.safe_load(path.read_text())
            steps = data.get("steps", [])
            if not steps:
                return {"status": "error", "error": "behaviour has no steps"}
            task = asyncio.create_task(_run_behaviour_coro(name, steps, disp))

            def _beh_done(t):
                exc = t.exception() if not t.cancelled() else None
                if exc:
                    logger.error(f"Behaviour '{name}' task raised: {exc}", exc_info=exc)

            task.add_done_callback(_beh_done)
            _beh_state["task"] = task
            return {"status": "ok"}

        @app.post("/api/behaviours/stop")
        async def api_behaviours_stop():
            task = _beh_state.get("task")
            if task and not task.done():
                task.cancel()
            return {"status": "ok"}

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
            # Send current battery voltage
            try:
                dispatcher = (self._voice_server and
                              getattr(self._voice_server, '_dispatcher', None))
                if dispatcher:
                    v = dispatcher.get_battery_voltage()
                    if v is not None:
                        await websocket.send_text(
                            json.dumps({"type": "battery_voltage", "voltage": v})
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

        async def _battery_push_loop():
            """Push battery voltage to all clients every 10 s."""
            while True:
                await asyncio.sleep(10)
                dispatcher = (self._voice_server and
                              getattr(self._voice_server, '_dispatcher', None))
                if dispatcher:
                    v = dispatcher.get_battery_voltage()
                    if v is not None:
                        await self.broadcast({"type": "battery_voltage", "voltage": v})

        asyncio.create_task(_battery_push_loop())

        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            loop="none",
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.install_signal_handlers = lambda: None  # don't override main handlers
        self._uvicorn_server = server
        logger.info(f"Web interface at http://{self._host}:{self._port}")
        asyncio.create_task(server.serve())
