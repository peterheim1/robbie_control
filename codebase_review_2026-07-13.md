# robbie_control Codebase Review — 2026-07-13

Full review of the voice/AI system: `robbie_voice_server.py`,
`ros2_dispatcher.py`, `web_server.py` (all 3254 lines), `task_runner.py`,
`intent_classifier.py` + `command_loader.py`, the engine modules
(STT/TTS/LLM/WLED/docs), and all config files. Companion to the same-day
reviews in `robbie_bot/` and `behaviour_tree/`.

Overall: a well-factored pipeline (audio → wake word → STT → intent →
dispatch → TTS) with clean module boundaries, config-driven components, a
whitelisted web-command runner, and the speak_done ack design that the BT
depends on. The biggest problems are that **"stop" doesn't actually stop a
navigation mission through any path**, and the web server's power over the
robot is completely unauthenticated.

**Findings 1 and 2 were fixed the same day — see
[Changes Made](#changes-made-2026-07-13) at the bottom. The rest are still
open.** Findings ordered by severity.

---

## Findings

### 1. CRITICAL (FIXED 2026-07-13): no voice-stop path ever cancelled a Nav2 mission

Three stop paths exist, and none of them reaches Argus:

- **Fast path** (`_handle_pipeline`, stop detector on the first 0.5 s of
  audio): calls `dispatcher.publish_stop()` → `/voice/stop` (Empty) + **one**
  zero `/cmd_vel` + **one** zero `/drive`, then returns *before intent
  classification*. Nav2's controller publishes cmd_vel continuously, so the
  robot resumes moving right after the momentary zero. No intent is
  published, so the BT never hears about it.
- **Full path** (`[stop]` in `config/commands.txt`): publishes intent name
  `"stop"` on `/voice/intent` plus the same one-shot zeros. But
  `bt_executor`'s `intent_cb` only handles `"stop_all"` — `"stop"` falls
  into the not-handled debug branch. Nothing cancels.
- **Web** `/api/stop_all`: same `publish_stop()` one-shot zeros only.

Worse, the fast path usually *wins* the race (any of stop/halt/freeze in the
first half-second), so even if the full path were wired correctly, it would
rarely run.

The 2026-07-13 BT fix made `stop_all` genuinely halt missions — but nothing
in robbie_control ever sends `stop_all`. **Suggested fix (small):** rename
the `[stop]` command block to `stop_all` (command name = published intent
name, same pattern the `look_*` commands use), and have both the fast path
and `/api/stop_all` also publish the `stop_all` intent JSON on
`/voice/intent` (or `{"cmd":"cancel"}` on `/bt_command`). Keep the
existing one-shot zeros — they're a good reflex for teleop — but they must
be accompanied by the BT cancel.

### 2. (FIXED 2026-07-13) Muted TTS never published speak_done — stalled every BT Speak by 6 s

`_speak()` early-returns when web-muted (`robbie_voice_server.py`) without
calling `publish_speak_done()`. The BT's SpeakAction then waits its full 6 s
timeout on every Speak node — a muted patrol gains ~6 s of dead time per
announcement (two per leg). One-line fix: publish the ack in the muted
branch too. (The success and exception branches both ack correctly.)

### 3. The web server is unauthenticated root-level control on 0.0.0.0

No auth, no origin checks, on every endpoint (`web.host: 0.0.0.0`, port
8090):

- `POST /api/shutdown` powers off the robot **and** ssh-poweroffs the oculus
  Pi. Any device on the LAN — or any web page open in a LAN browser (simple
  cross-origin POSTs are not blocked by CORS) — can do this with one request.
- `POST /api/listen` opens the microphone and returns a transcript.
- Joint motion, pose delete, task-file writes, whitelisted-command execution
  over WebSocket.

For a home LAN this is a known trade-off, but at minimum: require a confirm
token on `/api/shutdown`, and consider checking the `Origin`/`Host` headers
against an allowlist to stop drive-by browser requests. (The command runner
itself is properly whitelisted via `approved_commands.yaml` — good.)

### 4. SLAM "save map" writes to the pre-remap map file

`ros2_dispatcher.slam_serialize_map()` hardcodes
`filename='/home/pi/mar_19c_26'`. The active map has been `map_5_7_26_2`
since the 2026-07-05 remap — so the web UI's Map Update button serializes
the pose graph to a file navigation never loads, and reports success.
Update the path and move it into `voice_config.yaml`.

### 5. Parallel ROS plumbing in task_runner

`task_runner.py` lazily creates its own ROS node (`robbie_task_runner`) with
its own executor, spin thread, `/bt_command` publisher, `/bt_status`
subscriber, and dock/undock service clients — all capabilities the
dispatcher node already has. Two nodes, two spin threads, and two
`/bt_status` caches in one process. Consolidating on the dispatcher would
delete ~60 lines and one failure mode (the dispatcher's `_spin` already has
the restart-on-crash guard; task_runner's executor thread doesn't).

### 6. LLM endpoint hardcoded in three places (logs/report FIXED 2026-07-13)

`voice_config.yaml` has `llm.host: http://10.0.0.87:11434` (fine — config),
but `docs_engine.py` defaults to the same IP in code, and
`web_server.py`'s `/api/logs/report` hardcoded both the IP *and* the model
(`qwen3:4b`) inline.

**Update 2026-07-13:** the hardcoded `qwen3:4b` turned out to be an outright
bug, not just hygiene — qwen3 is a thinking model, current Ollama ignores
the legacy `/no_think` soft switch, so the entire `num_predict` budget went
to hidden thinking and **zero content tokens ever streamed: the Logs tab
never produced a report at all**. Fixed: `/api/logs/report` now reads
model+host from the `llm` config block (mistral), the dead `/no_think`
prefix is gone, and log collection is wrapped so an exception can't silently
kill the SSE stream. Verified end-to-end (real logs → 493-char report in
5 s). `docs_engine.py`'s in-code IP default remains open (its model was
already mistral, so it never had the thinking bug).

### 7. Smaller items

- **`broadcast()` mutation race** (`web_server.py`): iterates
  `self._clients` with `await` inside the loop; a client
  connecting/disconnecting mid-broadcast mutates the set during iteration →
  `RuntimeError`. Iterate `list(self._clients)`.
- **Cross-package file coupling:** web_server reads/writes
  `../robbie_bot/poses`, `../robbie_bot/behaviours`, serves
  `../robbie_bot/blog`, parses `../robbie_description/urdf/...` — all
  `__file__`-relative into sibling git repos. Works on this checkout, brittle
  anywhere else; pose/behaviour edits from the web UI also land as
  uncommitted changes in a *different* repo.
- **Dead config key:** `web.tts_muted_default` is never read —
  `WebServer.__init__` hardcodes `_tts_muted = False`.
- **`_HTML` is a single 104 KB inline string** — 69 % of web_server.py's
  bytes. Every UI tweak is a giant diff in a Python file. Move to static
  files (FastAPI `StaticFiles`) or at least a sibling `.html` file.
- **Stale comment:** startup log says "STT engines (CUDA)" while config runs
  `device: cpu`; README/docstring still describe the ESP32 TCP mic as
  primary while `audio_source: local_mic` is what's configured.
- **`asyncio.get_event_loop()`** used throughout in contexts where Python
  3.12 wants `get_running_loop()` — works today, deprecation-warns, will
  break on an interpreter bump.
- **Uncommitted work:** 10 modified files (README, command_loader,
  commands.txt, voice_config, voice server, dispatcher, presets, tts_engine,
  web_server, wled_client). Same commit-hygiene risk as the other packages.
- `intent "look"` routing is fine (verified): `look_*` commands.txt entries
  have no `ros:` lines, so their names publish as intents that BT's
  `LOOK_TARGETS` consumes — the design comment matches the code.

### What's notably good

- The speak_done ack contract with Argus (except the muted branch, #2).
- `_tts_lock` serializing TTS; `_listen_lock` serializing mic sessions.
- The dispatcher `_spin` restart guard (the silent-death bug from memory is
  fixed and commented).
- Whitelisted `approved_commands.yaml` command runner instead of arbitrary
  shell from the UI; task editor restricted to existing files with a
  name-sanitizing regex.
- WLED/head-wobble side effects wrapped in async context managers that
  clean up on cancellation.

---

## Suggested priority

1. ~~Fix stop (finding 1)~~ **DONE 2026-07-13, see below.**
2. ~~Muted speak_done fix (finding 2)~~ **DONE 2026-07-13, see below.**
3. Update the SLAM map path (finding 4) before the next map save wipes
   hours of mapping into the wrong file.
4. Shutdown-endpoint confirm token + Origin check (finding 3).
5. Commit the pending files, then the consolidation items (5, 6, 7) as
   opportunistic cleanups.

---

## Changes Made (2026-07-13)

### Finding 1 — stop now cancels missions through every path

- **`config/commands.txt`**: `[stop]` renamed `[stop_all]` (the command name
  is the published intent name, and bt_executor only handles `stop_all`) —
  with a keep-it-named-this comment. Phrases/say/flags/ros lines unchanged,
  so the one-shot cmd_vel/drive zero reflex is kept.
- **`ros2_dispatcher.publish_stop()`**: now also publishes
  `{"intent": "stop_all", "params": {}}` on `/voice/intent` after the
  one-shot zeros. This wires the voice fast path AND web `/api/stop_all`
  (both call `publish_stop()`) into the BT's imperative cancel.

**Verified live against a running bt_executor** (real IntentClassifier +
real ROS2Dispatcher, scripted):
1. `navigate_to(kitchen)` → `/bt_status` `cmd=goto_location` ✓
2. fast path `publish_stop()` → BT logged
   `cancel (voice stop_all) — halted mission 'goto_location'`, status
   cleared ✓
3. second `navigate_to(tv)` → dispatched ✓
4. full path `classify("stop")` → `stop_all` → `publish_intent()` → second
   BT cancel logged, status cleared ✓

Not exercised (Nav2 was down): the actual Nav2 goal cancellation and the
base physically stopping — same caveat as the BT-side fix. **On the next
robot session: say "stop" during a patrol leg and confirm the base halts
and Nav2 logs a cancelled goal.** Note the robot will say "stopping" (voice
server) and then "Cancelled, stopped in place" (BT) — slightly chatty,
trim later if it grates.

### Finding 2 — muted TTS acks speak_done

- **`robbie_voice_server._speak()`**: the muted early-return now calls
  `publish_speak_done(text)` first, matching the success and exception
  branches. Verified by inspection + py_compile (needs the full voice
  server + a muted web client to exercise end-to-end; behavior change is
  the absence of 6 s stalls per Speak while muted).

Both changes are uncommitted, joining the package's existing 10 modified
files.
