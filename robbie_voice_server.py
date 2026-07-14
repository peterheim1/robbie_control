#!/usr/bin/env python3
"""
Robbie Voice Server - Main entry point.

Receives audio from a ReSpeaker Lite (XIAO ESP32-S3) via TCP,
detects wake words with openwakeword, transcribes with Faster-Whisper,
classifies intent, dispatches to ROS 2, and responds via Piper TTS.

Usage:
    python3 -m robbie_control.robbie_voice_server
    python3 robbie_control/robbie_voice_server.py
    python3 robbie_control/robbie_voice_server.py --config robbie_control/config/voice_config.yaml
"""

import argparse
import asyncio
import contextlib
import dataclasses
import logging
import re
import signal
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

from robbie_control.console_logger import ConsoleLogger
from robbie_control.tcp_audio_client import TCPAudioClient, LED_IDLE, LED_WAKE, LED_THINKING
from robbie_control.intent_classifier import IntentClassifier
from robbie_control.wled_client import WLEDClient
from robbie_control.llm_client import LLMClient
from robbie_control.ros2_dispatcher import ROS2Dispatcher
from robbie_control.stop_detector import StopDetector
from robbie_control.stt_engine import STTEngine
from robbie_control.task_runner import TaskRunner
from robbie_control.tts_engine import TTSEngine

logging.basicConfig(level=logging.DEBUG, format="%(name)s: %(message)s")
logger = logging.getLogger(__name__)

# Bytes of audio for ~0.5s at 16kHz 16-bit mono
STOP_CHECK_BYTES = 16000

# head_roll wobble while talking — URDF limit is +-30 deg (0.5236 rad), stay well inside it
HEAD_TALK_ROLL_DEG = 8.0
HEAD_TALK_STEP_SECS = 0.6
HEAD_TALK_MOVE_SECS = 0.5

# Matched anywhere in the transcript (not just at the start) so a leading
# filler word ("uh", "um") from STT doesn't defeat the match.
_NAME_LEADIN_RE = re.compile(
    r"(?:my name is|i am|i'm|im|this is|it'?s|call me|name'?s|the name is)\s+(.+)$",
    re.IGNORECASE,
)
_NAME_FILLER_WORDS = {"uh", "um", "i", "think", "and", "so", "well", "the", "a"}


def _extract_name(text: str) -> str:
    """Extract a name from a spoken reply, stripping common lead-in phrases.

    Falls back to the raw transcript if no lead-in phrase matches (the user
    may have just said their name alone). Filler words are dropped and at
    most two words are kept, so trailing chatter ("...i think") doesn't get
    captured as part of the name.
    """
    cleaned = text.strip().strip(".!?").strip()
    if not cleaned:
        return ""
    match = _NAME_LEADIN_RE.search(cleaned)
    candidate = match.group(1) if match else cleaned
    words = [w for w in candidate.split() if w.lower() not in _NAME_FILLER_WORDS][:2]
    return " ".join(w.capitalize() for w in words)


def _json_safe(obj):
    """Recursively convert dataclass instances to dicts for JSON serialisation."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return dataclasses.asdict(obj)
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_json_safe(v) for v in obj]
    return obj


class RobbieVoiceServer:
    """Main voice control server orchestrating the full pipeline."""

    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        config_dir = Path(config_path).parent

        self.console = ConsoleLogger()
        self._executor = ThreadPoolExecutor(max_workers=3)
        self._running = False

        # Serializes TTS synthesis+send so concurrent /voice/speak messages
        # don't interleave their audio on the TCP stream.
        self._tts_lock = asyncio.Lock()

        # Components (initialized in startup)
        self._esphome: TCPAudioClient | None = None
        self._stt: STTEngine | None = None
        self._stop_detector: StopDetector | None = None
        self._tts: TTSEngine | None = None
        self._llm: LLMClient | None = None
        self._classifier: IntentClassifier | None = None
        self._dispatcher: ROS2Dispatcher | None = None
        self._task_runner: TaskRunner | None = None
        self._web = None
        self._wled: WLEDClient | None = None
        self._wled_listening = False   # True while voice pipeline is active
        self._listen_lock = asyncio.Lock()

        # Config file paths
        self._intents_path = str(config_dir / "intents.yaml")
        self._locations_path = str(config_dir / "locations.yaml")
        self._head_path = str(config_dir / "head_positions.yaml")
        self._commands_path = str(config_dir / "commands.txt")

    def _load_config(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)

    async def startup(self):
        """Initialize all components in order."""
        self.console.log_info("Starting Robbie Voice Server...")

        # 1. ROS 2 dispatcher (background thread)
        self.console.log_info("Initializing ROS 2...")
        ros2_cfg = self.config.get("ros2", {})
        self._dispatcher = ROS2Dispatcher(
            node_name=ros2_cfg.get("node_name", "robbie_voice"),
            topic_config=ros2_cfg.get("topics", {}),
        )
        # Wire /voice/speak → TTS so external nodes can speak text
        self._dispatcher.set_speak_callback(
            self._speak, asyncio.get_event_loop()
        )
        # Wire /voice/ask_and_listen → ask a question, listen for the reply,
        # and cue face enrollment (used by Argus's meet & greet)
        self._dispatcher.set_ask_and_listen_callback(
            self._ask_and_listen_for_name, asyncio.get_event_loop()
        )

        # 2. STT engines (CUDA)
        stt_cfg = self.config.get("stt", {})
        device = stt_cfg.get("device", "cuda")

        self.console.log_info(f"Loading Faster-Whisper ({stt_cfg.get('model', 'small')}) on {device}...")
        self._stt = await asyncio.get_event_loop().run_in_executor(
            self._executor,
            lambda: STTEngine(
                model_size=stt_cfg.get("model", "small"),
                device=device,
                language=stt_cfg.get("language", "en"),
            ),
        )

        # 3. Stop detector (tiny model)
        stop_cfg = self.config.get("stop_detector", {})
        self.console.log_info("Loading stop detector (tiny model)...")
        self._stop_detector = await asyncio.get_event_loop().run_in_executor(
            self._executor,
            lambda: StopDetector(
                keywords=stop_cfg.get("keywords"),
                device=device,
            ),
        )

        # 4. TTS
        tts_cfg = self.config.get("tts", {})
        self.console.log_info(f"Initializing Piper TTS ({tts_cfg.get('model', 'en_US-lessac-medium')})...")
        self._tts = TTSEngine(
            model=tts_cfg.get("model", "en_US-lessac-medium"),
            output_sample_rate=tts_cfg.get("output_sample_rate", 22050),
        )

        # 5. LLM
        llm_cfg = self.config.get("llm", {})
        self._llm = LLMClient(
            model=llm_cfg.get("model", "mistral"),
            host=llm_cfg.get("host", "http://localhost:11434"),
            system_prompt=llm_cfg.get("system_prompt", ""),
            max_tokens=llm_cfg.get("max_tokens", 100),
        )
        if self._llm.health_check():
            self.console.log_info(f"Ollama connected ({llm_cfg.get('model', 'mistral')})")
        else:
            self.console.log_error(
                f"Ollama not available - general questions will fail"
            )

        # 6. Intent classifier + commands.txt
        from robbie_control.command_loader import load_commands
        from pathlib import Path as _Path
        _cmd_path = _Path(self._commands_path)
        commands = load_commands(self._commands_path) if _cmd_path.exists() else []
        if commands:
            self.console.log_info(
                f"Loaded {len(commands)} commands: {', '.join(c.name for c in commands)}"
            )
        self._classifier = IntentClassifier(
            intents_path=self._intents_path,
            locations_path=self._locations_path,
            head_positions_path=self._head_path,
            commands=commands,
        )
        self.console.log_info("Intent classifier loaded")

        # 7. Task runner
        tasks_cfg = self.config.get("tasks", {})
        tasks_dir = tasks_cfg.get("dir", "")
        if tasks_dir:
            self._task_runner = TaskRunner(
                tasks_dir=tasks_dir,
                locations_path=self._locations_path,
                speak_fn=self._speak,
                loop=asyncio.get_event_loop(),
                status_fn=self._on_task_status,
            )
            task_names = self._task_runner.list_tasks()
            self.console.log_info(
                f"Task runner ready ({len(task_names)} tasks: {', '.join(task_names) or 'none'})"
            )

        # 9. Audio input (local mic or TCP/ESP32)
        audio_source = self.config.get("audio_source", "tcp")
        tcp_cfg = self.config.get("tcp_audio", {})

        if audio_source == "push_to_talk":
            from robbie_control.local_mic_client import PushToTalkClient
            mic_cfg = self.config.get("local_mic", {})
            self._esphome = PushToTalkClient(
                device=mic_cfg.get("device"),
                tts_device=mic_cfg.get("tts_device"),
                tts_sample_rate=self.config.get("tts", {}).get("output_sample_rate", 22050),
            )
            self.console.log_info("Using push-to-talk input (press ENTER to speak)")
        elif audio_source == "local_mic":
            from robbie_control.local_mic_client import LocalMicClient
            mic_cfg = self.config.get("local_mic", {})
            self._esphome = LocalMicClient(
                device=mic_cfg.get("device"),
                wake_word_model=mic_cfg.get("wake_word_model", "hey_jarvis"),
                wake_word_threshold=mic_cfg.get("wake_word_threshold", 0.85),
                tts_device=mic_cfg.get("tts_device"),
                tts_sample_rate=self.config.get("tts", {}).get("output_sample_rate", 22050),
            )
            self.console.log_info("Using local microphone input")
        else:
            self._esphome = TCPAudioClient(
                listen_host=tcp_cfg.get("listen_host", "0.0.0.0"),
                listen_port=tcp_cfg.get("listen_port", 8765),
                wake_word_model=tcp_cfg.get("wake_word_model", "hey_jarvis"),
                wake_word_threshold=tcp_cfg.get("wake_word_threshold", 0.5),
            )
            self.console.log_info(f"Starting TCP audio server on {tcp_cfg.get('listen_host', '0.0.0.0')}:{tcp_cfg.get('listen_port', 8765)}...")

        self._esphome.on_wake_word = self._on_wake_word
        await self._esphome.connect()

        # 10. Web interface
        web_cfg = self.config.get("web", {})
        if web_cfg.get("enabled", True):
            from robbie_control.web_server import WebServer
            self._web = WebServer(
                host=web_cfg.get("host", "0.0.0.0"),
                port=web_cfg.get("port", 8080),
            )
            await self._web.start(self)
            self.console.log_info(f"Web interface at http://0.0.0.0:{web_cfg.get('port', 8080)}")

        # 11. WLED
        wled_cfg = self.config.get("wled", {})
        if wled_cfg.get("enabled", False):
            self._wled = WLEDClient(host=wled_cfg.get("host", "10.0.0.85"))
            await self._update_idle_wled()
            await self._wled.set_mouth_off()
            await self._update_status_wled()
            self.console.log_info(f"WLED connected at {wled_cfg.get('host', '10.0.0.85')} (eyes=green)")
            asyncio.get_event_loop().create_task(self._status_wled_loop(
                flash_interval_s=wled_cfg.get("status_flash_interval_s", 600),
                flash_duration_s=wled_cfg.get("status_flash_duration_s", 4)))

        self.console.log_info("Robbie Voice Server ready")
        self.console.log_end()

    async def _on_task_status(self, name: str, step: str, running: bool):
        """Broadcast task progress events to the web UI."""
        await self.publish_event({
            "type": "task_update",
            "task": name,
            "step": step,
            "running": running,
        })

    async def _update_idle_wled(self):
        """Set the eyes segment to idle green (segment 0)."""
        if not self._wled:
            return
        self._wled_listening = False
        await self._wled.set_eyes_idle()

    async def _update_status_wled(self):
        """Set the status segment (segment 2) from battery voltage / active faults."""
        if not self._wled:
            return
        charging = self._dispatcher.get_charge_state() if self._dispatcher else None
        voltage  = self._dispatcher.get_battery_voltage() if self._dispatcher else None
        has_problem = bool(self._dispatcher.get_system_errors()) if self._dispatcher else False
        await self._wled.set_status(voltage, bool(charging), has_problem)

    async def _status_wled_loop(self, flash_interval_s: float = 600, flash_duration_s: float = 4):
        """Periodically refresh the status segment — independent of eyes/listening state.

        Every flash_interval_s (config: wled.status_flash_interval_s, default
        10 min), briefly flashes the status segment with an idle decoration,
        alternating each time between a rainbow effect and a chase effect in
        a "larva" color scheme, then reverts to the normal voltage/fault
        display. Purely cosmetic — unrelated to /diagnostics faults, which
        keep using the existing red blink in _update_status_wled() untouched.
        """
        elapsed = 0.0
        flash_rainbow_next = True
        while True:
            await self._update_status_wled()
            await asyncio.sleep(10)
            elapsed += 10
            if elapsed >= flash_interval_s:
                elapsed = 0.0
                if self._wled:
                    if flash_rainbow_next:
                        await self._wled.flash_rainbow()
                    else:
                        await self._wled.flash_chase_larva()
                    flash_rainbow_next = not flash_rainbow_next
                    await asyncio.sleep(flash_duration_s)
                    await self._update_status_wled()

    def _on_wake_word(self):
        """Called when wake word is detected."""
        self.console.log_wake()
        self._wled_listening = True
        loop = asyncio.get_event_loop()
        loop.create_task(self._esphome.send_led_state(LED_WAKE))
        if self._wled:
            loop.create_task(self._wled.set_eyes_listening())
        loop.create_task(self._handle_pipeline())

    async def publish_event(self, event: dict):
        """Broadcast a pipeline event to all web UI clients."""
        if self._web:
            await self._web.broadcast(event)

    async def handle_text_command(self, text: str):
        """Inject a text command directly into the pipeline, bypassing wake word and STT."""
        self.console.log_info(f"[WEB] {text}")
        await self.publish_event({"type": "status", "state": "processing"})
        await self._esphome.send_led_state(LED_THINKING)
        await self._process_text(text, source="web")
        await self._esphome.send_led_state(LED_IDLE)
        await self.publish_event({"type": "status", "state": "listening"})

    async def listen_for_answer(self, timeout: float = 10.0) -> str:
        """Capture a voice response without a wake word and return the transcript.

        Intended for external programs that ask Robbie a question and need the
        user's spoken reply.  Sets WLED to magenta while listening, returns to
        green (idle) when done.

        Args:
            timeout: Maximum seconds to wait for the user to speak.

        Returns:
            Transcribed text, or empty string on silence / error.
        """
        async with self._listen_lock:
            self._wled_listening = True
            loop = asyncio.get_event_loop()
            vad_cfg = self.config.get("vad", {})
            max_duration = min(timeout, vad_cfg.get("max_duration_ms", 10000) / 1000.0)

            # Signal listening state
            if self._wled:
                await self._wled.set_eyes_listening()
            await self._esphome.send_led_state(LED_WAKE)
            await self.publish_event({"type": "status", "state": "recording"})

            # Trigger recording without a wake word
            self._esphome.start_direct_listen()

            # Wait for the user to speak
            audio = await self._esphome.wait_for_audio(timeout=max_duration)

            # Return to idle LEDs before processing
            if self._wled:
                await self._update_idle_wled()
            await self._esphome.send_led_state(LED_IDLE)

            if not audio or len(audio) < 1600:
                self.console.log_error("listen_for_answer: no audio captured")
                await self.publish_event({"type": "status", "state": "listening"})
                return ""

            await self._esphome.send_led_state(LED_THINKING)
            text = await loop.run_in_executor(self._executor, self._stt.transcribe, audio)
            await self._esphome.send_led_state(LED_IDLE)

            self.console.log_utterance(text)
            await self.publish_event({"type": "transcript", "text": text, "source": "listen"})
            await self.publish_event({"type": "status", "state": "listening"})
            return text

    async def _ask_and_listen_for_name(self, question: str):
        """Ask a question via TTS, listen for a spoken name, cue enrollment.

        Triggered by /voice/ask_and_listen (Argus's meet & greet asks an
        unrecognized face for their name). Waits for the question to finish
        playing before opening the mic, extracts the name from the reply,
        publishes a train_face intent on /voice/intent, and greets them.
        """
        await self._speak(question)
        transcript = await self.listen_for_answer(timeout=8.0)
        name = _extract_name(transcript)
        if not name:
            self.console.log_error("ask_and_listen: no name captured")
            await self._speak("Sorry, I didn't catch your name")
            return
        self.console.log_info(f"ask_and_listen: captured name '{name}'")
        self._dispatcher.publish_train_face(name)
        await self._speak(f"Hello {name}, nice to meet you")

    async def _handle_pipeline(self):
        """Handle a complete voice pipeline: audio → STT → intent → dispatch → TTS."""
        loop = asyncio.get_event_loop()
        vad_cfg = self.config.get("vad", {})
        max_duration = vad_cfg.get("max_duration_ms", 5000) / 1000.0
        stop_check_secs = self.config.get("stop_detector", {}).get("max_audio_seconds", 0.5)

        await self.publish_event({"type": "status", "state": "recording"})

        # Wait briefly for initial audio to arrive for stop check
        await asyncio.sleep(stop_check_secs + 0.1)

        # Fast-path stop check on partial audio
        partial_audio = self._esphome.get_partial_audio(STOP_CHECK_BYTES)
        if partial_audio and len(partial_audio) >= STOP_CHECK_BYTES // 2:
            is_stop = await loop.run_in_executor(
                self._executor,
                self._stop_detector.check,
                partial_audio,
            )
            if is_stop:
                self.console.log_fast_stop()
                self._dispatcher.publish_stop()
                self.console.log_publish(["/voice/stop", "/cmd_vel", "/drive"])
                await self.publish_event({"type": "intent", "name": "stop_all", "params": {}, "response": "stopping"})
                await self._speak("stopping")
                if self._wled:
                    await self._update_idle_wled()
                await self._esphome.send_led_state(LED_IDLE)
                await self.publish_event({"type": "status", "state": "listening"})
                self.console.log_end()
                return

        # Wait for full audio
        audio = await self._esphome.wait_for_audio(timeout=max_duration)

        if not audio or len(audio) < 1600:
            self.console.log_error("No audio received")
            await self.publish_event({"type": "log", "level": "error", "msg": "No audio received"})
            if self._wled:
                await self._update_idle_wled()
            await self._esphome.send_led_state(LED_IDLE)
            await self.publish_event({"type": "status", "state": "listening"})
            self.console.log_end()
            return

        await self._esphome.send_led_state(LED_THINKING)
        await self.publish_event({"type": "status", "state": "processing"})

        # Full STT transcription
        text = await loop.run_in_executor(
            self._executor, self._stt.transcribe, audio
        )
        self.console.log_utterance(text)

        await self._process_text(text, source="voice")

        if self._wled:
            await self._wled.set_eyes_idle()
        await self._esphome.send_led_state(LED_IDLE)
        await self.publish_event({"type": "status", "state": "listening"})
        self.console.log_end()

    async def _process_text(self, text: str, source: str = "voice"):
        """Classify and dispatch a transcribed or injected text command."""
        loop = asyncio.get_event_loop()

        await self.publish_event({"type": "transcript", "text": text, "source": source})

        if not text:
            await self._speak("I didn't catch that")
            return

        # Classify intent
        intent = self._classifier.classify(text)
        self.console.log_intent(intent.name, intent.params)
        await self.publish_event({
            "type": "intent",
            "name": intent.name,
            "params": _json_safe(intent.params),
            "response": intent.response_text,
        })

        # Dispatch to ROS 2
        self._dispatcher.publish_intent(intent)
        published_topics = [self._dispatcher._topic_names["intent"]]

        # commands.txt actions — dispatch and cancel tasks if flagged
        ros_actions = intent.params.get("ros_actions")
        if ros_actions is not None:
            if "cancel_tasks" in intent.params.get("flags", []) and self._task_runner:
                self._task_runner.cancel()
            published_topics += self._dispatcher.dispatch_ros_actions(ros_actions)

        self.console.log_publish(published_topics)

        # Generate and speak response
        response_text = await self._generate_response(intent)
        if response_text:
            await self._speak(response_text)

    async def _generate_response(self, intent) -> str:
        """Generate the text response for an intent."""
        from datetime import datetime
        loop = asyncio.get_event_loop()

        if intent.name == "goto":
            location = intent.params.get("location", "destination")
            x = intent.params.get("x", 0.0)
            y = intent.params.get("y", 0.0)
            yaw_deg = intent.params.get("yaw_deg", 0.0)
            self._dispatcher.navigate_to(x, y, yaw_deg, location)
            return f"navigating to {location}"

        if intent.name == "query_datetime":
            now = datetime.now()
            hour = now.strftime("%I").lstrip("0")
            minute = now.strftime("%M")
            ampm = now.strftime("%p").lower()
            day = now.strftime("%A")
            date = now.strftime("%-d %B")
            if "date" in intent.utterance.lower() or "day" in intent.utterance.lower() or "today" in intent.utterance.lower():
                return f"today is {day} the {date}"
            return f"it's {hour} {minute} {ampm}"

        if intent.name == "general_question":
            question = intent.params.get("original", intent.utterance)
            self.console.log_info("Querying LLM...")
            answer = await loop.run_in_executor(
                self._executor, self._llm.ask, question
            )
            self.console.log_llm(answer)
            return answer

        if intent.name == "system_status":
            return self._build_system_status_response()

        if intent.name == "query_status":
            return self._build_status_response(intent.params.get("subject", "general"))

        if intent.name == "list_topics":
            return "Here are the active topics: " + ", ".join(
                self._dispatcher.get_published_topics()
            )

        if intent.name == "run_task":
            task_name = intent.params.get("task_name", "").lower()
            if not self._task_runner:
                return "Task runner is not configured"
            available = self._task_runner.list_tasks()
            if task_name not in available:
                return f"I don't have a task called {task_name}. Available tasks are: {', '.join(available)}"
            asyncio.create_task(self._task_runner.run_task(task_name))
            return None  # task_runner speaks its own confirmation

        if intent.name == "list_tasks":
            if not self._task_runner:
                return "Task runner is not configured"
            tasks = self._task_runner.list_tasks()
            if not tasks:
                return "I don't have any tasks loaded"
            return "Available tasks are: " + ", ".join(tasks)

        return intent.response_text

    def _build_system_status_response(self) -> str:
        """Check /diagnostics for errors, excluding wdog and high jitter."""
        errors = self._dispatcher.get_system_errors()
        if not errors:
            return "all systems healthy, no errors detected"
        count = len(errors)
        summary = f"{count} issue{'s' if count > 1 else ''} detected: "
        summary += "; ".join(errors[:3])
        if count > 3:
            summary += f"; and {count - 3} more"
        return summary

    def _build_status_response(self, subject: str) -> str:
        """Build a status response based on cached ROS 2 data."""
        if subject == "battery":
            volts = self._dispatcher.get_battery_voltage()
            if volts is None:
                return "I don't have battery information right now"
            # Health assessment for a 12V system (3S LiPo / lead-acid)
            if volts >= 12.4:
                level = "good"
            elif volts >= 11.5:
                level = "OK"
            elif volts >= 10.5:
                level = "low"
            else:
                level = "critically low"
            return f"battery is {volts:.1f} volts, level is {level}"
        elif subject == "dock":
            state = self._dispatcher.get_dock_state()
            if state is not None:
                return "I am docked" if state else "I am not docked"
            return "I don't have docking information right now"
        elif subject == "errors":
            return "no errors reported"
        elif subject == "general":
            return "I'm doing well, all systems are running"
        return f"I don't have {subject} information right now"

    @contextlib.asynccontextmanager
    async def _head_talking(self):
        """Wobble head_roll back and forth for the duration of the block."""
        task = asyncio.get_event_loop().create_task(self._head_talk_wobble())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
            self._dispatcher.send_joint_position(
                "head_roll", 0.0, move_time_sec=HEAD_TALK_MOVE_SECS,
                head_source='idle')

    async def _head_talk_wobble(self):
        sign = 1
        while True:
            # idle priority — any real head command (user look, docking
            # center, boredom scan) outranks the cosmetic wobble
            self._dispatcher.send_joint_position(
                "head_roll", sign * HEAD_TALK_ROLL_DEG, move_time_sec=HEAD_TALK_MOVE_SECS,
                head_source='idle')
            sign *= -1
            await asyncio.sleep(HEAD_TALK_STEP_SECS)

    async def _speak(self, text: str):
        """Synthesize and play TTS audio (skipped if silent mode is on)."""
        if self._tts is None:
            self.console.log_warning("TTS not ready, dropping speak request")
            return
        loop = asyncio.get_event_loop()
        self.console.log_response(text)
        muted = self._web is not None and self._web.tts_muted
        await self.publish_event({"type": "tts", "text": text, "muted": muted})
        if muted:
            # Still ack speak_done: Argus's SpeakAction blocks on this, so
            # skipping it made every muted Speak stall its full 6s timeout.
            if self._dispatcher:
                self._dispatcher.publish_speak_done(text)
            return
        async with self._tts_lock:
            try:
                audio = await loop.run_in_executor(
                    self._executor, self._tts.synthesize, text
                )
                if audio and self._esphome:
                    # send_tts_audio() blocks until local playback (sd.wait())
                    # actually finishes, so the ack below already lands at the
                    # right time — no extra sleep needed.
                    async with contextlib.AsyncExitStack() as stack:
                        if self._wled:
                            await stack.enter_async_context(self._wled.talking())
                        await stack.enter_async_context(self._head_talking())
                        await self._esphome.send_tts_audio(audio)
                self._dispatcher.publish_speak_done(text)
            except Exception as e:
                self.console.log_error(f"TTS failed: {e}")
                await self.publish_event({"type": "log", "level": "error", "msg": f"TTS failed: {e}"})
                self._dispatcher.publish_speak_done(text)

    async def run(self):
        """Main run loop."""
        self._running = True
        await self.startup()

        # Keep running until interrupted
        stop_event = asyncio.Event()

        def _signal_handler():
            stop_event.set()

        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, _signal_handler)

        await stop_event.wait()
        await self.shutdown()

    async def shutdown(self):
        """Gracefully shut down all components."""
        self._running = False
        self.console.log_info("Shutting down...")
        if self._web:
            await self._web.stop()
        if self._esphome:
            await self._esphome.disconnect()
        if self._task_runner:
            self._task_runner.cancel()
            self._task_runner.shutdown()
        if self._dispatcher:
            self._dispatcher.shutdown()
        self._executor.shutdown(wait=False)
        self.console.log_info("Robbie Voice Server stopped")


def main():
    parser = argparse.ArgumentParser(description="Robbie Voice Control Server")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).parent / "config" / "voice_config.yaml"),
        help="Path to voice_config.yaml",
    )
    args = parser.parse_args()

    server = RobbieVoiceServer(args.config)
    asyncio.run(server.run())


if __name__ == "__main__":
    main()
