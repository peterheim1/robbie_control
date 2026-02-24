"""Local microphone audio client with openwakeword wake word detection.

Drop-in replacement for TCPAudioClient. Captures audio from the local
microphone using sounddevice, runs openwakeword for wake word detection,
and plays TTS audio through local speakers.

Same interface as TCPAudioClient:
  - connect() / disconnect()
  - on_wake_word callback
  - wait_for_audio(timeout) -> bytes
  - get_partial_audio(max_bytes) -> bytes
  - send_tts_audio(audio_data)
  - send_led_state(state)  # no-op for desktop
"""

import asyncio
import logging
import sys
import threading

import numpy as np
import sounddevice as sd
from openwakeword.model import Model as OWWModel

# webrtcvad's wrapper needlessly imports pkg_resources for __version__.
# Stub it out if not available (conda setuptools splits it out).
if "pkg_resources" not in sys.modules:
    try:
        import pkg_resources  # noqa: F401
    except ImportError:
        _stub = type(sys)("pkg_resources")
        _stub.get_distribution = lambda *a, **k: type("D", (), {"version": "0.0.0"})()
        sys.modules["pkg_resources"] = _stub
import webrtcvad

logger = logging.getLogger(__name__)

SAMPLE_RATE = 16000       # capture and process at 16kHz
CHANNELS = 1
BYTES_PER_SAMPLE = 2

# openwakeword works well with 1024-sample blocks at 16kHz
OWW_CHUNK_SAMPLES = 1024
OWW_CHUNK_BYTES = OWW_CHUNK_SAMPLES * BYTES_PER_SAMPLE


class LocalMicClient:
    """Captures audio from a local microphone and detects wake words.

    Provides the same interface as TCPAudioClient so it can be used as a
    drop-in replacement in robbie_voice_server.py.
    """

    def __init__(
        self,
        device=None,
        wake_word_model: str = "hey_jarvis",
        wake_word_threshold: float = 0.5,
        tts_device=None,
        tts_sample_rate: int = 22050,
    ):
        self._device = device
        self._tts_device = tts_device
        self._tts_sample_rate = tts_sample_rate
        self._ww_model_name = wake_word_model
        self._ww_threshold = wake_word_threshold

        self._oww: OWWModel | None = None
        self._oww_buf = bytearray()

        self._vad = webrtcvad.Vad(3)

        self._audio_buffer = bytearray()
        self._is_recording = False
        self._recording_done = asyncio.Event()

        self._silence_frames = 0
        self._speech_frames = 0
        self._silence_limit = 48  # ~1.5s at 30ms frames (1500/30)

        self.on_wake_word: callable = None

        self._stream: sd.InputStream | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    async def connect(self):
        """Load the wake word model and start the microphone stream."""
        self._loop = asyncio.get_event_loop()

        logger.info(f"Loading openwakeword model: {self._ww_model_name}")
        self._oww = OWWModel(
            wakeword_models=[self._ww_model_name],
            inference_framework="onnx",
        )
        logger.info("openwakeword model loaded")

        device_info = sd.query_devices(self._device, "input")
        logger.info(f"Using mic: {device_info['name']}")

        self._stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
            blocksize=OWW_CHUNK_SAMPLES,
            callback=self._audio_callback,
            device=self._device,
        )
        self._stream.start()
        logger.info(f"Microphone stream started at {SAMPLE_RATE}Hz")

    def _audio_callback(self, indata, frames, time, status):
        """Called by sounddevice in a background thread for each audio chunk."""
        if status:
            logger.warning(f"Audio input status: {status}")
        audio_bytes = indata.flatten().tobytes()
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                self._process_mic_audio(audio_bytes), self._loop
            )

    async def _process_mic_audio(self, data: bytes):
        """Process incoming mic audio: wake word detection or recording."""
        if self._is_recording:
            self._audio_buffer.extend(data)
            self._check_vad(data)
            return

        # Feed to openwakeword in 80ms chunks
        self._oww_buf.extend(data)
        while len(self._oww_buf) >= OWW_CHUNK_BYTES:
            chunk = bytes(self._oww_buf[:OWW_CHUNK_BYTES])
            del self._oww_buf[:OWW_CHUNK_BYTES]

            samples = np.frombuffer(chunk, dtype=np.int16)
            self._debug_counter = getattr(self, "_debug_counter", 0) + 1
            if self._debug_counter % 50 == 0:
                logger.debug(
                    f"audio stats: min={samples.min()} max={samples.max()} "
                    f"rms={np.sqrt(np.mean(samples.astype(np.float32)**2)):.0f} "
                    f"len={len(samples)}"
                )
            prediction = self._oww.predict(samples)

            for model_name, score in prediction.items():
                if score > 0.1 or self._debug_counter % 50 == 0:
                    logger.debug(f"oww: {model_name}={score:.3f}")
                if score >= self._ww_threshold:
                    logger.info(f"Wake word '{model_name}' detected (score={score:.3f})")
                    self._oww.reset()
                    self._oww_buf.clear()
                    self._start_recording()
                    if self.on_wake_word:
                        self.on_wake_word()
                    return

    def _start_recording(self):
        """Begin recording audio for the current utterance."""
        self._audio_buffer = bytearray()
        self._is_recording = True
        self._recording_done.clear()
        self._silence_frames = 0
        self._speech_frames = 0

    def _check_vad(self, data: bytes):
        """Check for end of speech using webrtcvad."""
        frame_bytes = SAMPLE_RATE * BYTES_PER_SAMPLE * 30 // 1000  # 960 bytes = 30ms

        offset = 0
        while offset + frame_bytes <= len(data):
            frame = data[offset:offset + frame_bytes]
            offset += frame_bytes

            try:
                is_speech = self._vad.is_speech(frame, SAMPLE_RATE)
            except Exception:
                continue

            if is_speech:
                self._speech_frames += 1
                self._silence_frames = 0
            else:
                self._silence_frames += 1

            if self._speech_frames > 3 and self._silence_frames >= self._silence_limit:
                self._is_recording = False
                self._recording_done.set()
                logger.info(f"End of speech detected ({len(self._audio_buffer)} bytes)")
                return

    def start_direct_listen(self):
        """Start recording immediately without waiting for the wake word."""
        self._start_recording()

    async def wait_for_audio(self, timeout: float = 5.0) -> bytes:
        """Wait for the current utterance audio to complete."""
        try:
            await asyncio.wait_for(self._recording_done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            self._is_recording = False
            self._recording_done.set()
            logger.warning("Audio recording timed out")

        audio = bytes(self._audio_buffer)
        self._audio_buffer = bytearray()
        return audio

    def get_partial_audio(self, max_bytes: int = 0) -> bytes:
        """Get audio buffered so far without waiting."""
        if max_bytes and len(self._audio_buffer) > max_bytes:
            return bytes(self._audio_buffer[:max_bytes])
        return bytes(self._audio_buffer)

    async def send_tts_audio(self, audio_data: bytes):
        """Play TTS audio through local speakers.

        Pauses the mic stream during playback to prevent feedback.
        """
        samples = np.frombuffer(audio_data, dtype=np.int16)
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._play_audio, samples)

    def _play_audio(self, samples: np.ndarray):
        """Play audio synchronously in executor thread."""
        if self._stream:
            self._stream.stop()
        try:
            sd.play(samples, samplerate=self._tts_sample_rate, device=self._tts_device)
            sd.wait()
        finally:
            if self._stream:
                self._stream.start()

    async def send_led_state(self, state: int):
        """No-op - no LED hardware on desktop mic."""
        pass

    async def disconnect(self):
        """Stop the microphone stream."""
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        logger.info("Local microphone stopped")


class PushToTalkClient:
    """Push-to-talk mic client - press ENTER to start/stop recording.

    Drop-in replacement for TCPAudioClient and LocalMicClient.
    No wake word detection - just press Enter to speak a command.
    VAD auto-stops recording after silence; Enter also stops manually.
    """

    def __init__(
        self,
        device=None,
        tts_device=None,
        tts_sample_rate: int = 22050,
    ):
        self._device = device
        self._tts_device = tts_device
        self._tts_sample_rate = tts_sample_rate

        self._vad = webrtcvad.Vad(3)

        self._audio_buffer = bytearray()
        self._is_recording = False
        self._recording_done = asyncio.Event()

        self._silence_frames = 0
        self._speech_frames = 0
        self._silence_limit = 48  # ~1.5s at 30ms frames

        self.on_wake_word: callable = None

        self._stream: sd.InputStream | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._running = False

    async def connect(self):
        """Start mic stream and Enter-key listener."""
        self._loop = asyncio.get_event_loop()
        self._running = True

        device_info = sd.query_devices(self._device, "input")
        logger.info(f"Push-to-talk using mic: {device_info['name']}")

        self._stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
            blocksize=OWW_CHUNK_SAMPLES,
            callback=self._audio_callback,
            device=self._device,
        )
        self._stream.start()

        t = threading.Thread(target=self._key_listener, daemon=True)
        t.start()

        print("\n[Push-to-talk] Press ENTER to speak a command\n", flush=True)
        logger.info("Push-to-talk ready")

    def _key_listener(self):
        """Background thread: wait for Enter key presses."""
        while self._running:
            try:
                input()
            except EOFError:
                break
            if not self._running:
                break
            if not self._is_recording:
                asyncio.run_coroutine_threadsafe(self._trigger(), self._loop)
            else:
                # Manual stop - user pressed Enter again
                self._is_recording = False
                self._recording_done.set()
                logger.info("Recording stopped manually")

    async def _trigger(self):
        """Start recording and fire the wake word callback."""
        self._audio_buffer = bytearray()
        self._is_recording = True
        self._recording_done.clear()
        self._silence_frames = 0
        self._speech_frames = 0
        print("[Listening...] speak your command (silence auto-stops, or press ENTER)", flush=True)
        if self.on_wake_word:
            self.on_wake_word()

    def _audio_callback(self, indata, frames, time, status):
        """Called by sounddevice in a background thread for each audio chunk."""
        if status:
            logger.warning(f"Audio input status: {status}")
        if not self._is_recording:
            return
        audio_bytes = indata.flatten().tobytes()
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                self._buffer_audio(audio_bytes), self._loop
            )

    async def _buffer_audio(self, data: bytes):
        """Buffer incoming audio and check VAD for end of speech."""
        if not self._is_recording:
            return
        self._audio_buffer.extend(data)
        self._check_vad(data)

    def _check_vad(self, data: bytes):
        """Check for end of speech using webrtcvad."""
        frame_bytes = SAMPLE_RATE * BYTES_PER_SAMPLE * 30 // 1000  # 960 bytes = 30ms
        offset = 0
        while offset + frame_bytes <= len(data):
            frame = data[offset:offset + frame_bytes]
            offset += frame_bytes
            try:
                is_speech = self._vad.is_speech(frame, SAMPLE_RATE)
            except Exception:
                continue
            if is_speech:
                self._speech_frames += 1
                self._silence_frames = 0
            else:
                self._silence_frames += 1
            if self._speech_frames > 3 and self._silence_frames >= self._silence_limit:
                self._is_recording = False
                self._recording_done.set()
                logger.info(f"End of speech detected ({len(self._audio_buffer)} bytes)")
                return

    def start_direct_listen(self):
        """Start recording immediately without waiting for the wake word."""
        self._audio_buffer = bytearray()
        self._is_recording = True
        self._recording_done.clear()
        self._silence_frames = 0
        self._speech_frames = 0

    async def wait_for_audio(self, timeout: float = 5.0) -> bytes:
        """Wait for the current utterance audio to complete."""
        try:
            await asyncio.wait_for(self._recording_done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            self._is_recording = False
            self._recording_done.set()
            logger.warning("Audio recording timed out")
        audio = bytes(self._audio_buffer)
        self._audio_buffer = bytearray()
        return audio

    def get_partial_audio(self, max_bytes: int = 0) -> bytes:
        """Get audio buffered so far without waiting."""
        if max_bytes and len(self._audio_buffer) > max_bytes:
            return bytes(self._audio_buffer[:max_bytes])
        return bytes(self._audio_buffer)

    async def send_tts_audio(self, audio_data: bytes):
        """Play TTS audio through local speakers."""
        samples = np.frombuffer(audio_data, dtype=np.int16)
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._play_audio, samples)

    def _play_audio(self, samples: np.ndarray):
        """Play audio synchronously in executor thread."""
        if self._stream:
            self._stream.stop()
        try:
            sd.play(samples, samplerate=self._tts_sample_rate, device=self._tts_device)
            sd.wait()
        finally:
            if self._stream:
                self._stream.start()

    async def send_led_state(self, state: int):
        """No-op - no LED hardware."""
        pass

    async def disconnect(self):
        """Stop mic stream and key listener."""
        self._running = False
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        logger.info("Push-to-talk stopped")
