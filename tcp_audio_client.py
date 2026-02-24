"""TCP audio client with openwakeword wake word detection.

Drop-in replacement for ESPHomeClient. Runs an asyncio TCP server that
accepts a connection from the ReSpeaker Lite ESP32, receives raw 16-bit
PCM mic audio, runs openwakeword for wake word detection, and sends
TTS audio back for playback.

TCP protocol: 5-byte header [msg_type(1) | length(4 big-endian)] + payload
  0x01 AUDIO_MIC  ESP32→PC  raw 16-bit PCM @ 16kHz
  0x02 AUDIO_TTS  PC→ESP32  raw 16-bit PCM @ 16kHz
  0x03 TTS_START  PC→ESP32  signals start of TTS playback
  0x04 TTS_END    PC→ESP32  signals end of TTS playback
  0x05 LED_STATE  PC→ESP32  1 byte: 0=idle(blue), 1=wake(magenta), 2=thinking(yellow)
"""

import asyncio
import logging
import struct
import sys

import numpy as np
from openwakeword.model import Model as OWWModel
from scipy.signal import resample_poly

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

# Protocol message types
MSG_AUDIO_MIC = 0x01
MSG_AUDIO_TTS = 0x02
MSG_TTS_START = 0x03
MSG_TTS_END = 0x04
MSG_LED_STATE = 0x05

# LED state values
LED_IDLE = 0
LED_WAKE = 1
LED_THINKING = 2

HEADER_SIZE = 5
SAMPLE_RATE = 16000
BYTES_PER_SAMPLE = 2

# openwakeword expects 80ms chunks (1280 samples at 16kHz)
OWW_CHUNK_SAMPLES = 1280
OWW_CHUNK_BYTES = OWW_CHUNK_SAMPLES * BYTES_PER_SAMPLE


class TCPAudioClient:
    """TCP server that receives mic audio from ESP32 and detects wake words.

    Provides the same interface as ESPHomeClient:
      - connect() / disconnect()
      - on_wake_word callback
      - wait_for_audio(timeout) → bytes
      - get_partial_audio(max_bytes) → bytes
      - send_tts_audio(audio_data)
    """

    def __init__(
        self,
        listen_host: str = "0.0.0.0",
        listen_port: int = 8765,
        wake_word_model: str = "hey_jarvis",
        wake_word_threshold: float = 0.5,
    ):
        self._host = listen_host
        self._port = listen_port
        self._ww_model_name = wake_word_model
        self._ww_threshold = wake_word_threshold

        self._server: asyncio.AbstractServer | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._reader: asyncio.StreamReader | None = None

        # Wake word model
        self._oww: OWWModel | None = None
        self._oww_buf = bytearray()

        # VAD for end-of-speech detection
        self._vad = webrtcvad.Vad(3)

        # Audio buffer for current utterance
        self._audio_buffer = bytearray()
        self._is_recording = False
        self._recording_done = asyncio.Event()

        # VAD state
        self._silence_frames = 0
        self._speech_frames = 0
        self._silence_limit = 48  # ~1.5s at 30ms frames (1500/30)

        # Callbacks
        self.on_wake_word: callable = None

    async def connect(self):
        """Start the TCP server and load the wake word model."""
        logger.info(f"Loading openwakeword model: {self._ww_model_name}")
        self._oww = OWWModel(
            wakeword_models=[self._ww_model_name],
            inference_framework="onnx",
        )
        logger.info("openwakeword model loaded")

        self._server = await asyncio.start_server(
            self._handle_client, self._host, self._port
        )
        addr = self._server.sockets[0].getsockname()
        logger.info(f"TCP audio server listening on {addr[0]}:{addr[1]}")

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ):
        """Handle a single ESP32 client connection."""
        peer = writer.get_extra_info("peername")
        logger.info(f"ESP32 connected from {peer}")

        # Only allow one client at a time
        if self._writer is not None:
            old_writer = self._writer
            self._writer = None
            self._reader = None
            try:
                old_writer.close()
                await old_writer.wait_closed()
            except Exception:
                pass
            logger.info("Replaced previous client connection")

        self._reader = reader
        self._writer = writer

        try:
            while True:
                # Read 5-byte header
                header = await reader.readexactly(HEADER_SIZE)
                msg_type = header[0]
                length = struct.unpack(">I", header[1:5])[0]

                if msg_type == MSG_AUDIO_MIC:
                    payload = await reader.readexactly(length)
                    await self._process_mic_audio(payload)
                else:
                    # Drain unknown message types
                    if length > 0:
                        await reader.readexactly(length)
                    logger.warning(f"Unexpected msg type from ESP32: 0x{msg_type:02X}")

        except asyncio.IncompleteReadError:
            logger.info(f"ESP32 disconnected ({peer})")
        except Exception as e:
            logger.error(f"Client error: {e}")
        finally:
            if self._writer is writer:
                self._writer = None
                self._reader = None
            writer.close()

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
            # Log audio stats periodically to verify signal
            self._debug_counter = getattr(self, "_debug_counter", 0) + 1
            if self._debug_counter % 50 == 0:
                logger.debug(
                    f"audio stats: min={samples.min()} max={samples.max()} "
                    f"rms={np.sqrt(np.mean(samples.astype(np.float32)**2)):.0f} "
                    f"len={len(samples)}"
                )
            samples_f = samples.astype(np.float32) / 32768.0
            prediction = self._oww.predict(samples_f)

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
        # VAD needs 10/20/30ms frames at 16kHz
        frame_bytes = SAMPLE_RATE * BYTES_PER_SAMPLE * 30 // 1000  # 960 bytes = 30ms

        offset = 0
        while offset + frame_bytes <= len(data):
            frame = data[offset : offset + frame_bytes]
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

            # End recording after sustained silence (only if we got some speech)
            if self._speech_frames > 3 and self._silence_frames >= self._silence_limit:
                self._is_recording = False
                self._recording_done.set()
                logger.info(
                    f"End of speech detected ({len(self._audio_buffer)} bytes)"
                )
                return

    def start_direct_listen(self):
        """Start recording immediately without waiting for the wake word."""
        self._start_recording()

    async def wait_for_audio(self, timeout: float = 5.0) -> bytes:
        """Wait for the current utterance audio to complete.

        Args:
            timeout: Maximum wait time in seconds.

        Returns:
            Complete PCM audio buffer for the utterance.
        """
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
        """Get audio buffered so far without waiting.

        Args:
            max_bytes: Maximum bytes to return (0 = all available).

        Returns:
            PCM audio bytes buffered so far.
        """
        if max_bytes and len(self._audio_buffer) > max_bytes:
            return bytes(self._audio_buffer[:max_bytes])
        return bytes(self._audio_buffer)

    async def send_tts_audio(self, audio_data: bytes):
        """Send TTS audio to the ESP32 for speaker playback.

        Resamples from 22050 Hz to 16000 Hz before sending.

        Args:
            audio_data: Raw PCM 16-bit audio at 22050 Hz.
        """
        if not self._writer:
            logger.warning("No ESP32 connected, cannot send TTS")
            return

        # Resample 22050 → 16000 Hz (ratio 320/441)
        samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float64)
        resampled = resample_poly(samples, 320, 441).astype(np.int16)
        pcm_data = resampled.tobytes()

        try:
            # Send TTS_START
            self._writer.write(struct.pack(">BI", MSG_TTS_START, 0))

            # Send audio in chunks
            chunk_size = 4096
            for i in range(0, len(pcm_data), chunk_size):
                chunk = pcm_data[i : i + chunk_size]
                self._writer.write(struct.pack(">BI", MSG_AUDIO_TTS, len(chunk)))
                self._writer.write(chunk)

            # Send TTS_END
            self._writer.write(struct.pack(">BI", MSG_TTS_END, 0))
            await self._writer.drain()
            logger.info(f"Sent TTS audio: {len(pcm_data)} bytes (resampled from {len(audio_data)})")

        except Exception as e:
            logger.error(f"Failed to send TTS audio: {e}")

    async def send_led_state(self, state: int):
        """Send an LED state command to the ESP32.

        Args:
            state: LED_IDLE (0), LED_WAKE (1), or LED_THINKING (2).
        """
        if not self._writer:
            return
        try:
            self._writer.write(struct.pack(">BI", MSG_LED_STATE, 1))
            self._writer.write(bytes([state]))
            await self._writer.drain()
        except Exception as e:
            logger.error(f"Failed to send LED state: {e}")

    async def disconnect(self):
        """Stop the TCP server and close connections."""
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
            self._writer = None
            self._reader = None

        if self._server:
            self._server.close()
            await self._server.wait_closed()
            logger.info("TCP audio server stopped")
