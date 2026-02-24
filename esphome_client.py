"""ESPHome Native API client for voice satellite communication."""

import asyncio
import logging

from aioesphomeapi import (
    APIClient,
    VoiceAssistantEventType,
)

logger = logging.getLogger(__name__)


class ESPHomeClient:
    """Connects to ReSpeaker Lite via ESPHome Native API.

    Handles voice pipeline events: wake word detection, audio streaming,
    and TTS audio playback.
    """

    def __init__(self, host: str, port: int = 6053,
                 password: str = "", encryption_key: str = ""):
        self._host = host
        self._port = port
        self._password = password
        self._encryption_key = encryption_key
        self._client: APIClient | None = None
        self._unsub: callable = None

        # Audio buffer for current utterance
        self._audio_buffer = bytearray()
        self._is_recording = False
        self._recording_done = asyncio.Event()

        # Callbacks set by the server
        self.on_wake_word: callable = None
        self.on_pipeline_finished: callable = None

    async def connect(self):
        """Connect to the ESPHome device."""
        noise_psk = self._encryption_key if self._encryption_key else None
        self._client = APIClient(
            address=self._host,
            port=self._port,
            password=self._password,
            noise_psk=noise_psk,
        )
        await self._client.connect(login=True)
        info = await self._client.device_info()
        logger.info(f"Connected to ESPHome device: {info.name}")

    def start_voice_pipeline(self):
        """Register as the voice assistant handler.

        This tells the ESP32 that we handle the voice pipeline.
        When the ESP32 detects a wake word, it will stream audio to us.
        """

        async def _handle_start(
            conversation_id: str,
            flags: int,
            audio_settings,
            wake_word_phrase: str | None,
        ) -> int | None:
            """Called when ESP32 starts a voice pipeline (wake word detected)."""
            self._audio_buffer = bytearray()
            self._is_recording = True
            self._recording_done.clear()
            logger.info(f"Voice pipeline started (wake: {wake_word_phrase})")
            if self.on_wake_word:
                self.on_wake_word()
            return None

        async def _handle_stop(abort: bool) -> None:
            """Called when ESP32 ends the voice pipeline."""
            self._is_recording = False
            self._recording_done.set()
            logger.info(f"Voice pipeline stopped (abort={abort})")

        async def _handle_audio(data: bytes) -> None:
            """Called for each audio chunk from the ESP32."""
            if self._is_recording:
                self._audio_buffer.extend(data)

        self._unsub = self._client.subscribe_voice_assistant(
            handle_start=_handle_start,
            handle_stop=_handle_stop,
            handle_audio=_handle_audio,
        )
        logger.info("Subscribed to voice assistant pipeline")

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
        """Send TTS audio back to the ESP32 for speaker playback.

        Args:
            audio_data: Raw PCM audio bytes to play on the ESP32 speaker.
        """
        if self._client:
            chunk_size = 1024
            for i in range(0, len(audio_data), chunk_size):
                chunk = audio_data[i:i + chunk_size]
                self._client.send_voice_assistant_audio(chunk)
            self._client.send_voice_assistant_event(
                VoiceAssistantEventType.VOICE_ASSISTANT_TTS_END, {}
            )

    async def disconnect(self):
        """Disconnect from the ESPHome device."""
        if self._unsub:
            self._unsub()
        if self._client:
            await self._client.disconnect()
            logger.info("Disconnected from ESPHome device")
