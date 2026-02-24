"""Faster-Whisper speech-to-text engine with CUDA support."""

import io
import wave
import numpy as np
from faster_whisper import WhisperModel


class STTEngine:
    """Wraps Faster-Whisper for transcribing PCM audio buffers."""

    def __init__(self, model_size: str = "small", device: str = "cuda",
                 language: str = "en"):
        self.language = language
        self.model = WhisperModel(
            model_size,
            device=device,
            compute_type="float16" if device == "cuda" else "int8",
        )

    def transcribe(self, pcm_audio: bytes, sample_rate: int = 16000) -> str:
        """Transcribe 16-bit PCM audio bytes to text.

        Args:
            pcm_audio: Raw 16-bit signed little-endian PCM bytes.
            sample_rate: Sample rate of the audio (default 16kHz).

        Returns:
            Transcribed text string, stripped and lowercased.
        """
        if not pcm_audio:
            return ""

        audio_array = np.frombuffer(pcm_audio, dtype=np.int16).astype(np.float32) / 32768.0

        segments, _ = self.model.transcribe(
            audio_array,
            language=self.language,
            beam_size=5,
            vad_filter=True,
        )

        text = " ".join(seg.text for seg in segments).strip()
        return text.lower()
