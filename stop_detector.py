"""Fast-path stop keyword detector using Whisper tiny model."""

import numpy as np
from faster_whisper import WhisperModel


class StopDetector:
    """Checks the first chunk of audio for emergency stop keywords.

    Uses the tiny Whisper model for fast inference (~50-100ms).
    """

    def __init__(self, keywords: list[str] | None = None,
                 device: str = "cuda"):
        self.keywords = set(keywords or ["stop", "halt", "freeze", "emergency"])
        self.model = WhisperModel(
            "tiny",
            device=device,
            compute_type="float16" if device == "cuda" else "int8",
        )

    def check(self, pcm_audio: bytes, sample_rate: int = 16000) -> bool:
        """Check if audio contains a stop keyword.

        Args:
            pcm_audio: Raw 16-bit PCM audio (first ~0.5s is enough).
            sample_rate: Sample rate of the audio.

        Returns:
            True if a stop keyword was detected.
        """
        if not pcm_audio:
            return False

        audio_array = np.frombuffer(pcm_audio, dtype=np.int16).astype(np.float32) / 32768.0

        segments, _ = self.model.transcribe(
            audio_array,
            language="en",
            beam_size=1,
            vad_filter=False,
        )

        text = " ".join(seg.text for seg in segments).strip().lower()
        words = set(text.split())
        return bool(words & self.keywords)
