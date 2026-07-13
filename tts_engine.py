"""Piper TTS engine for generating speech audio."""

import io
import os
import wave

from piper.voice import PiperVoice


class TTSEngine:
    """Wraps Piper TTS to synthesize text to PCM audio.

    Loads the ONNX voice model once at construction and keeps it resident,
    since a fresh piper subprocess per utterance cost ~4s of model-load
    overhead on every call.
    """

    def __init__(self, model: str = "en_US-lessac-medium",
                 output_sample_rate: int = 22050):
        self.output_sample_rate = output_sample_rate
        # Resolve model: if it's a path to an .onnx file, use it directly;
        # otherwise look in the models/piper directory next to this file.
        if os.path.isfile(model):
            self.model = model
        else:
            models_dir = os.path.join(os.path.dirname(__file__), "models", "piper")
            onnx_path = os.path.join(models_dir, f"{model}.onnx")
            if os.path.isfile(onnx_path):
                self.model = onnx_path
            else:
                self.model = model  # let piper try to resolve it

        self._voice = PiperVoice.load(self.model)

    def synthesize(self, text: str) -> bytes:
        """Synthesize text to raw 16-bit PCM audio bytes.

        Args:
            text: Text to speak.

        Returns:
            Raw 16-bit signed little-endian PCM bytes at output_sample_rate.
        """
        if not text:
            return b""

        return b"".join(
            chunk.audio_int16_bytes for chunk in self._voice.synthesize(text)
        )

    def synthesize_wav(self, text: str) -> bytes:
        """Synthesize text to a WAV file in memory.

        Args:
            text: Text to speak.

        Returns:
            Complete WAV file bytes.
        """
        pcm = self.synthesize(text)
        if not pcm:
            return b""

        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.output_sample_rate)
            wf.writeframes(pcm)
        return buf.getvalue()
