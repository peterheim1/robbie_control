"""Piper TTS engine for generating speech audio."""

import io
import os
import sys
import wave
import subprocess
import shutil


class TTSEngine:
    """Wraps Piper TTS to synthesize text to PCM audio."""

    def __init__(self, model: str = "en_US-lessac-medium",
                 output_sample_rate: int = 22050):
        self.output_sample_rate = output_sample_rate
        # Check venv bin dir first, then PATH
        venv_bin = os.path.join(os.path.dirname(sys.executable), "piper")
        if os.path.isfile(venv_bin) and os.access(venv_bin, os.X_OK):
            self._piper_path = venv_bin
        else:
            self._piper_path = shutil.which("piper")
        if not self._piper_path:
            raise RuntimeError(
                "piper not found in PATH. Install with: pip install piper-tts"
            )
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

    def synthesize(self, text: str) -> bytes:
        """Synthesize text to raw 16-bit PCM audio bytes.

        Args:
            text: Text to speak.

        Returns:
            Raw 16-bit signed little-endian PCM bytes at output_sample_rate.
        """
        if not text:
            return b""

        result = subprocess.run(
            [
                self._piper_path,
                "--model", self.model,
                "--output-raw",
            ],
            input=text.encode("utf-8"),
            capture_output=True,
            timeout=10,
        )

        if result.returncode != 0:
            raise RuntimeError(f"Piper TTS failed: {result.stderr.decode()}")

        return result.stdout

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
