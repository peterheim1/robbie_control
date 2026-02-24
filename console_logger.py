"""Simple timestamped console logger for voice interactions."""

from datetime import datetime


class ConsoleLogger:
    """Prints voice interaction events to stdout with timestamps."""

    def _ts(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _sep(self):
        print(f"[{self._ts()}] {'─' * 40}")

    def log_wake(self):
        print(f"[{self._ts()}] WAKE  \"ok nabu\" detected")

    def log_utterance(self, text: str):
        print(f"[{self._ts()}] HEAR  \"{text}\"")

    def log_intent(self, intent_name: str, params: dict):
        params_str = ", ".join(f"{k}={v}" for k, v in params.items()) if params else ""
        if params_str:
            print(f"[{self._ts()}] INTENT {intent_name} → {params_str}")
        else:
            print(f"[{self._ts()}] INTENT {intent_name}")

    def log_fast_stop(self):
        print(f"[{self._ts()}] FAST  stop_all → /voice/stop, /cmd_vel, /drive")

    def log_response(self, text: str):
        print(f"[{self._ts()}] TTS   \"{text}\"")

    def log_llm(self, text: str):
        print(f"[{self._ts()}] LLM   \"{text}\"")

    def log_publish(self, topics: list[str]):
        topics_str = ", ".join(topics)
        print(f"[{self._ts()}] PUB   {topics_str}")

    def log_error(self, msg: str):
        print(f"[{self._ts()}] ERROR {msg}")

    def log_info(self, msg: str):
        print(f"[{self._ts()}] INFO  {msg}")

    def log_end(self):
        self._sep()
