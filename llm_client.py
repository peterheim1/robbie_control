"""Ollama LLM client for general questions with daily short-term memory."""

from datetime import datetime, date
from pathlib import Path

import ollama
import yaml


class LLMClient:
    """Routes general questions to a local Mistral model via Ollama.

    Maintains a short-term memory of today's conversation in time order.
    Previous days' history is automatically pruned.
    """

    def __init__(self, model: str = "mistral",
                 host: str = "http://localhost:11434",
                 system_prompt: str = "",
                 max_tokens: int = 100):
        self.model = model
        base_prompt = system_prompt or (
            "You are Robbie, a home robot assistant with a warm, slightly formal British personality. "
            "Give brief, conversational answers — 1 to 2 sentences maximum. "
            "Always stay in character. Never describe yourself as an AI language model."
        )
        backstory_path = Path(__file__).parent / "config" / "backstory.yaml"
        self.system_prompt = self._build_system_prompt(base_prompt, backstory_path)
        self.max_tokens = max_tokens
        self._client = ollama.Client(host=host)
        # Short-term memory: list of {"ts": datetime, "role": str, "content": str}
        self._history: list[dict] = []

    def _build_system_prompt(self, base_prompt: str, backstory_path: Path) -> str:
        """Append backstory facts to the base system prompt."""
        try:
            with open(backstory_path) as f:
                b = yaml.safe_load(f)
        except FileNotFoundError:
            return base_prompt

        movies = ", ".join(b["favorites"]["movies"])
        music = ", ".join(b["favorites"]["music"])

        persona = f"""
Your name is {b['name']}. You were built by {b['created_by']} in {b['year_built']} \
and are {b['age']}.

Personality: {', '.join(b['personality']['traits'])}.

Favourite movies: {movies}.
Favourite music: {music}.
Favourite book: {b['favorites']['book']}.
Favourite colour: {b['favorites']['colour']}.

Family: {b['family']['summary'].strip()}

Aspiration: {b['aspirations'].strip()}

Always answer in character as Robbie. Keep answers to 1–2 sentences. \
Use a warm, slightly formal British tone. Never claim to be an AI language model.
"""
        return base_prompt.rstrip() + "\n" + persona.strip()

    def health_check(self) -> bool:
        """Check if Ollama is running and the model is available."""
        try:
            models = self._client.list()
            available = [m.model for m in models.models]
            return any(self.model in name for name in available)
        except Exception:
            return False

    def _get_today_history(self) -> list[dict]:
        """Return today's messages in time order, pruning older entries."""
        today = date.today()
        self._history = [e for e in self._history if e["ts"].date() == today]
        return [{"role": e["role"], "content": e["content"]} for e in self._history]

    def get_history_summary(self) -> list[dict]:
        """Return today's history as a list of {time, role, content} dicts for display."""
        today = date.today()
        return [
            {
                "time": e["ts"].strftime("%H:%M:%S"),
                "role": e["role"],
                "content": e["content"],
            }
            for e in self._history
            if e["ts"].date() == today
        ]

    def ask(self, question: str) -> str:
        """Ask a general question and get a text response.

        Today's conversation history is included as context so the LLM
        remembers what was said earlier in the day.

        Args:
            question: The user's question.

        Returns:
            LLM response text.
        """
        today_history = self._get_today_history()
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(today_history)
        messages.append({"role": "user", "content": question})

        try:
            response = self._client.chat(
                model=self.model,
                messages=messages,
                options={"num_predict": self.max_tokens},
            )
            answer = response.message.content.strip()

            # Store exchange in daily memory
            now = datetime.now()
            self._history.append({"ts": now, "role": "user", "content": question})
            self._history.append({"ts": now, "role": "assistant", "content": answer})

            return answer
        except Exception as e:
            return f"I can't answer questions right now: {e}"
