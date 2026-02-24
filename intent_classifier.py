"""Rule-based intent classifier with fuzzy location matching."""

import re
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from pathlib import Path
from typing import TYPE_CHECKING

import yaml

if TYPE_CHECKING:
    from robbie_control.command_loader import CommandDef


@dataclass
class Intent:
    name: str
    utterance: str
    confidence: float = 1.0
    params: dict = field(default_factory=dict)
    response_text: str = ""


class IntentClassifier:
    """Classifies transcribed text into robot command intents."""

    def __init__(self, intents_path: str, locations_path: str,
                 head_positions_path: str,
                 commands: "list[CommandDef] | None" = None):
        with open(intents_path) as f:
            self._intents_cfg = yaml.safe_load(f)["intents"]
        with open(locations_path) as f:
            self._locations = yaml.safe_load(f)["locations"]
        with open(head_positions_path) as f:
            self._head = yaml.safe_load(f)["head"]

        # Build location alias lookup: alias_lower -> (name, location_data)
        self._location_aliases: dict[str, tuple[str, dict]] = {}
        for name, loc in self._locations.items():
            self._location_aliases[name.lower()] = (name, loc)
            for alias in loc.get("aliases", []):
                self._location_aliases[alias.lower()] = (name, loc)

        self._commands: list = commands or []

        # Sort intents by priority
        self._intent_order = sorted(
            self._intents_cfg.items(),
            key=lambda x: x[1].get("priority", 50),
        )

    def classify(self, text: str) -> Intent:
        """Classify transcribed text into an Intent.

        Args:
            text: Lowercased transcription from STT.

        Returns:
            Intent with name, params, confidence, and response_text.
        """
        text = text.strip().lower()
        if not text:
            return Intent(
                name="unknown", utterance=text, confidence=0.0,
                params={"original": text},
                response_text="I didn't catch that",
            )

        # Check commands.txt first (plain phrase matching, no regex)
        if self._commands:
            from robbie_control.command_loader import match_command
            cmd = match_command(text, self._commands)
            if cmd:
                return Intent(
                    name=cmd.name,
                    utterance=text,
                    confidence=1.0,
                    params={"ros_actions": cmd.ros_actions, "flags": cmd.flags},
                    response_text=cmd.say,
                )

        for intent_name, cfg in self._intent_order:
            if cfg.get("catch_all"):
                continue
            if cfg.get("detect_as_question"):
                continue

            for pattern in cfg.get("patterns", []):
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    return self._build_intent(intent_name, cfg, text, match)

        # Check if it's a question (general_question fallback)
        gen_cfg = self._intents_cfg.get("general_question", {})
        starters = gen_cfg.get("question_starters", [])
        first_word = text.split()[0] if text.split() else ""
        if first_word in starters or text.endswith("?"):
            return Intent(
                name="general_question",
                utterance=text,
                confidence=0.8,
                params={"original": text},
                response_text="",
            )

        # Unknown
        unk_cfg = self._intents_cfg.get("unknown", {})
        response = unk_cfg.get("response", "I don't know that command")
        response = response.replace("{utterance}", text)
        return Intent(
            name="unknown",
            utterance=text,
            confidence=0.0,
            params={"original": text},
            response_text=response,
        )

    def _build_intent(self, name: str, cfg: dict, text: str,
                      match: re.Match) -> Intent:
        """Build an Intent from a regex match and config."""
        params = {}
        response = cfg.get("response", "")

        if name == "look":
            direction = self._extract_direction(match, text)
            params["direction"] = direction
            pos = self._head.get("positions", {}).get(direction, {"pan": 0.0, "tilt": 0.0})
            params["pan"] = pos.get("pan", 0.0)
            params["tilt"] = pos.get("tilt", 0.0)
            response = (response or "").replace("{direction}", direction)

        elif name == "goto":
            raw_location = match.group(1).strip() if match.lastindex else text
            loc_name, loc_data = self._resolve_location(raw_location)
            if loc_name:
                params["location"] = loc_name
                params["x"] = loc_data["x"]
                params["y"] = loc_data["y"]
                params["yaw_deg"] = loc_data["yaw_deg"]
                response = (response or "").replace("{location}", raw_location)
            else:
                return Intent(
                    name="unknown", utterance=text, confidence=0.5,
                    params={"original": text},
                    response_text=f"I don't know where {raw_location} is",
                )

        elif name == "point_to":
            target = match.group(1).strip() if match.lastindex else ""
            params["target"] = target
            response = (response or "").replace("{target}", target)

        elif name == "query_status":
            params["subject"] = self._detect_status_subject(text)

        elif name == "list_node_params":
            params["node_name"] = match.group(1).strip() if match.lastindex else ""

        elif name == "run_task":
            task_name = match.group(1).strip() if match.lastindex else ""
            params["task_name"] = task_name
            response = (response or "").replace("{task_name}", task_name)

        elif name == "list_tasks":
            pass  # no params needed — handled in voice server

        return Intent(
            name=name,
            utterance=text,
            confidence=1.0,
            params=params,
            response_text=response or "",
        )

    def _extract_direction(self, match: re.Match, text: str) -> str:
        """Extract and normalize direction from match."""
        raw = match.group(1).strip().lower() if match.lastindex else ""
        synonyms = {
            "straight": "forward",
            "ahead": "forward",
            "front": "forward",
        }
        # Check for "straight ahead" special case
        if "straight ahead" in text:
            return "forward"
        return synonyms.get(raw, raw)

    def _resolve_location(self, raw: str) -> tuple[str | None, dict]:
        """Resolve a location name or alias, with fuzzy fallback.

        Returns:
            (location_name, location_data) or (None, {}) if not found.
        """
        raw_lower = raw.lower().strip()

        # Exact match on name or alias
        if raw_lower in self._location_aliases:
            name, data = self._location_aliases[raw_lower]
            return name, data

        # Fuzzy match (threshold 0.75)
        best_score = 0.0
        best_match = None
        for alias, (name, data) in self._location_aliases.items():
            score = SequenceMatcher(None, raw_lower, alias).ratio()
            if score > best_score:
                best_score = score
                best_match = (name, data)

        if best_score >= 0.75 and best_match:
            return best_match

        return None, {}

    def _detect_status_subject(self, text: str) -> str:
        """Detect which status subject the user is asking about."""
        keyword_map = {
            "battery": ["battery", "charge", "power"],
            "errors": ["error", "fault", "problem"],
            "nodes": ["node", "nodes"],
            "room": ["room", "where"],
            "dock": ["dock", "docked"],
        }
        text_lower = text.lower()
        for subject, keywords in keyword_map.items():
            if any(kw in text_lower for kw in keywords):
                return subject
        return "general"
