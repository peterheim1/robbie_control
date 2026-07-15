"""Classifies a spoken reply to an event-bound curiosity question (FSD §9.2).

Pure text classification — no ROS/asyncio dependencies — so it's easy to unit
test in isolation from the voice pipeline.
"""

import re

# "I don't know" / uncertainty patterns.
_UNKNOWN_PATTERNS = [
    r"\bi (?:don'?t|do not) know\b",
    r"\bno idea\b",
    r"\bnot sure\b",
    r"\bcan'?t tell\b",
    r"\bno clue\b",
]

# Refusal phrases beyond a bare "no" (checked separately below).
_DECLINE_PATTERNS = [
    r"\bi(?:'d| would)? (?:rather |prefer to )?not (?:say|tell you|answer)\b",
    r"\bnone of your business\b",
    r"\b(?:don'?t|do not) want to (?:say|answer|tell you)\b",
]
_DECLINE_WORDS = {"no", "nope", "nah", "negative", "skip", "pass"}

# "no, that's a thermos" / "it's actually a mug" style corrections.
_CORRECTION_RE = re.compile(
    r"\bno,?\s+(?:actually,?\s+)?(?:that'?s|it'?s|that is|it is)\s+"
    r"(?:actually\s+)?(?:a|an|the)\s+.+",
    re.IGNORECASE,
)


def classify_answer(transcript: str, stop_keywords: set[str],
                    known_intent_name: str | None) -> str:
    """Classify a transcript into a CuriosityAnswer answer_type (FSD §9.2).

    Args:
        transcript: STT output for the spoken reply (may be empty on silence).
        stop_keywords: word set to match against (reuse StopDetector.keywords).
        known_intent_name: name of the intent the transcript classifies as via
            IntentClassifier.classify(), or None/"unknown"/"general_question"
            if it isn't a recognized command.

    Returns:
        One of ANSWER|UNKNOWN|DECLINE|STOP|CORRECTION|NO_RESPONSE|COMMAND.
    """
    text = (transcript or "").strip()
    if not text:
        return "NO_RESPONSE"

    lower = text.lower()
    words = set(re.findall(r"[a-z']+", lower))

    if words & stop_keywords:
        return "STOP"

    if known_intent_name and known_intent_name not in ("unknown", "general_question"):
        return "COMMAND"

    for pattern in _UNKNOWN_PATTERNS:
        if re.search(pattern, lower):
            return "UNKNOWN"

    stripped = lower.strip(" .!?")
    if stripped in _DECLINE_WORDS:
        return "DECLINE"

    if _CORRECTION_RE.search(text):
        return "CORRECTION"

    for pattern in _DECLINE_PATTERNS:
        if re.search(pattern, lower):
            return "DECLINE"

    return "ANSWER"
