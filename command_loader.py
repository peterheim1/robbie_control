"""Loads config/commands.txt — human-readable phrase-to-ROS-action mappings.

File format
-----------
# comment
[command name]
phrases: phrase one, phrase two, ...
say:     spoken response
flags:   cancel_tasks           (optional)
ros:     type:topic  key=val    (one or more lines)

ros types:
  none
  empty:/topic
  twist:/topic
  joint_traj:/topic  joint1=rad joint2=rad
  float64_array:/topic  values=v1,v2,v3
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import logging

logger = logging.getLogger(__name__)


@dataclass
class RosAction:
    """One ROS publish step from a 'ros:' line."""
    action_type: str        # empty | twist | joint_traj | float64_array | none
    topic: str
    params: dict = field(default_factory=dict)


@dataclass
class CommandDef:
    """One voice command block from commands.txt."""
    name: str
    phrases: list[str]
    say: str
    ros_actions: list[RosAction] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)


# Words ignored during keyword-overlap matching
_STOP_WORDS = frozenset({
    "a", "an", "the", "to", "your", "my", "please",
    "can", "you", "me", "i", "it", "this", "that", "its",
})


def load_commands(path: str) -> list[CommandDef]:
    """Parse commands.txt and return list of CommandDef objects."""
    commands: list[CommandDef] = []
    current: CommandDef | None = None

    for raw in Path(path).read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        if line.startswith("[") and line.endswith("]"):
            if current is not None:
                commands.append(current)
            current = CommandDef(name=line[1:-1].strip(), phrases=[], say="")
            continue

        if current is None:
            continue

        if line.startswith("phrases:"):
            current.phrases = [p.strip() for p in line[8:].split(",") if p.strip()]
        elif line.startswith("say:"):
            current.say = line[4:].strip()
        elif line.startswith("flags:"):
            current.flags = [f.strip() for f in line[6:].split(",") if f.strip()]
        elif line.startswith("ros:"):
            action = _parse_ros(line[4:].strip())
            if action:
                current.ros_actions.append(action)

    if current is not None:
        commands.append(current)

    logger.info(f"Loaded {len(commands)} commands from {path}")
    return commands


def match_command(text: str, commands: list[CommandDef]) -> CommandDef | None:
    """Find the best matching command for a transcribed utterance.

    Matching passes (first hit wins):
      1. Exact phrase match
      2. Phrase is a substring of the utterance
      3. All significant words of a phrase appear in the utterance
    """
    t = text.strip().lower()
    t_words = set(t.split()) - _STOP_WORDS

    # Pass 1 — exact
    for cmd in commands:
        for phrase in cmd.phrases:
            if phrase.lower() == t:
                return cmd

    # Pass 2 — substring
    for cmd in commands:
        for phrase in cmd.phrases:
            if phrase.lower() in t:
                return cmd

    # Pass 3 — all significant words present
    for cmd in commands:
        for phrase in cmd.phrases:
            sig = set(phrase.lower().split()) - _STOP_WORDS
            if sig and sig.issubset(t_words):
                return cmd

    return None


def _parse_ros(spec: str) -> RosAction | None:
    """Parse the value of a 'ros:' line.

    Examples::
        none
        empty:/voice/stop
        twist:/cmd_vel
        joint_traj:/head_controller/joint_trajectory  head_pan_joint=-1.0 head_tilt_joint=0.0
        float64_array:/drive  values=0,0,0,0
        srv_empty:/undock
    """
    if not spec or spec == "none":
        return RosAction(action_type="none", topic="")

    parts = spec.split()
    if ":" not in parts[0]:
        logger.warning(f"commands.txt: bad ros line (missing type:topic): {spec!r}")
        return None

    action_type, topic = parts[0].split(":", 1)
    params: dict = {}

    for part in parts[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            if "," in v:
                try:
                    params[k] = [float(x) for x in v.split(",")]
                except ValueError:
                    params[k] = v
            else:
                try:
                    params[k] = float(v)
                except ValueError:
                    params[k] = v

    return RosAction(action_type=action_type, topic=topic, params=params)
