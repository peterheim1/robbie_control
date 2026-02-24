"""Documentation indexer, LLM query engine and history manager for Robbie."""

import asyncio
import json
import logging
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import AsyncGenerator

import httpx
import yaml

logger = logging.getLogger(__name__)

_BASE        = Path("/home/pi/ros2_ws/src")
_PRIME_PATH  = _BASE / "prime_docs"
_SRC_PATH    = _BASE
_ROS2_SETUP  = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/pi/ros2_ws/install/setup.bash && "
)
_EXCLUDE = frozenset({
    "venv", ".venv", "__pycache__", ".git", "node_modules",
    "build", "install", "log",
})
_FAULT_KEYWORDS = frozenset({
    "crash", "crashed", "fail", "failed", "error", "won't start", "not start",
    "stuck", "stall", "stalled", "estop", "fault", "broken", "dead", "missing",
    "debug", "diagnose", "fix", "problem", "issue", "wrong", "bad", "help",
    "not working", "won't", "cant", "cannot",
})


# ── Document chunk ────────────────────────────────────────────────────────────

class DocChunk:
    __slots__ = ("path", "heading", "content", "line_start")

    def __init__(self, path: str, heading: str, content: str, line_start: int):
        self.path       = path
        self.heading    = heading
        self.content    = content
        self.line_start = line_start

    def as_context(self) -> str:
        return f"[{self.path} § {self.heading}]\n{self.content[:900]}"


# ── Engine ────────────────────────────────────────────────────────────────────

class DocsEngine:

    def __init__(
        self,
        ollama_url: str = "http://10.0.0.87:11434",
        model: str      = "mistral",
    ):
        self._ollama_url = ollama_url
        self._model      = model
        self._chunks: list[DocChunk] = []
        self._index_ts   = 0.0

        rc = Path("/home/pi/ros2_ws/src/robbie_control")
        self._history_path = rc / "data" / "query_history.jsonl"
        self._cmds_path    = rc / "config" / "approved_commands.yaml"
        self._history_path.parent.mkdir(exist_ok=True)

    # ── Startup ───────────────────────────────────────────────────────────────

    def startup(self):
        """Pull prime docs and build index. Call once at server startup."""
        self._pull_prime_docs()
        self._build_index()

    _PRIME_REPO = "https://github.com/peterheim1/robbie_prime"

    def _pull_prime_docs(self):
        try:
            if not _PRIME_PATH.exists():
                logger.info(f"Cloning prime docs from {self._PRIME_REPO}")
                r = subprocess.run(
                    ["git", "clone", self._PRIME_REPO, str(_PRIME_PATH)],
                    capture_output=True, text=True, timeout=60,
                )
                logger.info(f"prime_docs clone: {(r.stdout or r.stderr).strip()}")
            else:
                r = subprocess.run(
                    ["git", "pull"], cwd=_PRIME_PATH,
                    capture_output=True, text=True, timeout=15,
                )
                logger.info(f"prime_docs pull: {(r.stdout or r.stderr).strip()}")
        except Exception as e:
            logger.warning(f"prime_docs sync skipped: {e}")

    # ── Indexing ──────────────────────────────────────────────────────────────

    def _md_files(self) -> list[Path]:
        roots = [_SRC_PATH]
        if _PRIME_PATH.exists():
            roots.append(_PRIME_PATH)
        out = []
        for root in roots:
            for p in root.rglob("*.md"):
                if not _EXCLUDE.intersection(p.parts):
                    out.append(p)
        return out

    def _parse_file(self, path: Path) -> list[DocChunk]:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            return []
        try:
            rel = str(path.relative_to(_SRC_PATH))
        except ValueError:
            rel = str(path)

        chunks: list[DocChunk] = []
        heading, start, buf = "intro", 0, []
        for i, line in enumerate(text.splitlines()):
            if line.startswith("#"):
                if buf:
                    content = "\n".join(buf).strip()
                    if len(content) > 30:
                        chunks.append(DocChunk(rel, heading, content, start))
                heading, start, buf = line.lstrip("#").strip(), i + 1, []
            else:
                buf.append(line)
        if buf:
            content = "\n".join(buf).strip()
            if len(content) > 30:
                chunks.append(DocChunk(rel, heading, content, start))
        return chunks

    def _build_index(self):
        files  = self._md_files()
        chunks = [c for f in files for c in self._parse_file(f)]
        self._chunks   = chunks
        self._index_ts = time.monotonic()
        logger.info(f"Docs indexed: {len(chunks)} chunks from {len(files)} files")

    def _refresh_if_stale(self):
        if time.monotonic() - self._index_ts > 60:
            self._build_index()

    # ── Search ────────────────────────────────────────────────────────────────

    def search(self, query: str, top_k: int = 8) -> list[DocChunk]:
        self._refresh_if_stale()
        words = {w for w in query.lower().split() if len(w) > 2}
        scored: list[tuple[int, DocChunk]] = []
        for chunk in self._chunks:
            text  = f"{chunk.heading} {chunk.content}".lower()
            score = sum(1 for w in words if w in text)
            if score:
                scored.append((score, chunk))
        scored.sort(key=lambda x: -x[0])
        return [c for _, c in scored[:top_k]]

    # ── Query classification ──────────────────────────────────────────────────

    def is_fault_query(self, query: str) -> bool:
        q = query.lower()
        return any(kw in q for kw in _FAULT_KEYWORDS)

    def detect_ros2_query(self, query: str) -> str | None:
        """Return a safe ROS2 read-only shell command if query matches, else None."""
        q, parts = query.lower().strip(), query.split()
        for prefix in ("list params for", "list parameters for", "params for"):
            if q.startswith(prefix) and parts:
                return f"ros2 param list {parts[-1]}"
        if q.startswith("get param ") and len(parts) >= 4:
            return f"ros2 param get {parts[2]} {parts[3]}"
        if ("topic echo" in q) and len(parts) >= 3:
            topic = next((p for p in parts if p.startswith("/")), parts[-1])
            return f"ros2 topic echo {topic} --once"
        if q in ("topic list", "list topics", "ros2 topic list"):
            return "ros2 topic list"
        if q in ("node list", "list nodes", "ros2 node list"):
            return "ros2 node list"
        return None

    # ── Approved commands ─────────────────────────────────────────────────────

    def load_commands(self) -> list[dict]:
        try:
            return yaml.safe_load(self._cmds_path.read_text()).get("commands", [])
        except Exception as e:
            logger.error(f"approved_commands.yaml: {e}")
            return []

    def get_command(self, cmd_id: str) -> dict | None:
        return next((c for c in self.load_commands() if c["id"] == cmd_id), None)

    def relevant_commands(self, query: str, answer: str) -> list[dict]:
        text = f"{query} {answer}".lower()
        out  = []
        for cmd in self.load_commands():
            words = [w for w in cmd["label"].lower().split() if len(w) > 3]
            if any(w in text for w in words):
                out.append({"id": cmd["id"], "label": cmd["label"]})
        return out

    # ── LLM streaming ─────────────────────────────────────────────────────────

    async def stream_answer(
        self,
        query:       str,
        chunks:      list[DocChunk],
        diagnostics: str = "",
    ) -> AsyncGenerator[str, None]:
        context   = "\n\n---\n\n".join(c.as_context() for c in chunks)
        diag_part = f"\n\nLive diagnostics:\n{diagnostics}" if diagnostics else ""
        prompt = (
            f"Documentation excerpts:\n{context}{diag_part}\n\n"
            f"Question: {query}\n\n"
            f"Answer using ONLY the excerpts above. "
            f"If the excerpts do not contain enough information to answer, "
            f"say exactly what they do say and state clearly what is missing. "
            f"Do NOT add information from outside the excerpts. "
            f"If suggesting a command, copy it exactly from the documentation."
        )
        system = (
            "You are Robbie's technical assistant. "
            "Your ONLY source of truth is the documentation excerpts provided in the prompt. "
            "Never use knowledge from your training data. "
            "If something is not in the excerpts, say so — do not guess or infer. "
            "Cite sources as [file § section]."
        )
        payload = {
            "model":  self._model,
            "prompt": prompt,
            "system": system,
            "stream": True,
            "options": {
                "temperature": 0.1,   # low = factual, high = creative/hallucinatory
                "num_predict": 400,   # cap response length to reduce drift
            },
        }
        try:
            async with httpx.AsyncClient(timeout=60) as client:
                async with client.stream(
                    "POST", f"{self._ollama_url}/api/generate", json=payload
                ) as resp:
                    if resp.status_code != 200:
                        yield f"⚠ LLM error: HTTP {resp.status_code}"
                        return
                    async for line in resp.aiter_lines():
                        if not line:
                            continue
                        try:
                            d = json.loads(line)
                            if t := d.get("response"):
                                yield t
                            if d.get("done"):
                                return
                        except json.JSONDecodeError:
                            continue
        except httpx.ConnectError:
            yield "⚠ LLM unavailable — Ollama not reachable at 10.0.0.87"
        except Exception as e:
            yield f"⚠ LLM error: {e}"

    # ── Live diagnostics ──────────────────────────────────────────────────────

    async def get_diagnostics(self) -> str:
        """Fetch /health_summary and running node list."""
        results = []
        cmds = [
            ("health",  "ros2 topic echo /health_summary --once --no-daemon"),
            ("nodes",   "ros2 node list"),
        ]
        for label, cmd in cmds:
            try:
                proc = await asyncio.create_subprocess_shell(
                    _ROS2_SETUP + cmd,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.DEVNULL,
                    executable="/bin/bash",
                )
                out, _ = await asyncio.wait_for(proc.communicate(), timeout=4)
                text = out.decode(errors="replace").strip()
                if text:
                    results.append(f"[{label}]\n{text}")
            except Exception as e:
                results.append(f"[{label}] unavailable: {e}")
        return "\n\n".join(results)

    # ── ROS2 live query ───────────────────────────────────────────────────────

    async def run_ros2_query(self, cmd: str) -> str:
        """Execute a safe ROS2 read-only command and return output."""
        full = _ROS2_SETUP + cmd
        try:
            proc = await asyncio.create_subprocess_shell(
                full,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                executable="/bin/bash",
            )
            out, _ = await asyncio.wait_for(proc.communicate(), timeout=8)
            return out.decode(errors="replace").strip() or "(no output)"
        except asyncio.TimeoutError:
            return "(timed out after 8 s)"
        except Exception as e:
            return f"(error: {e})"

    # ── History ───────────────────────────────────────────────────────────────

    def save_history(self, query: str, answer: str, sources: list[str]):
        entry = {
            "ts":      datetime.now().isoformat(),
            "query":   query,
            "answer":  answer,
            "sources": sources,
        }
        try:
            with open(self._history_path, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception as e:
            logger.error(f"history write: {e}")

    def load_history(self, limit: int = 20) -> list[dict]:
        if not self._history_path.exists():
            return []
        try:
            lines   = self._history_path.read_text().splitlines()
            entries: list[dict] = []
            for line in reversed(lines):
                if line.strip():
                    try:
                        entries.append(json.loads(line))
                    except Exception:
                        pass
                    if len(entries) >= limit:
                        break
            return entries
        except Exception:
            return []
