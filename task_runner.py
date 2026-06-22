"""Task sequence runner — loads and executes .txt task files.

File format (one step per line, blank lines ignored):
    NAV:LocationName          navigate to a named location via Nav2
    TASK:MakeSound:some text  speak text via TTS
    TASK:Wait                 pause 5 seconds
    TASK:Dock                 call start_docking service
    TASK:Undock               call undock service
    TASK:LookAt               head look-at (placeholder for future arm poses)
"""

import asyncio
import json
import logging
import threading
from pathlib import Path

import yaml

logger = logging.getLogger(__name__)


class TaskRunner:
    """Loads and executes named task sequence files from a directory.

    Tasks run asynchronously and can be cancelled with cancel().
    Only one task runs at a time — a new run_task() call cancels the current one.
    """

    WAIT_SECS = 5.0

    def __init__(
        self,
        tasks_dir: str,
        locations_path: str,
        speak_fn,
        loop: asyncio.AbstractEventLoop,
        status_fn=None,
    ):
        self._tasks_dir = Path(tasks_dir)
        self._speak_fn = speak_fn
        self._loop = loop
        self._status_fn = status_fn   # optional async fn(name, step, running)
        self._cancelled = False
        self._current_task: str | None = None
        self._current_step: str | None = None
        self._task_lock = asyncio.Lock()

        # Load locations: {canonical_name: {x, y, yaw_deg, aliases, ...}}
        self._locations: dict = {}
        self._load_locations(locations_path)

        # Lazy ROS2 node — only created when first nav/dock step runs
        self._ros_node = None
        self._ros_executor = None
        self._spin_thread = None
        self._bt_cmd_pub = None
        self._bt_status_cmd: str = ""
        self._bt_status_lock = threading.Lock()
        self._dock_client = None
        self._undock_client = None

    # ------------------------------------------------------------------
    # Locations
    # ------------------------------------------------------------------

    def _load_locations(self, path: str):
        try:
            with open(path) as f:
                raw = yaml.safe_load(f).get("locations", {})
            self._locations = raw
        except Exception as e:
            logger.warning(f"TaskRunner: could not load locations: {e}")

    def _resolve_location(self, name: str) -> tuple[str, dict] | None:
        """Find (canonical_name, location_data) by name or alias (case-insensitive)."""
        key = name.lower().replace(" ", "_")
        if key in self._locations:
            return (key, self._locations[key])
        for canon_key, loc_data in self._locations.items():
            for alias in loc_data.get("aliases", []):
                if alias.lower() == name.lower():
                    return (canon_key, loc_data)
        return None

    # ------------------------------------------------------------------
    # ROS2 lazy init
    # ------------------------------------------------------------------

    def _ensure_ros_node(self):
        """Create the task runner's own ROS2 node on first use and spin it."""
        if self._ros_node is not None:
            return
        try:
            import rclpy
            import rclpy.executors
            from rclpy.node import Node
            from std_msgs.msg import String as StringMsg
            self._ros_node = Node("robbie_task_runner")

            # Argus command publisher and status subscriber
            self._bt_cmd_pub = self._ros_node.create_publisher(
                StringMsg, "/bt_command", 10)
            self._ros_node.create_subscription(
                StringMsg, "/bt_status", self._on_bt_status, 10)

            try:
                from std_srvs.srv import Empty
                self._dock_client = self._ros_node.create_client(Empty, "start_docking")
                self._undock_client = self._ros_node.create_client(Empty, "undock")
            except Exception:
                pass

            self._ros_executor = rclpy.executors.SingleThreadedExecutor()
            self._ros_executor.add_node(self._ros_node)
            self._spin_thread = threading.Thread(
                target=self._ros_executor.spin,
                daemon=True,
                name="task_runner_spin",
            )
            self._spin_thread.start()

            logger.info("TaskRunner: ROS2 node initialised")
        except Exception as e:
            logger.error(f"TaskRunner: ROS2 init failed: {e}")

    def _on_bt_status(self, msg):
        """Cache current command from /bt_status JSON."""
        try:
            data = json.loads(msg.data)
            with self._bt_status_lock:
                self._bt_status_cmd = data.get("cmd", "")
        except Exception:
            pass

    async def _poll_until_ready(self, ready_fn, timeout: float = 5.0):
        """Async-poll ready_fn() until True or timeout (seconds)."""
        deadline = asyncio.get_running_loop().time() + timeout
        while not ready_fn():
            if asyncio.get_running_loop().time() >= deadline:
                raise asyncio.TimeoutError
            await asyncio.sleep(0.2)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def list_tasks(self) -> list[str]:
        """Return sorted list of task names (filenames without .txt)."""
        if not self._tasks_dir.exists():
            return []
        return sorted(p.stem for p in self._tasks_dir.glob("*.txt"))

    def parse_task(self, task_name: str) -> list[str]:
        """Return non-blank step strings from a task file."""
        path = self._tasks_dir / f"{task_name}.txt"
        if not path.exists():
            return []
        return [l.strip() for l in path.read_text().splitlines() if l.strip()]

    def cancel(self):
        """Cancel the currently running task."""
        self._cancelled = True

    @property
    def current_task(self) -> str | None:
        return self._current_task

    @property
    def current_step(self) -> str | None:
        return self._current_step

    async def run_task(self, task_name: str):
        """Load and execute a named task. Cancels any running task first."""
        # Cancel previous task if running
        if self._current_task:
            self.cancel()
            await asyncio.sleep(0.3)

        steps = self.parse_task(task_name)
        if not steps:
            await self._speak(f"I couldn't find a task called {task_name}")
            return

        async with self._task_lock:
            self._cancelled = False
            self._current_task = task_name
            total = len(steps)
            logger.info(f"[TASK] Starting '{task_name}' ({total} steps)")
            await self._broadcast(task_name, "starting", True)
            await self._speak(f"Starting task {task_name}")

            for i, step in enumerate(steps, 1):
                if self._cancelled:
                    logger.info(f"[TASK] '{task_name}' cancelled at step {i}")
                    await self._speak("Task cancelled")
                    await self._broadcast(task_name, "cancelled", False)
                    break

                self._current_step = f"{i}/{total}: {step}"
                logger.info(f"[TASK] {self._current_step}")
                await self._broadcast(task_name, self._current_step, True)

                try:
                    await self._execute_step(step)
                except asyncio.CancelledError:
                    self._cancelled = True
                except Exception as e:
                    logger.error(f"[TASK] Step failed: {e}")
            else:
                logger.info(f"[TASK] '{task_name}' complete")
                await self._broadcast(task_name, "complete", False)

            self._current_task = None
            self._current_step = None

    # ------------------------------------------------------------------
    # Step execution
    # ------------------------------------------------------------------

    async def _execute_step(self, step: str):
        if step.startswith("NAV:"):
            await self._step_navigate(step[4:].strip())

        elif step.startswith("TASK:"):
            parts = step.split(":", 2)
            task_type = parts[1].strip()
            arg = parts[2].strip() if len(parts) > 2 else None

            if task_type == "MakeSound":
                await self._speak(arg or "")
            elif task_type == "Wait":
                await asyncio.sleep(self.WAIT_SECS)
            elif task_type == "Dock":
                await self._step_dock()
            elif task_type == "Undock":
                await self._step_undock()
            elif task_type == "LookAt":
                pass  # future: arm / head pose
            else:
                logger.warning(f"[TASK] Unknown task type: {task_type}")
        else:
            logger.warning(f"[TASK] Unknown step format: {step}")

    async def _speak(self, text: str):
        if text:
            await self._speak_fn(text)

    async def _broadcast(self, name: str, step: str, running: bool):
        if self._status_fn:
            try:
                await self._status_fn(name, step, running)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Navigation (Nav2 action, fully async via Future bridge)
    # ------------------------------------------------------------------

    async def _step_navigate(self, loc_name: str):
        self._ensure_ros_node()
        if not self._bt_cmd_pub:
            logger.warning("[TASK] NAV skipped — bt_command not available")
            return

        result = self._resolve_location(loc_name)
        if not result:
            logger.warning(f"[TASK] Unknown location: {loc_name}")
            await self._speak(f"I don't know where {loc_name} is")
            return

        canonical_name, _ = result
        await self._speak(f"navigating to {loc_name}")

        from std_msgs.msg import String as StringMsg
        cmd_msg = StringMsg()
        cmd_msg.data = json.dumps({"cmd": "goto_location", "location": canonical_name})
        self._bt_cmd_pub.publish(cmd_msg)

        loop = asyncio.get_running_loop()

        # Wait for argus to pick up the command (bt_status.cmd → "goto_location")
        pickup_deadline = loop.time() + 10.0
        while True:
            with self._bt_status_lock:
                if self._bt_status_cmd == "goto_location":
                    break
            if loop.time() > pickup_deadline:
                logger.error(f"[TASK] Argus did not pick up nav to {loc_name}")
                return
            await asyncio.sleep(0.2)

        # Wait for completion (bt_status.cmd → "" after ClearCommand)
        nav_deadline = loop.time() + 300.0
        while True:
            with self._bt_status_lock:
                if self._bt_status_cmd == "":
                    break
            if self._cancelled or loop.time() > nav_deadline:
                if loop.time() > nav_deadline:
                    logger.error(f"[TASK] Nav to {loc_name} timed out after 300s")
                break
            await asyncio.sleep(0.5)

        logger.info(f"[TASK] Nav to {loc_name}: complete")

    # ------------------------------------------------------------------
    # Docking / undocking services (fully async via Future bridge)
    # ------------------------------------------------------------------

    @staticmethod
    def _bridge(rclpy_future, loop: asyncio.AbstractEventLoop):
        """Bridge an rclpy Future onto the asyncio event loop."""
        aio_future = loop.create_future()

        def _done(f):
            if aio_future.done():
                return
            exc = f.exception()
            if exc is not None:
                loop.call_soon_threadsafe(aio_future.set_exception, exc)
            else:
                loop.call_soon_threadsafe(aio_future.set_result, f.result())

        rclpy_future.add_done_callback(_done)
        return aio_future

    async def _call_service(self, client, name: str):
        from std_srvs.srv import Empty
        loop = asyncio.get_running_loop()
        try:
            await self._poll_until_ready(client.service_is_ready, timeout=5.0)
        except asyncio.TimeoutError:
            logger.error(f"[TASK] {name} service not available")
            return
        try:
            await asyncio.wait_for(
                self._bridge(client.call_async(Empty.Request()), loop),
                timeout=30.0,
            )
        except asyncio.TimeoutError:
            logger.error(f"[TASK] {name} service call timed out")

    async def _step_dock(self):
        self._ensure_ros_node()
        if not self._dock_client:
            logger.warning("[TASK] Dock skipped — service not available")
            return
        await self._call_service(self._dock_client, "start_docking")

    async def _step_undock(self):
        self._ensure_ros_node()
        if not self._undock_client:
            logger.warning("[TASK] Undock skipped — service not available")
            return
        await self._speak("undocking")
        await self._call_service(self._undock_client, "undock")

    def shutdown(self):
        try:
            if self._ros_executor:
                self._ros_executor.shutdown()
        except Exception:
            pass
        try:
            if self._ros_node:
                self._ros_node.destroy_node()
        except Exception:
            pass
