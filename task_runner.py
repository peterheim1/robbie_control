"""Task sequence runner — loads and executes .txt task files.

File format (one step per line, blank lines ignored):
    NAV:LocationName          navigate to a named location via Nav2
    TASK:MakeSound:some text  speak text via TTS
    TASK:Wait                 pause 5 seconds
    TASK:Dock                 call start_docking service
    TASK:LookAt               head look-at (placeholder for future arm poses)
"""

import asyncio
import logging
import math
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
        self._nav_client = None
        self._dock_client = None
        self._NavigateToPose = None

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

    def _resolve_location(self, name: str) -> dict | None:
        """Find location data by canonical name or alias (case-insensitive)."""
        key = name.lower().replace(" ", "_")
        if key in self._locations:
            return self._locations[key]
        # Try aliases
        for loc_data in self._locations.values():
            for alias in loc_data.get("aliases", []):
                if alias.lower() == name.lower():
                    return loc_data
        return None

    # ------------------------------------------------------------------
    # ROS2 lazy init
    # ------------------------------------------------------------------

    def _ensure_ros_node(self):
        """Create the task runner's own ROS2 node on first use."""
        if self._ros_node is not None:
            return
        try:
            import rclpy
            from rclpy.node import Node
            self._ros_node = Node("robbie_task_runner")

            try:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient
                self._NavigateToPose = NavigateToPose
                self._nav_client = ActionClient(
                    self._ros_node, NavigateToPose, "navigate_to_pose"
                )
            except ImportError:
                logger.warning("TaskRunner: nav2_msgs not available — NAV steps will skip")

            try:
                from std_srvs.srv import Empty
                self._dock_client = self._ros_node.create_client(Empty, "start_docking")
            except Exception:
                pass

            logger.info("TaskRunner: ROS2 node initialised")
        except Exception as e:
            logger.error(f"TaskRunner: ROS2 init failed: {e}")

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
    # Navigation (Nav2 action, blocking in thread pool)
    # ------------------------------------------------------------------

    async def _step_navigate(self, loc_name: str):
        self._ensure_ros_node()
        if not self._nav_client:
            logger.warning("[TASK] NAV skipped — Nav2 not available")
            return

        loc = self._resolve_location(loc_name)
        if not loc:
            logger.warning(f"[TASK] Unknown location: {loc_name}")
            await self._speak(f"I don't know where {loc_name} is")
            return

        await self._speak(f"navigating to {loc_name}")
        yaw = math.radians(loc.get("yaw_deg", 0.0))

        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._ros_node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc["x"])
        goal.pose.pose.position.y = float(loc["y"])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        success = await asyncio.get_event_loop().run_in_executor(
            None, self._nav_blocking, goal, loc_name
        )
        if not success and not self._cancelled:
            await self._speak(f"Navigation to {loc_name} failed")

    def _nav_blocking(self, goal, loc_name: str) -> bool:
        """Send Nav2 goal and block until complete. Runs in a thread pool."""
        import rclpy
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            logger.error("[TASK] Nav2 action server not available")
            return False

        goal_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._ros_node, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            logger.error(f"[TASK] Nav goal to {loc_name} rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._ros_node, result_future, timeout_sec=120.0
        )
        result = result_future.result()
        success = bool(result and result.status == 4)
        logger.info(f"[TASK] Nav to {loc_name}: {'OK' if success else 'FAILED'}")
        return success

    # ------------------------------------------------------------------
    # Docking service
    # ------------------------------------------------------------------

    async def _step_dock(self):
        self._ensure_ros_node()
        if not self._dock_client:
            logger.warning("[TASK] Dock skipped — service not available")
            return
        await asyncio.get_event_loop().run_in_executor(
            None, self._service_blocking, self._dock_client, "start_docking"
        )

    def _service_blocking(self, client, name: str) -> bool:
        import rclpy
        from std_srvs.srv import Empty
        if not client.wait_for_service(timeout_sec=5.0):
            logger.error(f"[TASK] {name} service not available")
            return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._ros_node, future, timeout_sec=10.0)
        return True

    def shutdown(self):
        try:
            if self._ros_node:
                self._ros_node.destroy_node()
        except Exception:
            pass
