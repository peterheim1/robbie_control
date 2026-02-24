"""ROS 2 topic publisher for voice command dispatch."""

import asyncio
import dataclasses
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float64MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from robbie_control.intent_classifier import Intent


def _json_safe(obj):
    """Recursively convert dataclass instances to dicts for JSON serialisation."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return dataclasses.asdict(obj)
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_json_safe(v) for v in obj]
    return obj


class ROS2Dispatcher:
    """Publishes voice intents and commands to ROS 2 topics.

    Runs rclpy in a background daemon thread. Publisher methods are
    thread-safe and can be called from asyncio.

    Subscribes to /voice/speak so external nodes can request TTS output.
    """

    def __init__(self, node_name: str = "robbie_voice", topic_config: dict | None = None):
        topics = topic_config or {}
        self._topic_names = {
            "intent": topics.get("intent", "/voice/intent"),
            "stop": topics.get("stop", "/voice/stop"),
            "tts_text": topics.get("tts_text", "/voice/tts_text"),
            "speak": topics.get("speak", "/voice/speak"),
            "cmd_vel": topics.get("cmd_vel", "/cmd_vel"),
            "drive": topics.get("drive", "/drive"),
            "head_position": topics.get("head_position", "/head/position"),
        }

        rclpy.init()
        self._node = Node(node_name)

        # Publishers
        self._pub_intent = self._node.create_publisher(
            String, self._topic_names["intent"], 10)
        self._pub_stop = self._node.create_publisher(
            Empty, self._topic_names["stop"], 10)
        self._pub_tts_text = self._node.create_publisher(
            String, self._topic_names["tts_text"], 10)
        self._pub_cmd_vel = self._node.create_publisher(
            Twist, self._topic_names["cmd_vel"], 10)
        self._pub_drive = self._node.create_publisher(
            Float64MultiArray, self._topic_names["drive"], 10)
        self._pub_head = self._node.create_publisher(
            JointTrajectory, self._topic_names["head_position"], 10)

        # Subscriber: /voice/speak — external nodes request TTS output
        self._speak_callback = None   # async coroutine to call
        self._speak_loop: asyncio.AbstractEventLoop | None = None
        self._sub_speak = self._node.create_subscription(
            String, self._topic_names["speak"], self._on_speak_msg, 10)

        # Camera — latest compressed JPEG frame (updated from spin thread)
        self._latest_camera_frame: bytes | None = None
        self._sub_camera = self._node.create_subscription(
            CompressedImage, "/oak/rgb/image_raw/compressed",
            self._on_camera_msg, 1)

        # Cached state from subscribers
        self._battery_percentage: float | None = None
        self._battery_voltage: float | None = None
        self._is_docked: bool | None = None

        self._sub_battery = self._node.create_subscription(
            Float32, "/battery_voltage", self._on_battery_voltage, 10)

        # Dynamic publishers and service clients created on demand
        self._dynamic_pubs: dict = {}
        self._service_clients: dict = {}

        # Spin in background thread
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True, name="ros2_spin")
        self._spin_thread.start()

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def set_speak_callback(self, callback, loop: asyncio.AbstractEventLoop):
        """Register the async TTS callback invoked when /voice/speak is received.

        Args:
            callback: async coroutine function that accepts a text string.
            loop:     the running asyncio event loop from the voice server.
        """
        self._speak_callback = callback
        self._speak_loop = loop

    def _on_camera_msg(self, msg: CompressedImage):
        """Cache the latest JPEG frame (called from rclpy spin thread)."""
        self._latest_camera_frame = bytes(msg.data)

    def get_latest_camera_frame(self) -> bytes | None:
        """Return the most recent camera frame as raw JPEG bytes, or None."""
        return self._latest_camera_frame

    def _on_speak_msg(self, msg: String):
        """Called from the rclpy spin thread when /voice/speak is published."""
        text = msg.data.strip()
        if not text or not self._speak_callback or not self._speak_loop:
            return
        asyncio.run_coroutine_threadsafe(
            self._speak_callback(text), self._speak_loop
        )

    def publish_intent(self, intent: Intent):
        """Publish a JSON-encoded intent to /voice/intent and action-specific topics."""
        # JSON intent
        msg = String()
        msg.data = json.dumps({
            "timestamp": datetime.now().isoformat(),
            "utterance": intent.utterance,
            "intent": intent.name,
            "confidence": intent.confidence,
            "params": _json_safe(intent.params),
        })
        self._pub_intent.publish(msg)

        # Publish TTS text if available
        if intent.response_text:
            tts_msg = String()
            tts_msg.data = intent.response_text
            self._pub_tts_text.publish(tts_msg)

    def publish_stop(self):
        """Fast-path emergency stop: publish to multiple topics simultaneously."""
        # /voice/stop
        self._pub_stop.publish(Empty())

        # /cmd_vel - all zeros
        twist = Twist()
        self._pub_cmd_vel.publish(twist)

        # /drive - all zeros [FL, RL, RR, FR]
        drive_msg = Float64MultiArray()
        drive_msg.data = [0.0, 0.0, 0.0, 0.0]
        self._pub_drive.publish(drive_msg)

    def publish_head(self, pan: float, tilt: float):
        """Publish head position to /head_controller/joint_trajectory."""
        msg = JointTrajectory()
        msg.joint_names = ["head_pan_joint", "head_tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(sec=0, nanosec=500_000_000)  # 0.5 s
        msg.points = [pt]
        self._pub_head.publish(msg)

    def _on_battery_voltage(self, msg: Float32):
        self._battery_voltage = float(msg.data)

    def get_battery_voltage(self) -> float | None:
        """Return cached battery voltage in volts, or None if unknown."""
        return self._battery_voltage

    def dispatch_ros_actions(self, ros_actions: list) -> list[str]:
        """Publish each RosAction from commands.txt. Returns list of topics published."""
        published = []
        for action in ros_actions:
            if action.action_type == "none":
                continue
            elif action.action_type == "empty":
                pub = self._get_dynamic_pub(Empty, action.topic)
                pub.publish(Empty())
                published.append(action.topic)
            elif action.action_type == "twist":
                pub = self._get_dynamic_pub(Twist, action.topic)
                pub.publish(Twist())  # all zeros
                published.append(action.topic)
            elif action.action_type == "joint_traj":
                pub = self._get_dynamic_pub(JointTrajectory, action.topic)
                msg = JointTrajectory()
                msg.joint_names = list(action.params.keys())
                pt = JointTrajectoryPoint()
                pt.positions = [float(v) for v in action.params.values()]
                pt.time_from_start = Duration(sec=0, nanosec=500_000_000)
                msg.points = [pt]
                pub.publish(msg)
                published.append(action.topic)
            elif action.action_type == "float64_array":
                pub = self._get_dynamic_pub(Float64MultiArray, action.topic)
                msg = Float64MultiArray()
                values = action.params.get("values", [])
                msg.data = values if isinstance(values, list) else [values]
                pub.publish(msg)
                published.append(action.topic)
            elif action.action_type == "srv_empty":
                from std_srvs.srv import Empty as EmptySrv
                client = self._get_service_client(EmptySrv, action.topic)
                threading.Thread(
                    target=self._call_service,
                    args=(client, EmptySrv.Request(), action.topic),
                    daemon=True,
                ).start()
                published.append(action.topic)
            else:
                logger.warning(f"Unknown ros action type: {action.action_type!r}")
        return published

    def _call_service(self, client, request, service_name: str):
        """Call a ROS 2 service in a daemon thread (fire and forget)."""
        if not client.wait_for_service(timeout_sec=3.0):
            logger.warning(f"Service {service_name} not available")
            return
        try:
            client.call(request)
        except Exception as e:
            logger.error(f"Service call {service_name} failed: {e}")

    def _get_service_client(self, srv_type, service_name: str):
        """Return a service client, creating it on first use."""
        key = (srv_type, service_name)
        if key not in self._service_clients:
            self._service_clients[key] = self._node.create_client(srv_type, service_name)
        return self._service_clients[key]

    def _get_dynamic_pub(self, msg_type, topic: str):
        """Return a publisher for msg_type/topic, creating it on first use."""
        key = (msg_type, topic)
        if key not in self._dynamic_pubs:
            self._dynamic_pubs[key] = self._node.create_publisher(msg_type, topic, 10)
        return self._dynamic_pubs[key]

    def get_battery_percentage(self) -> float | None:
        """Return cached battery percentage, or None if unknown."""
        return self._battery_percentage

    def get_dock_state(self) -> bool | None:
        """Return cached dock state, or None if unknown."""
        return self._is_docked

    def get_published_topics(self) -> list[str]:
        """Return list of topic names this dispatcher publishes to."""
        return list(self._topic_names.values())

    def shutdown(self):
        """Shutdown rclpy and stop the spin thread."""
        try:
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
