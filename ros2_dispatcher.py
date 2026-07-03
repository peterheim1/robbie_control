"""ROS 2 topic publisher for voice command dispatch."""

import asyncio
import dataclasses
import json
import logging
import math
import threading
from datetime import datetime

logger = logging.getLogger(__name__)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float64MultiArray, Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray

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

    Joint control uses the JointTrajectoryController topic interface
    (/<ctrl>/joint_trajectory) — simpler than action clients, fire-and-forget.
    """

    _JOINT_CONTROLLER_MAP: dict[str, str] = {
        'head_yaw_joint':          'head_controller',
        'head_roll':               'head_controller',
        'head_pitch':              'head_controller',
        'right_yaw_joint':         'right_arm_controller',
        'right_lift_joint':        'right_arm_controller',
        'right_rotate_joint':      'right_arm_controller',
        'right_elbow_joint':       'right_arm_controller',
        'right_wrist_yaw_joint':   'right_arm_controller',
        'right_wrist_pitch_joint': 'right_arm_controller',
        'left_yaw_joint':          'left_arm_controller',
        'left_lift_joint':         'left_arm_controller',
        'left_rotate_joint':       'left_arm_controller',
        'left_elbow_joint':        'left_arm_controller',
        'left_wrist_yaw_joint':    'left_arm_controller',
        'left_wrist_pitch_joint':  'left_arm_controller',
    }
    _CONTROLLER_JOINTS: dict[str, list[str]] = {
        'head_controller':      ['head_yaw_joint', 'head_roll', 'head_pitch'],
        'right_arm_controller': ['right_yaw_joint', 'right_lift_joint', 'right_rotate_joint',
                                 'right_elbow_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint'],
        'left_arm_controller':  ['left_yaw_joint', 'left_lift_joint', 'left_rotate_joint',
                                 'left_elbow_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint'],
    }
    # (right_joint, left_joint, sign) — sign flips the value when mirroring
    _MIRROR_MAP: list[tuple[str, str, int]] = [
        ('right_yaw_joint',         'left_yaw_joint',          -1),
        ('right_lift_joint',        'left_lift_joint',           1),
        ('right_rotate_joint',      'left_rotate_joint',        -1),
        ('right_elbow_joint',       'left_elbow_joint',          1),
        ('right_wrist_yaw_joint',   'left_wrist_yaw_joint',     -1),
        ('right_wrist_pitch_joint', 'left_wrist_pitch_joint',    1),
    ]

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

        # Camera — latest JPEG frame from OAK-D compressed topic (updated from spin thread)
        self._latest_camera_frame: bytes | None = None
        self._sub_camera = self._node.create_subscription(
            CompressedImage, "/oak/rgb/image_raw/compressed",
            self._on_camera_msg, 1)

        # Cached state from subscribers
        self._battery_percentage: float | None = None
        self._battery_voltage: float | None = None
        self._is_docked: bool | None = None
        self._is_charging: bool | None = None
        self._diagnostic_errors: list[str] = []
        self._all_diagnostics: dict[str, dict] = {}  # keyed by name, merged across publishers
        self._joint_positions: dict[str, float] = {}  # joint_name → radians

        self._sub_battery = self._node.create_subscription(
            Float32, "/battery_voltage", self._on_battery_voltage, 10)
        self._sub_charge = self._node.create_subscription(
            Bool, "/charge_state", self._on_charge_state, 10)
        self._sub_diagnostics = self._node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self._sub_joint_states = self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10)

        # Torque enable publisher for thor_joint
        self._pub_torque = self._node.create_publisher(Bool, '/thor_joint/torque_enable', 10)

        # Dynamic publishers and service clients created on demand
        self._dynamic_pubs: dict = {}
        self._service_clients: dict = {}

        # Spin in background thread
        self._spinning = True
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True, name="ros2_spin")
        self._spin_thread.start()

    def _spin(self):
        import time
        while self._spinning:
            try:
                rclpy.spin(self._node)
            except Exception as e:
                if self._spinning:
                    logger.warning(f"ROS2 spin error: {e}, restarting in 1s")
                    time.sleep(1)

    def set_speak_callback(self, callback, loop: asyncio.AbstractEventLoop):
        """Register the async TTS callback invoked when /voice/speak is received.

        Args:
            callback: async coroutine function that accepts a text string.
            loop:     the running asyncio event loop from the voice server.
        """
        self._speak_callback = callback
        self._speak_loop = loop

    def _on_camera_msg(self, msg: CompressedImage):
        """Cache compressed JPEG frame (called from rclpy spin thread)."""
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

    # Diagnostic items to silently ignore (name substring match, lower-case)
    _DIAG_IGNORE = ("wdog", "watchdog", "high jitter", "high_jitter")

    def _on_diagnostics(self, msg: DiagnosticArray):
        """Cache diagnostic statuses from /diagnostics."""
        errors = []
        for status in msg.status:
            name_lc = status.name.lower()
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            self._all_diagnostics[status.name] = {
                "name": status.name,
                "level": level,
                "message": status.message,
                "hardware_id": status.hardware_id,
            }
            if any(ig in name_lc for ig in self._DIAG_IGNORE):
                continue
            if level >= 1:
                label = "ERROR" if level >= 2 else "WARN"
                errors.append(f"{status.name}: {status.message} [{label}]")
        self._diagnostic_errors = errors

    def get_system_errors(self) -> list[str]:
        """Return cached diagnostic errors (wdog and high jitter excluded)."""
        return list(self._diagnostic_errors)

    def get_all_diagnostics(self) -> list[dict]:
        """Return all cached diagnostic statuses for the web UI."""
        return list(self._all_diagnostics.values())

    def _on_battery_voltage(self, msg: Float32):
        self._battery_voltage = float(msg.data)

    def get_battery_voltage(self) -> float | None:
        """Return cached battery voltage in volts, or None if unknown."""
        return self._battery_voltage

    def _on_charge_state(self, msg: Bool):
        self._is_charging = bool(msg.data)

    def get_charge_state(self) -> bool | None:
        """Return True if the robot is currently charging, or None if unknown."""
        return self._is_charging

    def publish_speak_done(self, text: str):
        """Notify Argus that a /voice/speak request has finished playing.

        Argus's Speak BT node blocks on this (via a blackboard sequence
        counter) before letting the next node in the sequence — e.g. Undock,
        NavigateTo — run, so the robot doesn't start moving mid-sentence.
        """
        pub = self._get_dynamic_pub(String, "/voice/speak_done")
        msg = String()
        msg.data = text
        pub.publish(msg)

    def navigate_to(self, x: float, y: float, yaw_deg: float = 0.0, location: str = ""):
        """Send a goto_location command to Argus via /bt_command (fire-and-forget).

        `location` is included so Argus's cmd_cb doesn't clobber the
        target_location set moments earlier by the /voice/intent message
        (see intent_cb) with an empty string.
        """
        pub = self._get_dynamic_pub(String, "/bt_command")
        msg = String()
        msg.data = json.dumps({
            "cmd": "goto_location",
            "location": location,
            "pose": {"x": float(x), "y": float(y), "yaw_deg": float(yaw_deg)},
        })
        pub.publish(msg)

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

    def slam_serialize_map(self, filename: str = '/home/pi/mar_19c_26') -> bool:
        """Save the SLAM pose graph to filename (blocking). Returns True on success."""
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            logger.error("slam_toolbox srv not importable")
            return False
        client = self._get_service_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5.0):
            logger.warning("Service /slam_toolbox/serialize_map not available")
            return False
        req = SerializePoseGraph.Request()
        req.filename = filename
        try:
            resp = client.call(req)
            return bool(resp.result)
        except Exception as e:
            logger.error(f"slam_serialize_map failed: {e}")
            return False

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

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if not math.isnan(pos):
                self._joint_positions[name] = pos

    def get_joint_states(self) -> dict[str, float]:
        """Return current joint positions in degrees, keyed by joint name."""
        return {name: math.degrees(rad) for name, rad in self._joint_positions.items()}

    def send_joint_position(self, joint_name: str, deg: float,
                            move_time_sec: float = 0.5) -> bool:
        """Move one joint to deg°; hold all other joints in that controller at current positions."""
        ctrl = self._JOINT_CONTROLLER_MAP.get(joint_name)
        if not ctrl:
            logger.warning(f"send_joint_position: unknown joint {joint_name!r}")
            return False
        joints = self._CONTROLLER_JOINTS[ctrl]
        msg = JointTrajectory()
        msg.joint_names = joints
        pt = JointTrajectoryPoint()
        for j in joints:
            if j == joint_name:
                pt.positions.append(math.radians(deg))
            else:
                pt.positions.append(self._joint_positions.get(j, 0.0))
        sec = int(move_time_sec)
        pt.time_from_start = Duration(sec=sec,
                                      nanosec=int((move_time_sec - sec) * 1_000_000_000))
        msg.points = [pt]
        pub = self._get_dynamic_pub(JointTrajectory, f'/{ctrl}/joint_trajectory')
        pub.publish(msg)
        return True

    def send_all_to_zero(self, move_time_sec: float = 1.0):
        """Send all three controllers to the zero position."""
        for ctrl, joints in self._CONTROLLER_JOINTS.items():
            msg = JointTrajectory()
            msg.joint_names = joints
            pt = JointTrajectoryPoint()
            pt.positions = [0.0] * len(joints)
            sec = int(move_time_sec)
            pt.time_from_start = Duration(sec=sec,
                                          nanosec=int((move_time_sec - sec) * 1_000_000_000))
            msg.points = [pt]
            pub = self._get_dynamic_pub(JointTrajectory, f'/{ctrl}/joint_trajectory')
            pub.publish(msg)

    def send_all_joints(self, joint_dict: dict[str, float], move_time_sec: float = 1.0):
        """Send a {joint_name: degrees} dict to their controllers.

        Only controllers with at least one joint in joint_dict are messaged.
        Joints in that controller but absent from joint_dict hold their current position.
        """
        ctrl_targets: dict[str, dict[str, float]] = {}
        for name, deg in joint_dict.items():
            ctrl = self._JOINT_CONTROLLER_MAP.get(name)
            if ctrl:
                ctrl_targets.setdefault(ctrl, {})[name] = deg
        for ctrl, targets in ctrl_targets.items():
            joints = self._CONTROLLER_JOINTS[ctrl]
            msg = JointTrajectory()
            msg.joint_names = joints
            pt = JointTrajectoryPoint()
            for j in joints:
                if j in targets:
                    pt.positions.append(math.radians(targets[j]))
                else:
                    pt.positions.append(self._joint_positions.get(j, 0.0))
            sec = int(move_time_sec)
            pt.time_from_start = Duration(sec=sec,
                                          nanosec=int((move_time_sec - sec) * 1_000_000_000))
            msg.points = [pt]
            pub = self._get_dynamic_pub(JointTrajectory, f'/{ctrl}/joint_trajectory')
            pub.publish(msg)

    def speak(self, text: str):
        """Publish text to the TTS topic."""
        msg = String()
        msg.data = text
        self._pub_tts_text.publish(msg)

    def set_servo_torque(self, enable: bool):
        """Enable or disable torque on all arm/head servos via thor_joint."""
        msg = Bool()
        msg.data = enable
        self._pub_torque.publish(msg)

    def mirror_arm(self, direction: str, move_time_sec: float = 1.0):
        """Mirror one arm onto the other.

        direction: 'right_to_left' copies right arm → left arm (with axis sign flip)
                   'left_to_right' copies left arm → right arm
        """
        target: dict[str, float] = {}
        for r_joint, l_joint, sign in self._MIRROR_MAP:
            if direction == 'right_to_left':
                src_rad = self._joint_positions.get(r_joint, 0.0)
                target[l_joint] = math.degrees(src_rad) * sign
            else:
                src_rad = self._joint_positions.get(l_joint, 0.0)
                target[r_joint] = math.degrees(src_rad) * sign
        self.send_all_joints(target, move_time_sec)

    def shutdown(self):
        """Shutdown rclpy and stop the spin thread."""
        self._spinning = False
        try:
            self._node.destroy_node()
        except Exception:
            pass
        self._spin_thread.join(timeout=3.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass
