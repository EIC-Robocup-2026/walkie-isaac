# -*- coding: utf-8 -*-

"""ROS 2 bridge and Action Graph integration module for Walkie-Isaac."""

import socket
import subprocess
import time
from typing import Any, Dict, Optional
from walkie_isaac.robot import OmniBaseController


def ensure_zenoh_router(port: int = 7447) -> bool:
    """Ensure a Zenoh router (rmw_zenohd) daemon is active, auto-starting if needed.

    Args:
        port: Zenoh router TCP port to check (default: 7447).

    Returns:
        True if the router is reachable, False otherwise.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        if s.connect_ex(("127.0.0.1", port)) == 0:
            print(f"[Walkie-Isaac:Zenoh] Connected to active Zenoh router on port {port}.")
            return True

    print(f"[Walkie-Isaac:Zenoh] Zenoh router not detected. Auto-starting 'rmw_zenohd' daemon...")
    try:
        subprocess.Popen(
            ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        start = time.time()
        while time.time() - start < 4.0:
            time.sleep(0.2)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    print(f"[Walkie-Isaac:Zenoh] Zenoh router successfully started on port {port}.")
                    return True
        print(f"[Walkie-Isaac:Zenoh] Warning: Timed out waiting for Zenoh router on port {port}.")
        return False
    except Exception as e:
        print(f"[Walkie-Isaac:Zenoh] Warning: Could not auto-start rmw_zenohd: {e}")
        return False


class CmdVelSubscriber:
    """ROS 2 Twist subscriber with watchdog timeout for teleoperation control."""

    def __init__(self, topic_name: str = "/cmd_vel", timeout_sec: float = 0.5):
        """Initialize the ROS 2 node and subscriber.

        Args:
            topic_name: Topic to subscribe for Twist velocity commands (default: /cmd_vel).
            timeout_sec: Timeout after which commands decay to zero if no message received (default: 0.5s).
        """
        import rclpy
        from geometry_msgs.msg import Twist

        if not rclpy.ok():
            rclpy.init()

        self._rclpy = rclpy
        self.node = rclpy.create_node("isaac_cmd_vel_driver")
        self.timeout_sec = timeout_sec

        self.vx: float = 0.0
        self.vy: float = 0.0
        self.wz: float = 0.0
        self.last_msg_time: float = 0.0

        def _twist_callback(msg: Twist):
            self.vx = float(msg.linear.x)
            self.vy = float(msg.linear.y)
            self.wz = float(msg.angular.z)
            self.last_msg_time = time.time()

        self.sub = self.node.create_subscription(
            Twist,
            topic_name,
            _twist_callback,
            10,
        )
        print(f"[Walkie-Isaac:ROS2] Subscribed to {topic_name} (watchdog timeout: {timeout_sec}s)")

    def update(self, base_controller: OmniBaseController) -> None:
        """Spin ROS 2 node non-blockingly and apply velocity to base controller.

        Args:
            base_controller: OmniBaseController instance to apply joint velocities.
        """
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

        now = time.time()
        if (now - self.last_msg_time) < self.timeout_sec:
            base_controller.apply_velocity(self.vx, self.vy, self.wz)
        else:
            base_controller.stop()

    def destroy(self) -> None:
        """Destroy the ROS 2 node."""
        if hasattr(self, "node") and self.node:
            self.node.destroy_node()


def setup_ros2_action_graph(
    config: Dict[str, Any],
    sensor_prim_path: str,
) -> str:
    """Programmatically construct the OmniGraph ROS 2 Action Graph at runtime.

    Wires:
      • OnPlaybackTick + IsaacReadSimulationTime -> ROS2PublishClock
      • OnPlaybackTick + ROS2Context             -> ROS2SubscribeTwist
      • OnPlaybackTick + IsaacCreateRenderProduct-> ROS2RtxLidarHelper

    Args:
        config: Parsed simulation configuration dictionary.
        sensor_prim_path: USD path to the RTX LiDAR sensor prim.

    Returns:
        Graph prim path (e.g. '/ROS2_ActionGraph').
    """
    import omni.graph.core as og

    ros2_cfg = config.get("ros2", {})
    domain_id = int(ros2_cfg.get("domain_id", 0))
    clock_topic = ros2_cfg.get("clock_topic", "/clock")
    cmd_vel_topic = ros2_cfg.get("cmd_vel_topic", "/cmd_vel")

    sensors_cfg = config.get("sensors", {})
    lidar_cfg = sensors_cfg.get("lidar", {})
    lidar_frame_id = lidar_cfg.get("frame_id", "front_lidar_link")
    lidar_topic = lidar_cfg.get("topic_name", "/scan")
    lidar_type = lidar_cfg.get("type", "laser_scan")

    graph_path = "/ROS2_ActionGraph"
    print(f"[Walkie-Isaac:ROS2] Building Action Graph at {graph_path}...")
    keys = og.Controller.Keys

    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("RtxLidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
    ]

    values = [
        ("ROS2Context.inputs:domain_id", domain_id),
        ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ("PublishClock.inputs:topicName", clock_topic),
        ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
        ("CreateRenderProduct.inputs:cameraPrim", [sensor_prim_path]),
        ("CreateRenderProduct.inputs:width", 1),
        ("CreateRenderProduct.inputs:height", 1),
        ("RtxLidarHelper.inputs:topicName", lidar_topic),
        ("RtxLidarHelper.inputs:type", lidar_type),
        ("RtxLidarHelper.inputs:frameId", lidar_frame_id),
        ("RtxLidarHelper.inputs:resetSimulationTimeOnStop", True),
    ]

    connections = [
        # Clock
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("ROS2Context.outputs:context", "PublishClock.inputs:context"),

        # Twist
        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),

        # LiDAR Helper
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "RtxLidarHelper.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "RtxLidarHelper.inputs:renderProductPath"),
        ("ROS2Context.outputs:context", "RtxLidarHelper.inputs:context"),
    ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.SET_VALUES: values,
            keys.CONNECT: connections,
        },
    )

    print("[Walkie-Isaac:ROS2] Action Graph successfully built!")
    print(f"  • Clock:   {clock_topic} (rosgraph_msgs/msg/Clock)")
    print(f"  • Cmd Vel: {cmd_vel_topic} (geometry_msgs/msg/Twist)")
    print(f"  • LiDAR:   {lidar_topic} (sensor_msgs/msg/LaserScan, frame: {lidar_frame_id})")

    return graph_path
