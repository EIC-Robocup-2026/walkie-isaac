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
    sensors: Dict[str, Any],
) -> str:
    """Programmatically construct the OmniGraph ROS 2 Action Graph for all sensors.

    Wires:
      • Clock: OnPlaybackTick + IsaacReadSimulationTime -> ROS2PublishClock
      • CmdVel: OnPlaybackTick + ROS2Context -> ROS2SubscribeTwist
      • 2D LiDAR: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2RtxLidarHelper (laser_scan)
      • 3D LiDAR: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2RtxLidarHelper (point_cloud)
      • IMU: OnPlaybackTick + IsaacReadIMU -> ROS2PublishImu
      • Head Camera: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2CameraHelper (RGB, Depth, PointCloud)
      • Bottom Camera: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2CameraHelper (RGB, Depth, PointCloud)

    Args:
        config: Parsed simulation configuration dictionary.
        sensors: Dictionary of configured sensor metadata from setup_sensors().

    Returns:
        Graph prim path (e.g. '/ROS2_ActionGraph').
    """
    import omni.graph.core as og

    ros2_cfg = config.get("ros2", {})
    domain_id = int(ros2_cfg.get("domain_id", 0))
    clock_topic = ros2_cfg.get("clock_topic", "/clock")
    cmd_vel_topic = ros2_cfg.get("cmd_vel_topic", "/cmd_vel")

    graph_path = "/ROS2_ActionGraph"
    print(f"[Walkie-Isaac:ROS2] Building Action Graph at {graph_path}...")
    keys = og.Controller.Keys

    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ]

    values = [
        ("ROS2Context.inputs:domain_id", domain_id),
        ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ("PublishClock.inputs:topicName", clock_topic),
        ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
    ]

    connections = [
        # Clock
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("ROS2Context.outputs:context", "PublishClock.inputs:context"),

        # Twist
        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
    ]

    # --------------------------------------------------------------------------
    # 2D LiDAR (Center / Merged planar scan)
    # --------------------------------------------------------------------------
    if "lidar_2d" in sensors:
        l2d = sensors["lidar_2d"]
        nodes.extend([
            ("RP_Lidar2D", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Helper_Lidar2D", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ])
        values.extend([
            ("RP_Lidar2D.inputs:cameraPrim", [l2d["sensor_prim_path"]]),
            ("RP_Lidar2D.inputs:width", 1),
            ("RP_Lidar2D.inputs:height", 1),
            ("Helper_Lidar2D.inputs:type", "laser_scan"),
            ("Helper_Lidar2D.inputs:topicName", l2d["topic_name"]),
            ("Helper_Lidar2D.inputs:frameId", l2d["frame_id"]),
            ("Helper_Lidar2D.inputs:resetSimulationTimeOnStop", True),
        ])
        connections.extend([
            ("OnPlaybackTick.outputs:tick", "RP_Lidar2D.inputs:execIn"),
            ("RP_Lidar2D.outputs:execOut", "Helper_Lidar2D.inputs:execIn"),
            ("RP_Lidar2D.outputs:renderProductPath", "Helper_Lidar2D.inputs:renderProductPath"),
            ("ROS2Context.outputs:context", "Helper_Lidar2D.inputs:context"),
        ])

    # --------------------------------------------------------------------------
    # 3D LiDAR (Unitree 4D multi-ring point cloud)
    # --------------------------------------------------------------------------
    if "lidar_3d" in sensors:
        l3d = sensors["lidar_3d"]
        nodes.extend([
            ("RP_Lidar3D", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("Helper_Lidar3D", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ])
        values.extend([
            ("RP_Lidar3D.inputs:cameraPrim", [l3d["sensor_prim_path"]]),
            ("RP_Lidar3D.inputs:width", 1),
            ("RP_Lidar3D.inputs:height", 1),
            ("Helper_Lidar3D.inputs:type", "point_cloud"),
            ("Helper_Lidar3D.inputs:topicName", l3d["topic_name"]),
            ("Helper_Lidar3D.inputs:frameId", l3d["frame_id"]),
            ("Helper_Lidar3D.inputs:resetSimulationTimeOnStop", True),
        ])
        connections.extend([
            ("OnPlaybackTick.outputs:tick", "RP_Lidar3D.inputs:execIn"),
            ("RP_Lidar3D.outputs:execOut", "Helper_Lidar3D.inputs:execIn"),
            ("RP_Lidar3D.outputs:renderProductPath", "Helper_Lidar3D.inputs:renderProductPath"),
            ("ROS2Context.outputs:context", "Helper_Lidar3D.inputs:context"),
        ])

    # --------------------------------------------------------------------------
    # IMU (Integrated at 3D LiDAR)
    # --------------------------------------------------------------------------
    if "imu" in sensors:
        imu = sensors["imu"]
        nodes.extend([
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ])
        values.extend([
            ("ReadIMU.inputs:imuPrim", [imu["sensor_prim_path"]]),
            ("ReadIMU.inputs:readGravity", True),
            ("PublishIMU.inputs:topicName", imu["topic_name"]),
            ("PublishIMU.inputs:frameId", imu["frame_id"]),
        ])
        connections.extend([
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("ROS2Context.outputs:context", "PublishIMU.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ])

    # --------------------------------------------------------------------------
    # Cameras: Head (ZED 2i) and Bottom (RealSense D415)
    # --------------------------------------------------------------------------
    camera_keys = [("head_camera", "HeadCam"), ("bottom_camera", "BottomCam")]
    for cam_key, prefix in camera_keys:
        if cam_key in sensors:
            cam = sensors[cam_key]
            rp_node = f"RP_{prefix}"
            info_node = f"Info_{prefix}"
            rgb_node = f"RGB_{prefix}"
            depth_node = f"Depth_{prefix}"
            pcl_node = f"Pcl_{prefix}"

            nodes.extend([
                (rp_node, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (info_node, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                (rgb_node, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (depth_node, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (pcl_node, "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ])

            values.extend([
                (f"{rp_node}.inputs:cameraPrim", [cam["camera_prim_path"]]),
                (f"{rp_node}.inputs:width", cam["width"]),
                (f"{rp_node}.inputs:height", cam["height"]),
                # Camera Info
                (f"{info_node}.inputs:topicName", cam["rgb_info_topic"]),
                (f"{info_node}.inputs:frameId", cam["frame_id"]),
                (f"{info_node}.inputs:resetSimulationTimeOnStop", True),
                # RGB
                (f"{rgb_node}.inputs:type", "rgb"),
                (f"{rgb_node}.inputs:topicName", cam["rgb_topic"]),
                (f"{rgb_node}.inputs:frameId", cam["frame_id"]),
                (f"{rgb_node}.inputs:resetSimulationTimeOnStop", True),
                # Depth
                (f"{depth_node}.inputs:type", "depth"),
                (f"{depth_node}.inputs:topicName", cam["depth_topic"]),
                (f"{depth_node}.inputs:frameId", cam["frame_id"]),
                (f"{depth_node}.inputs:resetSimulationTimeOnStop", True),
                # Depth Point Cloud
                (f"{pcl_node}.inputs:type", "depth_pcl"),
                (f"{pcl_node}.inputs:topicName", cam["depth_pcl_topic"]),
                (f"{pcl_node}.inputs:frameId", cam["frame_id"]),
                (f"{pcl_node}.inputs:resetSimulationTimeOnStop", True),
            ])

            connections.extend([
                ("OnPlaybackTick.outputs:tick", f"{rp_node}.inputs:execIn"),
                # Info
                (f"{rp_node}.outputs:execOut", f"{info_node}.inputs:execIn"),
                (f"{rp_node}.outputs:renderProductPath", f"{info_node}.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", f"{info_node}.inputs:context"),
                # RGB
                (f"{rp_node}.outputs:execOut", f"{rgb_node}.inputs:execIn"),
                (f"{rp_node}.outputs:renderProductPath", f"{rgb_node}.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", f"{rgb_node}.inputs:context"),
                # Depth
                (f"{rp_node}.outputs:execOut", f"{depth_node}.inputs:execIn"),
                (f"{rp_node}.outputs:renderProductPath", f"{depth_node}.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", f"{depth_node}.inputs:context"),
                # Depth Point Cloud
                (f"{rp_node}.outputs:execOut", f"{pcl_node}.inputs:execIn"),
                (f"{rp_node}.outputs:renderProductPath", f"{pcl_node}.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", f"{pcl_node}.inputs:context"),
            ])

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.SET_VALUES: values,
            keys.CONNECT: connections,
        },
    )

    print("[Walkie-Isaac:ROS2] Multi-Sensor Action Graph successfully built!")
    print(f"  • Clock:       {clock_topic} (rosgraph_msgs/msg/Clock)")
    print(f"  • Cmd Vel:     {cmd_vel_topic} (geometry_msgs/msg/Twist)")
    if "lidar_2d" in sensors:
        print(f"  • 2D LiDAR:    {sensors['lidar_2d']['topic_name']} (LaserScan, frame: {sensors['lidar_2d']['frame_id']})")
    if "lidar_3d" in sensors:
        print(f"  • 3D LiDAR:    {sensors['lidar_3d']['topic_name']} (PointCloud2, frame: {sensors['lidar_3d']['frame_id']})")
    if "imu" in sensors:
        print(f"  • IMU:         {sensors['imu']['topic_name']} (Imu, frame: {sensors['imu']['frame_id']})")
    if "head_camera" in sensors:
        h = sensors["head_camera"]
        print(f"  • Head RGB:    {h['rgb_topic']} (Image, frame: {h['frame_id']})")
        print(f"  • Head Depth:  {h['depth_topic']} (Image, frame: {h['frame_id']})")
        print(f"  • Head PCL:    {h['depth_pcl_topic']} (PointCloud2, frame: {h['frame_id']})")
    if "bottom_camera" in sensors:
        b = sensors["bottom_camera"]
        print(f"  • Bottom RGB:  {b['rgb_topic']} (Image, frame: {b['frame_id']})")
        print(f"  • Bottom Depth:{b['depth_topic']} (Image, frame: {b['frame_id']})")
        print(f"  • Bottom PCL:  {b['depth_pcl_topic']} (PointCloud2, frame: {b['frame_id']})")

    return graph_path
