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
    robot_prim_path: str = "/World/walkie_bot",
) -> str:
    """Programmatically construct the OmniGraph ROS 2 Action Graph for all sensors and TF.

    Wires:
      • Clock: OnPlaybackTick + IsaacReadSimulationTime -> ROS2PublishClock
      • CmdVel: OnPlaybackTick + ROS2Context -> ROS2SubscribeTwist
      • TF Tree: OnPlaybackTick + IsaacComputeTransformTree -> ROS2PublishTransformTree
      • 2D LiDAR: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2RtxLidarHelper (laser_scan)
      • 3D LiDAR: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2RtxLidarHelper (point_cloud)
      • IMU: OnPlaybackTick + IsaacReadIMU -> ROS2PublishImu
      • Head Camera: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2CameraHelper (RGB, Depth, PointCloud)
      • Bottom Camera: OnPlaybackTick + IsaacCreateRenderProduct -> ROS2CameraHelper (RGB, Depth, PointCloud)

    Args:
        config: Parsed simulation configuration dictionary.
        sensors: Dictionary of configured sensor metadata from setup_sensors().
        robot_prim_path: USD root prim path for the robot (default: /World/walkie_bot).

    Returns:
        Graph prim path (e.g. '/ROS2_ActionGraph').
    """
    import omni.graph.core as og

    sim_cfg = config.get("simulation", {})
    physics_dt = float(sim_cfg.get("physics_dt", 1.0 / 60.0))
    sim_hz = (1.0 / physics_dt) if physics_dt > 0 else 60.0

    ros2_cfg = config.get("ros2", {})
    domain_id = int(ros2_cfg.get("domain_id", 0))
    clock_topic = ros2_cfg.get("clock_topic", "/clock")
    cmd_vel_topic = ros2_cfg.get("cmd_vel_topic", "/cmd_vel")

    graph_path = "/ROS2_ActionGraph"
    print(f"[Walkie-Isaac:ROS2] Building Action Graph at {graph_path} (Sim Rate: {sim_hz:.1f} Hz)...")
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
    # Joint States Publisher
    # --------------------------------------------------------------------------
    joint_states_cfg = ros2_cfg.get("joint_states", {})
    if joint_states_cfg.get("enabled", True):
        from pxr import Sdf
        js_topic = joint_states_cfg.get("topic_name", "/joint_states")
        js_target_prim = joint_states_cfg.get(
            "target_prim",
            f"{robot_prim_path}/Geometry/base_footprint/base_link",
        )
        js_hz = float(joint_states_cfg.get("publish_rate_hz", sim_hz))
        js_step = max(1, round(sim_hz / js_hz)) if js_hz > 0 else 1

        nodes.append(("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"))
        values.extend([
            ("PublishJointState.inputs:targetPrim", [Sdf.Path(js_target_prim)]),
            ("PublishJointState.inputs:topicName", js_topic),
        ])

        if js_step > 1:
            nodes.append(("Gate_JointState", "isaacsim.core.nodes.IsaacSimulationGate"))
            values.append(("Gate_JointState.inputs:step", int(js_step)))
            connections.extend([
                ("OnPlaybackTick.outputs:tick", "Gate_JointState.inputs:execIn"),
                ("Gate_JointState.outputs:execOut", "PublishJointState.inputs:execIn"),
            ])
        else:
            connections.append(("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"))

        connections.extend([
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
        ])

    # --------------------------------------------------------------------------
    # Joint Command Subscriber & Articulation Controller (Arm & Lift control)
    # --------------------------------------------------------------------------
    joint_cmd_cfg = ros2_cfg.get("joint_commands", {})
    if joint_cmd_cfg.get("enabled", True):
        from pxr import Sdf
        cmd_topic = joint_cmd_cfg.get("topic_name", "/joint_commands")
        art_target_prim = joint_cmd_cfg.get(
            "target_prim",
            f"{robot_prim_path}/Geometry/base_footprint/base_link",
        )
        nodes.extend([
            ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ])
        values.extend([
            ("SubscribeJointCommand.inputs:topicName", cmd_topic),
            ("ArticulationController.inputs:targetPrim", [Sdf.Path(art_target_prim)]),
        ])
        connections.extend([
            ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
            ("ROS2Context.outputs:context", "SubscribeJointCommand.inputs:context"),
            ("SubscribeJointCommand.outputs:execOut", "ArticulationController.inputs:execIn"),
            ("SubscribeJointCommand.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointCommand.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointCommand.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointCommand.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ])

    # --------------------------------------------------------------------------
    # Transform Tree (TF & TF_Static) Publishers
    # --------------------------------------------------------------------------
    tf_cfg = ros2_cfg.get("tf", {})
    if tf_cfg.get("enabled", False):
        from pxr import Sdf
        tf_topic = tf_cfg.get("topic_name", "/tf")
        tf_static_topic = tf_cfg.get("static_topic_name", "/tf_static")
        parent_prim_str = tf_cfg.get(
            "parent_prim",
            f"{robot_prim_path}/Geometry/base_footprint",
        )

        # Static sensor frames (fixed relative to base footprint/chassis).
        # Published on /tf_static with staticPublisher=True (latched transient_local QoS).
        # In ROS 2 tf2, static transforms are valid for ALL timestamps, completely
        # eliminating extrapolation-into-the-future errors in RViz.
        static_target_paths = [
            f"{robot_prim_path}/Geometry/base_footprint/base_link",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/merged_lidar_link",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/front_lidar_link",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/back_lidar_link",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/unitree4d_l2_imu_initial",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/unitree4d_l2_imu_initial/unitree4d_l2_imu",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/bottom_bottom_screw_frame/bottom_link/bottom_depth_frame/bottom_color_frame/bottom_color_optical_frame",
        ]

        # Dynamic target frames (articulated bodies that move during simulation).
        # Articulation root base_link automatically expands to all actuated joints
        # (wheels, torso lift, head servo, arms).
        dynamic_target_paths = [
            f"{robot_prim_path}/Geometry/base_footprint/base_link",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/lift_link/head_servo_link/zed_head_camera_link/zed_head_camera_center/zed_head_left_camera_frame/zed_head_left_camera_frame_optical",
        ]

        nodes.extend([
            # Dynamic TF
            ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            # Static TF
            ("ComputeTFStatic", "isaacsim.core.nodes.IsaacComputeTransformTree"),
            ("PublishTFStatic", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ])
        values.extend([
            # Dynamic TF
            ("ComputeTF.inputs:parentPrim", [Sdf.Path(parent_prim_str)]),
            ("ComputeTF.inputs:targetPrims", [Sdf.Path(p) for p in dynamic_target_paths]),
            ("PublishTF.inputs:topicName", tf_topic),
            ("PublishTF.inputs:staticPublisher", False),
            # Static TF
            ("ComputeTFStatic.inputs:parentPrim", [Sdf.Path(parent_prim_str)]),
            ("ComputeTFStatic.inputs:targetPrims", [Sdf.Path(p) for p in static_target_paths]),
            ("PublishTFStatic.inputs:topicName", tf_static_topic),
            ("PublishTFStatic.inputs:staticPublisher", True),
        ])
        connections.extend([
            # Dynamic TF
            ("OnPlaybackTick.outputs:tick", "ComputeTF.inputs:execIn"),
            ("ComputeTF.outputs:execOut", "PublishTF.inputs:execIn"),
            ("ComputeTF.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
            ("ComputeTF.outputs:childFrames", "PublishTF.inputs:childFrames"),
            ("ComputeTF.outputs:translations", "PublishTF.inputs:translations"),
            ("ComputeTF.outputs:orientations", "PublishTF.inputs:orientations"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ("ROS2Context.outputs:context", "PublishTF.inputs:context"),
            # Static TF
            ("OnPlaybackTick.outputs:tick", "ComputeTFStatic.inputs:execIn"),
            ("ComputeTFStatic.outputs:execOut", "PublishTFStatic.inputs:execIn"),
            ("ComputeTFStatic.outputs:parentFrames", "PublishTFStatic.inputs:parentFrames"),
            ("ComputeTFStatic.outputs:childFrames", "PublishTFStatic.inputs:childFrames"),
            ("ComputeTFStatic.outputs:translations", "PublishTFStatic.inputs:translations"),
            ("ComputeTFStatic.outputs:orientations", "PublishTFStatic.inputs:orientations"),
            ("ReadSimTime.outputs:simulationTime", "PublishTFStatic.inputs:timeStamp"),
            ("ROS2Context.outputs:context", "PublishTFStatic.inputs:context"),
        ])

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
        imu_hz = float(imu.get("publish_rate_hz", sim_hz))
        imu_step = max(1, round(sim_hz / imu_hz)) if imu_hz > 0 else 1

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

        if imu_step > 1:
            nodes.append(("Gate_IMU", "isaacsim.core.nodes.IsaacSimulationGate"))
            values.append(("Gate_IMU.inputs:step", int(imu_step)))
            connections.extend([
                ("OnPlaybackTick.outputs:tick", "Gate_IMU.inputs:execIn"),
                ("Gate_IMU.outputs:execOut", "ReadIMU.inputs:execIn"),
            ])
        else:
            connections.append(("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"))

        connections.extend([
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
    if joint_states_cfg.get("enabled", True):
        print(f"  • Joint States:{js_topic} (sensor_msgs/msg/JointState, rate: {js_hz:.1f} Hz, step: {js_step})")
    if joint_cmd_cfg.get("enabled", True):
        print(f"  • Joint Cmds:  {cmd_topic} (sensor_msgs/msg/JointState -> ArticulationController)")
    if tf_cfg.get("enabled", False):
        print(f"  • TF Dynamic:  {tf_topic} (tf2_msgs/msg/TFMessage, parent: {parent_prim_str})")
        print(f"  • TF Static:   {tf_static_topic} (tf2_msgs/msg/TFMessage, latched)")
    else:
        print("  • TF Broadcast: Disabled (delegated to ROS 2 robot_state_publisher)")
    if "lidar_2d" in sensors:
        l = sensors["lidar_2d"]
        print(f"  • 2D LiDAR:    {l['topic_name']} (LaserScan, frame: {l['frame_id']}, rate: {l.get('publish_rate_hz', 20.0):.1f} Hz)")
    if "lidar_3d" in sensors:
        l = sensors["lidar_3d"]
        print(f"  • 3D LiDAR:    {l['topic_name']} (PointCloud2, frame: {l['frame_id']}, rate: {l.get('publish_rate_hz', 10.0):.1f} Hz)")
    if "imu" in sensors:
        im = sensors["imu"]
        print(f"  • IMU:         {im['topic_name']} (Imu, frame: {im['frame_id']}, rate: {imu_hz:.1f} Hz, step: {imu_step})")
    if "head_camera" in sensors:
        h = sensors["head_camera"]
        rate = h.get("publish_rate_hz", 15.0)
        print(f"  • Head RGB:    {h['rgb_topic']} (Image, frame: {h['frame_id']}, rate: {rate:.1f} Hz)")
        print(f"  • Head Depth:  {h['depth_topic']} (Image, frame: {h['frame_id']}, rate: {rate:.1f} Hz)")
        print(f"  • Head PCL:    {h['depth_pcl_topic']} (PointCloud2, frame: {h['frame_id']}, rate: {rate:.1f} Hz)")
    if "bottom_camera" in sensors:
        b = sensors["bottom_camera"]
        rate = b.get("publish_rate_hz", 15.0)
        print(f"  • Bottom RGB:  {b['rgb_topic']} (Image, frame: {b['frame_id']}, rate: {rate:.1f} Hz)")
        print(f"  • Bottom Depth:{b['depth_topic']} (Image, frame: {b['frame_id']}, rate: {rate:.1f} Hz)")
        print(f"  • Bottom PCL:  {b['depth_pcl_topic']} (PointCloud2, frame: {b['frame_id']}, rate: {rate:.1f} Hz)")

    return graph_path
