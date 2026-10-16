# -*- coding: utf-8 -*-

"""Sensor initialization and configuration module for Walkie-Isaac.

Supports:
  • 2D LiDAR (Center / Merged planar scan, Gazebo compatible)
  • 3D LiDAR (Unitree 4D multi-ring point cloud)
  • IMU (Integrated at the 3D LiDAR sensor)
  • Head Camera (ZED 2i: RGB + Depth + 3D Point Cloud + CameraInfo)
  • Bottom Camera (RealSense D415: RGB + Depth + 3D Point Cloud + CameraInfo)
"""

from typing import Any, Dict
import carb
import omni.usd
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom


def _create_camera_prim(camera_prim_path: str, focal_length: float = 24.0, tick_rate: float = 0.0) -> None:
    """Create a USD Camera prim oriented along standard forward optical viewing direction.

    Args:
        camera_prim_path: USD path where the camera prim will be defined.
        focal_length: Camera focal length in mm (default: 24.0).
        tick_rate: Camera sensor tick rate in Hz (default: 0.0 for every frame).
    """
    stage = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage, camera_prim_path)
    cam.CreateFocalLengthAttr(focal_length)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.29)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))

    if tick_rate > 0:
        cam.GetPrim().CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(float(tick_rate))

    # In ROS optical frame (+X right, +Y down, +Z forward), rotating 180 deg around X
    # aligns USD Camera (-Z view, +Y up, +X right) with the ROS optical frame view.
    xform = UsdGeom.Xformable(cam)
    xform.ClearXformOpOrder()
    orient_op = xform.AddOrientOp()
    orient_op.Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))


def _create_rtx_lidar(sensor_prim_path: str, tick_rate: float = 0.0, scan_rate_hz: float = 0.0) -> None:
    """Create an RTX LiDAR prim at the specified path with configurable tick and scan rates.

    Args:
        sensor_prim_path: USD path for the OmniLidar prim.
        tick_rate: Sensor tick rate in Hz (omni:sensor:tickRate).
        scan_rate_hz: Base rotation rate in Hz for rotary laser scans (omni:sensor:Core:scanRateBaseHz).
    """
    from isaacsim.sensors.experimental.rtx import Lidar

    try:
        if tick_rate > 0:
            Lidar(path=sensor_prim_path, tick_rate=float(tick_rate))
        else:
            Lidar(path=sensor_prim_path)
    except Exception as e:
        carb.log_warn(f"Lidar experimental wrapper returned: {e}. Trying Kit command fallback...")
        omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path=sensor_prim_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(sensor_prim_path)
    if prim and prim.IsValid():
        if tick_rate > 0:
            prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(float(tick_rate))
        if scan_rate_hz > 0:
            scan_attr = prim.GetAttribute("omni:sensor:Core:scanRateBaseHz")
            if scan_attr and scan_attr.IsValid():
                scan_attr.Set(int(scan_rate_hz))


def _create_imu_sensor(imu_prim_path: str, publish_rate_hz: float = 60.0) -> None:
    """Create a PhysX IMU sensor prim at the specified path.

    Args:
        imu_prim_path: USD path for the IMU prim.
        publish_rate_hz: Publishing / sampling frequency in Hz (sets sensorPeriod = 1.0 / hz).
    """
    from isaacsim.sensors.experimental.physics import IMU

    IMU.create(imu_prim_path)
    if publish_rate_hz > 0:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(imu_prim_path)
        if prim and prim.IsValid():
            period_attr = prim.GetAttribute("sensorPeriod")
            if period_attr and period_attr.IsValid():
                period_attr.Set(1.0 / float(publish_rate_hz))


def setup_sensors(config: Dict[str, Any], robot_prim_path: str = "/World/walkie_bot") -> Dict[str, Any]:
    """Configure and attach all simulation sensors to the robot.

    Args:
        config: Parsed simulation configuration dictionary.
        robot_prim_path: USD path where the robot is referenced.

    Returns:
        Dictionary containing configured sensor metadata for Action Graph wiring.
    """
    sensors_cfg = config.get("sensors", {})
    configured: Dict[str, Any] = {}

    # --------------------------------------------------------------------------
    # 1. 2D LiDAR (Center / Merged planar scan like Gazebo setup)
    # --------------------------------------------------------------------------
    # Backwards compatibility check for old config key 'lidar'
    lidar_2d_cfg = sensors_cfg.get("lidar_2d") or sensors_cfg.get("lidar", {})
    if lidar_2d_cfg.get("enabled", True):
        mount = lidar_2d_cfg.get(
            "mount_prim_path",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/merged_lidar_link",
        )
        sensor_prim = lidar_2d_cfg.get("sensor_prim_path", f"{mount}/RtxLidar2D")
        frame_id = lidar_2d_cfg.get("frame_id", "merged_lidar_link")
        topic = lidar_2d_cfg.get("topic_name", "/scan")
        rate_hz = float(lidar_2d_cfg.get("publish_rate_hz", 20.0))

        print(f"[Walkie-Isaac:Sensors] Initializing 2D LiDAR at {sensor_prim} (topic: {topic}, frame: {frame_id}, rate: {rate_hz} Hz)...")
        _create_rtx_lidar(sensor_prim, tick_rate=rate_hz, scan_rate_hz=rate_hz)
        configured["lidar_2d"] = {
            "sensor_prim_path": sensor_prim,
            "frame_id": frame_id,
            "topic_name": topic,
            "type": "laser_scan",
            "publish_rate_hz": rate_hz,
        }

    # --------------------------------------------------------------------------
    # 2. 3D LiDAR (Unitree 4D multi-ring point cloud)
    # --------------------------------------------------------------------------
    lidar_3d_cfg = sensors_cfg.get("lidar_3d", {})
    if lidar_3d_cfg.get("enabled", False):
        mount = lidar_3d_cfg.get(
            "mount_prim_path",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/unitree4d_l2_imu_initial",
        )
        sensor_prim = lidar_3d_cfg.get("sensor_prim_path", f"{mount}/RtxLidar3D")
        frame_id = lidar_3d_cfg.get("frame_id", "unitree4d_l2_imu_initial")
        topic = lidar_3d_cfg.get("topic_name", "/unilidar/cloud")
        rate_hz = float(lidar_3d_cfg.get("publish_rate_hz", 10.0))

        print(f"[Walkie-Isaac:Sensors] Initializing 3D LiDAR at {sensor_prim} (topic: {topic}, frame: {frame_id}, rate: {rate_hz} Hz)...")
        _create_rtx_lidar(sensor_prim, tick_rate=rate_hz, scan_rate_hz=rate_hz)
        configured["lidar_3d"] = {
            "sensor_prim_path": sensor_prim,
            "frame_id": frame_id,
            "topic_name": topic,
            "type": "point_cloud",
            "publish_rate_hz": rate_hz,
        }

    # --------------------------------------------------------------------------
    # 3. IMU (Mounted at the 3D LiDAR)
    # --------------------------------------------------------------------------
    imu_cfg = sensors_cfg.get("imu", {})
    if imu_cfg.get("enabled", False):
        mount = imu_cfg.get(
            "mount_prim_path",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/unitree4d_l2_imu_initial/unitree4d_l2_imu",
        )
        sensor_prim = imu_cfg.get("sensor_prim_path", f"{mount}/ImuSensor")
        frame_id = imu_cfg.get("frame_id", "unitree4d_l2_imu")
        topic = imu_cfg.get("topic_name", "/unilidar/imu")
        rate_hz = float(imu_cfg.get("publish_rate_hz", 60.0))

        print(f"[Walkie-Isaac:Sensors] Initializing IMU at {sensor_prim} (topic: {topic}, frame: {frame_id}, rate: {rate_hz} Hz)...")
        _create_imu_sensor(sensor_prim, publish_rate_hz=rate_hz)
        configured["imu"] = {
            "sensor_prim_path": sensor_prim,
            "frame_id": frame_id,
            "topic_name": topic,
            "publish_rate_hz": rate_hz,
        }

    # --------------------------------------------------------------------------
    # 4. Head Camera (ZED 2i: RGB + Depth + 3D Point Cloud + CameraInfo)
    # --------------------------------------------------------------------------
    head_cam_cfg = sensors_cfg.get("head_camera", {})
    if head_cam_cfg.get("enabled", False):
        mount = head_cam_cfg.get(
            "mount_prim_path",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/lift_link/head_servo_link/zed_head_camera_link/zed_head_camera_center/zed_head_left_camera_frame/zed_head_left_camera_frame_optical",
        )
        cam_prim = head_cam_cfg.get("camera_prim_path", f"{mount}/Camera")
        frame_id = head_cam_cfg.get("frame_id", "zed_head_left_camera_frame_optical")
        focal_len = float(head_cam_cfg.get("focal_length", 24.0))
        rate_hz = float(head_cam_cfg.get("publish_rate_hz", 15.0))

        print(f"[Walkie-Isaac:Sensors] Initializing Head Camera (ZED 2i) at {cam_prim} (frame: {frame_id}, rate: {rate_hz} Hz)...")
        _create_camera_prim(cam_prim, focal_length=focal_len, tick_rate=rate_hz)
        configured["head_camera"] = {
            "camera_prim_path": cam_prim,
            "frame_id": frame_id,
            "width": int(head_cam_cfg.get("width", 640)),
            "height": int(head_cam_cfg.get("height", 360)),
            "rgb_topic": head_cam_cfg.get("rgb_topic", "/zed_head/zed_node/rgb/image_rect_color"),
            "rgb_info_topic": head_cam_cfg.get("rgb_info_topic", "/zed_head/zed_node/rgb/camera_info"),
            "depth_topic": head_cam_cfg.get("depth_topic", "/zed_head/zed_node/depth/depth_registered"),
            "depth_pcl_topic": head_cam_cfg.get("depth_pcl_topic", "/zed_head/zed_node/point_cloud/cloud_registered"),
            "publish_rate_hz": rate_hz,
        }

    # --------------------------------------------------------------------------
    # 5. Bottom Camera (RealSense D415: RGB + Depth + 3D Point Cloud + CameraInfo)
    # --------------------------------------------------------------------------
    bottom_cam_cfg = sensors_cfg.get("bottom_camera", {})
    if bottom_cam_cfg.get("enabled", False):
        mount = bottom_cam_cfg.get(
            "mount_prim_path",
            f"{robot_prim_path}/Geometry/base_footprint/base_link/bottom_bottom_screw_frame/bottom_link/bottom_depth_frame/bottom_color_frame/bottom_color_optical_frame",
        )
        cam_prim = bottom_cam_cfg.get("camera_prim_path", f"{mount}/Camera")
        frame_id = bottom_cam_cfg.get("frame_id", "bottom_color_optical_frame")
        focal_len = float(bottom_cam_cfg.get("focal_length", 24.0))
        rate_hz = float(bottom_cam_cfg.get("publish_rate_hz", 15.0))

        print(f"[Walkie-Isaac:Sensors] Initializing Bottom Camera (RealSense D415) at {cam_prim} (frame: {frame_id}, rate: {rate_hz} Hz)...")
        _create_camera_prim(cam_prim, focal_length=focal_len, tick_rate=rate_hz)
        configured["bottom_camera"] = {
            "camera_prim_path": cam_prim,
            "frame_id": frame_id,
            "width": int(bottom_cam_cfg.get("width", 640)),
            "height": int(bottom_cam_cfg.get("height", 480)),
            "rgb_topic": bottom_cam_cfg.get("rgb_topic", "/bottom/color/image_raw"),
            "rgb_info_topic": bottom_cam_cfg.get("rgb_info_topic", "/bottom/color/camera_info"),
            "depth_topic": bottom_cam_cfg.get("depth_topic", "/bottom/depth/image_rect_raw"),
            "depth_pcl_topic": bottom_cam_cfg.get("depth_pcl_topic", "/camera/camera/depth/color/points"),
            "publish_rate_hz": rate_hz,
        }

    print(f"[Walkie-Isaac:Sensors] All sensors successfully initialized: {list(configured.keys())}")
    return configured
