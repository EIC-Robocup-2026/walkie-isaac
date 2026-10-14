# -*- coding: utf-8 -*-

"""Sensor initialization and configuration module for Walkie-Isaac."""

from typing import Any, Dict


def setup_sensors(config: Dict[str, Any], robot_prim_path: str = "/World/walkie_bot") -> Dict[str, Any]:
    """Configure and attach simulation sensors (e.g., RTX LiDAR) to the robot.

    Args:
        config: Parsed simulation configuration dictionary.
        robot_prim_path: USD path where the robot is referenced.

    Returns:
        Dictionary containing configured sensor metadata.
    """
    import carb
    from isaacsim.sensors.experimental.rtx import Lidar

    sensors_cfg = config.get("sensors", {})
    lidar_cfg = sensors_cfg.get("lidar", {})

    # Default mount is front_lidar_link under base_link
    mount_prim_path = lidar_cfg.get(
        "mount_prim_path",
        f"{robot_prim_path}/Geometry/base_footprint/base_link/front_lidar_link",
    )
    sensor_prim_path = lidar_cfg.get("sensor_prim_path", f"{mount_prim_path}/RtxLidar")
    lidar_frame_id = lidar_cfg.get("frame_id", "front_lidar_link")
    lidar_topic = lidar_cfg.get("topic_name", "/scan")
    lidar_type = lidar_cfg.get("type", "laser_scan")

    print(f"[Walkie-Isaac:Sensors] Creating RTX LiDAR sensor at {sensor_prim_path}...")
    try:
        Lidar(path=sensor_prim_path)
    except Exception as e:
        carb.log_warn(f"Lidar experimental wrapper returned: {e}. Trying Kit command fallback...")
        import omni.kit.commands
        omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path=sensor_prim_path)

    print(f"[Walkie-Isaac:Sensors] LiDAR initialized (frame_id='{lidar_frame_id}', topic='{lidar_topic}')")

    return {
        "lidar": {
            "mount_prim_path": mount_prim_path,
            "sensor_prim_path": sensor_prim_path,
            "frame_id": lidar_frame_id,
            "topic_name": lidar_topic,
            "type": lidar_type,
        }
    }
