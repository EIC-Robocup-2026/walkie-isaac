#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==============================================================================
# Walkie Bot - Isaac Sim Standalone Simulation Launcher
# RoboCup 2027 Integration
# ==============================================================================

import argparse
import os
import sys
from pathlib import Path
import yaml

# ------------------------------------------------------------------------------
# Principle 1: Environment Isolation & Sanitization
# Ensure host/Pixi PYTHONPATH does not leak into Isaac Sim's Python runtime
# ------------------------------------------------------------------------------
if "PYTHONPATH" in os.environ:
    del os.environ["PYTHONPATH"]

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "config" / "sim_config.yaml"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Walkie Bot standalone Isaac Sim launcher with auto-generated ROS 2 Action Graph"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_CONFIG),
        help="Path to sim_config.yaml (default: %(default)s)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=None,
        help="Run simulator in headless mode without GUI (overrides config file)",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        default=False,
        help="Run in test mode for a fixed number of steps and exit cleanly",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=120,
        help="Number of steps to run when --test is enabled (default: %(default)s)",
    )
    return parser.parse_args()


def load_config(config_path_str: str) -> dict:
    config_path = Path(config_path_str)
    if not config_path.is_absolute():
        config_path = (SCRIPT_DIR / config_path).resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    args = parse_args()
    config = load_config(args.config)

    # Resolve headless setting: CLI argument takes precedence over config
    headless = args.headless if args.headless is not None else config.get("simulation", {}).get("headless", False)

    # --------------------------------------------------------------------------
    # Initialize Isaac Sim SimulationApp
    # --------------------------------------------------------------------------
    print(f"[Walkie-Isaac] Initializing SimulationApp (headless={headless})...")
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": headless})

    # Enable ROS 2 bridge extension
    print("[Walkie-Isaac] Enabling isaacsim.ros2.bridge extension...")
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")

    # Import Isaac Sim and Omni modules after SimulationApp startup
    import carb
    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    from pxr import Sdf, UsdGeom, UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
    from isaacsim.sensors.experimental.rtx import Lidar

    # Parse simulation config values
    sim_cfg = config.get("simulation", {})
    physics_dt = float(sim_cfg.get("physics_dt", 1.0 / 60.0))
    rendering_dt = float(sim_cfg.get("rendering_dt", 1.0 / 60.0))

    robot_cfg = config.get("robot", {})
    usd_path_str = robot_cfg.get("usd_path", "assets/Collected_gz_walkie_roller/gz_walkie_roller_absolute_path.usda")
    robot_usd_path = Path(usd_path_str)
    if not robot_usd_path.is_absolute():
        robot_usd_path = (SCRIPT_DIR / robot_usd_path).resolve()
    if not robot_usd_path.is_file():
        raise FileNotFoundError(f"Robot USD model not found: {robot_usd_path}")
    robot_prim_path = robot_cfg.get("prim_path", "/World/walkie_bot")

    sensors_cfg = config.get("sensors", {})
    lidar_cfg = sensors_cfg.get("lidar", {})
    mount_prim_path = lidar_cfg.get(
        "mount_prim_path",
        f"{robot_prim_path}/Geometry/base_footprint/base_link/front_lidar_link",
    )
    sensor_prim_path = lidar_cfg.get("sensor_prim_path", f"{mount_prim_path}/RtxLidar")
    lidar_frame_id = lidar_cfg.get("frame_id", "front_lidar_link")
    lidar_topic = lidar_cfg.get("topic_name", "/scan")
    lidar_type = lidar_cfg.get("type", "laser_scan")

    ros2_cfg = config.get("ros2", {})
    domain_id = int(ros2_cfg.get("domain_id", 0))
    clock_topic = ros2_cfg.get("clock_topic", "/clock")
    cmd_vel_topic = ros2_cfg.get("cmd_vel_topic", "/cmd_vel")

    # --------------------------------------------------------------------------
    # Principle 4: Clean Stage Referencing
    # SimulationApp boots a clean stage; we initialize World and reference only
    # the robot prim to extract kinematics while ignoring GUI viewport artifacts.
    # --------------------------------------------------------------------------
    print(f"[Walkie-Isaac] Initializing simulation World (physics_dt={physics_dt}, rendering_dt={rendering_dt})...")
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
    world.scene.add_default_ground_plane()

    # Add default scene lighting
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/World/defaultLight"):
        light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
        light_prim.CreateIntensityAttr(1000.0)

    # Reference robot USD
    print(f"[Walkie-Isaac] Referencing robot USD: {robot_usd_path} -> {robot_prim_path}")
    add_reference_to_stage(usd_path=str(robot_usd_path), prim_path=robot_prim_path)

    # --------------------------------------------------------------------------
    # Attach RTX LiDAR sensor to designated mount
    # --------------------------------------------------------------------------
    print(f"[Walkie-Isaac] Creating RTX Lidar sensor at {sensor_prim_path}...")
    try:
        Lidar(path=sensor_prim_path)
    except Exception as e:
        carb.log_warn(f"Lidar experimental wrapper returned: {e}. Trying Kit command fallback...")
        import omni.kit.commands
        omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path=sensor_prim_path)

    # --------------------------------------------------------------------------
    # Auto-generate ROS 2 Action Graph at runtime
    #   • OnPlaybackTick + IsaacReadSimulationTime -> ROS2PublishClock
    #   • OnPlaybackTick + ROS2Context             -> ROS2SubscribeTwist
    #   • OnPlaybackTick + IsaacCreateRenderProduct-> ROS2RtxLidarHelper
    # --------------------------------------------------------------------------
    graph_path = "/ROS2_ActionGraph"
    print(f"[Walkie-Isaac] Building ROS 2 Action Graph at {graph_path}...")
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

    print("[Walkie-Isaac] Action Graph successfully built!")
    print(f"  • Clock Topic:   {clock_topic} (rosgraph_msgs/msg/Clock)")
    print(f"  • Velocity Cmd:  {cmd_vel_topic} (geometry_msgs/msg/Twist)")
    print(f"  • LiDAR Scan:    {lidar_topic} (sensor_msgs/msg/LaserScan, frame: {lidar_frame_id})")

    # --------------------------------------------------------------------------
    # Physics Loop Execution
    # --------------------------------------------------------------------------
    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    step_count = 0
    print("[Walkie-Isaac] Simulation running. Advancing physics loop...")
    while simulation_app.is_running():
        world.step(render=True)
        step_count += 1
        if args.test and step_count >= args.steps:
            print(f"[Walkie-Isaac] Test mode completed {step_count} steps. Shutting down cleanly.")
            break

    timeline.stop()
    simulation_app.close()
    print("[Walkie-Isaac] Simulation finished.")


if __name__ == "__main__":
    main()
