#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==============================================================================
# Walkie Bot - Isaac Sim Standalone Simulation Launcher
# RoboCup 2027 Integration (Modular Architecture)
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
# Ensure walkie_isaac module package is importable
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

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
    headless = (
        args.headless
        if args.headless is not None
        else config.get("simulation", {}).get("headless", False)
    )

    # --------------------------------------------------------------------------
    # 0. Zenoh Router Preflight Check
    # Ensure Zenoh router (rmw_zenohd) daemon is active if using rmw_zenoh_cpp
    # --------------------------------------------------------------------------
    active_rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    config_rmw = config.get("ros2", {}).get("rmw", "")
    if active_rmw == "rmw_zenoh_cpp" or config_rmw == "rmw_zenoh_cpp":
        from walkie_isaac.ros2_bridge import ensure_zenoh_router
        ensure_zenoh_router()

    # --------------------------------------------------------------------------
    # 1. Initialize Isaac Sim SimulationApp
    # --------------------------------------------------------------------------
    print(f"[Walkie-Isaac] Initializing SimulationApp (headless={headless})...")
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": headless})

    # --------------------------------------------------------------------------
    # 2. Enable ROS 2 Bridge Extension
    # --------------------------------------------------------------------------
    print("[Walkie-Isaac] Enabling isaacsim.ros2.bridge extension...")
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")

    # --------------------------------------------------------------------------
    # 3. Import simulation modules (safe after SimulationApp is active)
    # --------------------------------------------------------------------------
    import omni.timeline
    import omni.usd
    from pxr import Sdf, UsdLux
    from isaacsim.core.api import World

    from walkie_isaac.robot import setup_robot
    from walkie_isaac.sensors import setup_sensors
    from walkie_isaac.ros2_bridge import setup_ros2_action_graph, CmdVelSubscriber

    # --------------------------------------------------------------------------
    # 4. Initialize Simulation World & Default Lighting
    # --------------------------------------------------------------------------
    sim_cfg = config.get("simulation", {})
    physics_dt = float(sim_cfg.get("physics_dt", 1.0 / 60.0))
    rendering_dt = float(sim_cfg.get("rendering_dt", 1.0 / 60.0))

    print(
        f"[Walkie-Isaac] Initializing World (physics_dt={physics_dt:.4f}, rendering_dt={rendering_dt:.4f})..."
    )
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
    world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/World/defaultLight"):
        light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/defaultLight"))
        light_prim.CreateIntensityAttr(1000.0)

    # --------------------------------------------------------------------------
    # 5. Robot Setup Module (USD reference, PhysX variant, drive tuning, controller)
    # --------------------------------------------------------------------------
    robot_prim_path = config.get("robot", {}).get("prim_path", "/World/walkie_bot")
    robot_art, base_controller = setup_robot(config, world, SCRIPT_DIR)

    # --------------------------------------------------------------------------
    # 6. Sensors Setup Module (RTX LiDARs, Cameras, IMU)
    # --------------------------------------------------------------------------
    sensors = setup_sensors(config, robot_prim_path)

    # --------------------------------------------------------------------------
    # 7. ROS 2 Bridge Setup Module (Action Graph & Teleop Subscriber)
    # --------------------------------------------------------------------------
    setup_ros2_action_graph(config, sensors, robot_prim_path=robot_prim_path)
    cmd_vel_topic = config.get("ros2", {}).get("cmd_vel_topic", "/cmd_vel")
    cmd_vel_sub = CmdVelSubscriber(topic_name=cmd_vel_topic, timeout_sec=0.5)

    # --------------------------------------------------------------------------
    # 8. Physics Loop Execution
    # --------------------------------------------------------------------------
    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    step_count = 0
    print("[Walkie-Isaac] Simulation running. Advancing physics loop...")
    while simulation_app.is_running():
        # Update teleoperation command to omni wheels
        cmd_vel_sub.update(base_controller)

        # Advance physics step
        world.step(render=True)
        step_count += 1

        if args.test and step_count >= args.steps:
            print(
                f"[Walkie-Isaac] Test mode completed {step_count} steps. Shutting down cleanly."
            )
            break

    cmd_vel_sub.destroy()
    timeline.stop()
    simulation_app.close()
    print("[Walkie-Isaac] Simulation finished.")


if __name__ == "__main__":
    main()
