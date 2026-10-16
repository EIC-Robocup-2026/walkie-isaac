# -*- coding: utf-8 -*-

"""Configuration loader and schema resolver for Walkie-Isaac.

Supports loading top-level configuration with modular sub-configs:
  • drivebase.yaml (kinematics, wheels, gains)
  • arm.yaml (lift, head servo, dual arms, grippers)
  • sensors.yaml (2D LiDAR, 3D LiDAR, IMU, cameras)
"""

from pathlib import Path
from typing import Any, Dict
import yaml


def load_yaml(file_path: Path) -> Dict[str, Any]:
    """Safely load a YAML file.

    Args:
        file_path: Path to YAML file.

    Returns:
        Loaded dictionary, or empty dict if file is empty or missing.
    """
    if not file_path.is_file():
        return {}
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_modular_config(config_path_str: str, base_dir: Path) -> Dict[str, Any]:
    """Load master simulation config and merge modular subsystem configs.

    Args:
        config_path_str: Path to master config YAML (relative or absolute).
        base_dir: Base directory for resolving relative paths (typically walkie-isaac root).

    Returns:
        Complete configuration dictionary with merged 'drivebase', 'arm', and 'sensors' modules.
    """
    config_path = Path(config_path_str)
    if not config_path.is_absolute():
        config_path = (base_dir / config_path).resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    config = load_yaml(config_path)
    config_dir = config_path.parent

    # Resolve modular configuration files
    modules = config.get("modules", {})

    # 1. Drivebase Module
    db_file = modules.get("drivebase", "drivebase.yaml")
    db_path = (config_dir / db_file).resolve() if not Path(db_file).is_absolute() else Path(db_file)
    if db_path.is_file():
        db_data = load_yaml(db_path)
        # Inlined values in master config take precedence if present
        existing_db = config.get("drivebase", {})
        merged_db = {**db_data, **existing_db}
        config["drivebase"] = merged_db
        print(f"[Walkie-Isaac:Config] Loaded drivebase module: {db_path.name}")

    # 2. Arm / Lift / Gripper Module
    arm_file = modules.get("arm", "arm.yaml")
    arm_path = (config_dir / arm_file).resolve() if not Path(arm_file).is_absolute() else Path(arm_file)
    if arm_path.is_file():
        arm_data = load_yaml(arm_path)
        existing_arm = config.get("arm", {})
        merged_arm = {**arm_data, **existing_arm}
        config["arm"] = merged_arm
        print(f"[Walkie-Isaac:Config] Loaded arm module: {arm_path.name}")

    # 3. Sensors Module
    sens_file = modules.get("sensors", "sensors.yaml")
    sens_path = (config_dir / sens_file).resolve() if not Path(sens_file).is_absolute() else Path(sens_file)
    if sens_path.is_file():
        sens_data = load_yaml(sens_path)
        existing_sens = config.get("sensors", {})
        merged_sens = {**sens_data, **existing_sens}
        config["sensors"] = merged_sens
        print(f"[Walkie-Isaac:Config] Loaded sensors module: {sens_path.name}")

    return config
