# -*- coding: utf-8 -*-

"""Robot spawning, physics setup, and omnidirectional base control for Walkie-Isaac."""

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class OmniBaseController:
    """Holonomic 4-wheel omnidirectional base controller for Walkie Bot.

    Wheel layout (in base_footprint frame: +x forward, +y left):
      - fl_wheel_joint: 45°  (pi/4)
      - bl_wheel_joint: 135° (3*pi/4)
      - br_wheel_joint: 225° (5*pi/4)
      - fr_wheel_joint: 315° (7*pi/4)
    """

    def __init__(
        self,
        articulation: Any,
        robot_radius: float = 0.185,
        wheel_radius: float = 0.076,
    ):
        """Initialize the controller.

        Args:
            articulation: SingleArticulation instance of the robot.
            robot_radius: Distance from base_footprint center to wheels in meters (default 0.185 m).
            wheel_radius: Wheel radius in meters (default 0.076 m).
        """
        from isaacsim.core.utils.types import ArticulationAction

        self._articulation = articulation
        self._action_cls = ArticulationAction
        self.robot_radius = robot_radius
        self.wheel_radius = wheel_radius

        self.wheel_names: List[str] = [
            "fl_wheel_joint",
            "bl_wheel_joint",
            "br_wheel_joint",
            "fr_wheel_joint",
        ]
        self.wheel_angles: List[float] = [
            math.pi / 4.0,       # 45 deg (FL)
            3.0 * math.pi / 4.0, # 135 deg (BL)
            5.0 * math.pi / 4.0, # 225 deg (BR)
            7.0 * math.pi / 4.0, # 315 deg (FR)
        ]

        self._wheel_indices: Optional[List[int]] = None

    @property
    def wheel_indices(self) -> List[int]:
        """Get cached DOF indices of the 4 wheel joints."""
        if self._wheel_indices is None or any(i is None for i in self._wheel_indices):
            indices = [
                self._articulation.get_dof_index(name) for name in self.wheel_names
            ]
            if not any(i is None for i in indices):
                self._wheel_indices = indices
                print(f"[Walkie-Isaac:Robot] OmniBaseController cached wheel indices: {self._wheel_indices}")
            return indices
        return self._wheel_indices

    def apply_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Calculate and apply 4-wheel joint velocities from body twist command.

        Formula:
            wheel_speed = (sin(theta) * vx - cos(theta) * vy - wz * robot_radius) / wheel_radius

        Args:
            vx: Linear velocity in robot forward direction (m/s).
            vy: Linear velocity in robot left direction (m/s).
            wz: Angular velocity around vertical axis (rad/s).
        """
        indices = self.wheel_indices
        if any(idx is None for idx in indices):
            return

        wheel_speeds = [
            (math.sin(theta) * vx - math.cos(theta) * vy - wz * self.robot_radius)
            / self.wheel_radius
            for theta in self.wheel_angles
        ]

        action = self._action_cls(
            joint_velocities=wheel_speeds,
            joint_indices=indices,
        )
        self._articulation.apply_action(action)

    def stop(self) -> None:
        """Command zero velocity to all wheel joints."""
        self.apply_velocity(0.0, 0.0, 0.0)


def setup_robot(
    config: Dict[str, Any],
    world: Any,
    script_dir: Path,
) -> Tuple[Any, OmniBaseController]:
    """Load robot USD, activate PhysX physics, tune drives, and instantiate base controller.

    Args:
        config: Parsed simulation configuration dictionary.
        world: Isaac Sim World instance.
        script_dir: Root directory of walkie-isaac containing assets.

    Returns:
        Tuple of (SingleArticulation, OmniBaseController).
    """
    import omni.usd
    from pxr import UsdPhysics
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage

    robot_cfg = config.get("robot", {})
    usd_path_str = robot_cfg.get(
        "usd_path",
        "assets/Collected_gz_walkie_roller/gz_walkie_roller_absolute_path.usda",
    )
    robot_usd_path = Path(usd_path_str)
    if not robot_usd_path.is_absolute():
        robot_usd_path = (script_dir / robot_usd_path).resolve()
    if not robot_usd_path.is_file():
        raise FileNotFoundError(f"Robot USD model not found: {robot_usd_path}")

    robot_prim_path = robot_cfg.get("prim_path", "/World/walkie_bot")

    # Reference robot USD cleanly into the stage
    print(f"[Walkie-Isaac:Robot] Referencing robot USD: {robot_usd_path} -> {robot_prim_path}")
    add_reference_to_stage(usd_path=str(robot_usd_path), prim_path=robot_prim_path)

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)

    # Activate PhysX variant set
    if robot_prim.IsValid() and robot_prim.GetVariantSets().HasVariantSet("Physics"):
        robot_prim.GetVariantSet("Physics").SetVariantSelection("physx")
        print("[Walkie-Isaac:Robot] Activated PhysX physics variant (145+ joints loaded)")

    # Set initial robot spawn translation
    spawn_pos = robot_cfg.get("spawn_position", [0.0, 0.0, 0.05])
    if robot_prim.IsValid() and spawn_pos is not None:
        from pxr import Gf, UsdGeom
        xformable = UsdGeom.Xformable(robot_prim)
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(float(spawn_pos[0]), float(spawn_pos[1]), float(spawn_pos[2])))
        print(f"[Walkie-Isaac:Robot] Set robot spawn position to {spawn_pos}")

    # Configure wheel drives for velocity control (damping = gain Kv, stiffness = 0)
    wheel_names = ["fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint", "fr_wheel_joint"]
    for w_name in wheel_names:
        joint_prim_path = f"{robot_prim_path}/Physics/{w_name}"
        joint_prim = stage.GetPrimAtPath(joint_prim_path)
        if joint_prim.IsValid():
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if drive:
                drive.GetDampingAttr().Set(1000.0)
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetMaxForceAttr().Set(100.0)
        else:
            print(f"[Walkie-Isaac:Robot] Warning: joint prim not found at {joint_prim_path}")

    print("[Walkie-Isaac:Robot] Configured wheel velocity drives (damping=1000.0, stiffness=0.0)")

    # Create SingleArticulation wrapper and register with simulation world
    robot_art = SingleArticulation(robot_prim_path)
    world.scene.add(robot_art)

    # Initialize omnidirectional base controller
    base_controller = OmniBaseController(
        articulation=robot_art,
        robot_radius=float(robot_cfg.get("robot_radius", 0.185)),
        wheel_radius=float(robot_cfg.get("wheel_radius", 0.076)),
    )

    return robot_art, base_controller
