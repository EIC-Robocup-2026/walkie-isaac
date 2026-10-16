# -*- coding: utf-8 -*-

"""Robot spawning, physics setup, and omnidirectional base control for Walkie-Isaac."""

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class OmniBaseController:
    """Holonomic 4-wheel omnidirectional base controller for Walkie Bot.

    Reads kinematics (robot_radius, wheel_radius) and wheel angles dynamically
    from the drivebase configuration.
    """

    def __init__(
        self,
        articulation: Any,
        drivebase_cfg: Optional[Dict[str, Any]] = None,
        robot_radius: float = 0.185,
        wheel_radius: float = 0.076,
    ):
        """Initialize the controller.

        Args:
            articulation: SingleArticulation instance of the robot.
            drivebase_cfg: Optional parsed drivebase configuration dictionary.
            robot_radius: Fallback distance from base center to wheels (meters).
            wheel_radius: Fallback wheel radius (meters).
        """
        from isaacsim.core.utils.types import ArticulationAction

        self._articulation = articulation
        self._action_cls = ArticulationAction

        drivebase_cfg = drivebase_cfg or {}
        kinematics = drivebase_cfg.get("kinematics", {})
        self.robot_radius = float(kinematics.get("robot_radius", robot_radius))
        self.wheel_radius = float(kinematics.get("wheel_radius", wheel_radius))

        wheels = drivebase_cfg.get("wheels", {})
        if wheels:
            self.wheel_names: List[str] = list(wheels.keys())
            self.wheel_angles: List[float] = [
                math.radians(float(wheels[w].get("angle_deg", 0.0)))
                for w in self.wheel_names
            ]
        else:
            # Fallback default geometry
            self.wheel_names = [
                "fl_wheel_joint",
                "bl_wheel_joint",
                "br_wheel_joint",
                "fr_wheel_joint",
            ]
            self.wheel_angles = [
                math.pi / 4.0,       # 45 deg (FL)
                3.0 * math.pi / 4.0, # 135 deg (BL)
                5.0 * math.pi / 4.0, # 225 deg (BR)
                7.0 * math.pi / 4.0, # 315 deg (FR)
            ]

        self._wheel_indices: Optional[List[int]] = None

    @property
    def wheel_indices(self) -> List[int]:
        """Get cached DOF indices of the wheel joints."""
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
    """Load robot USD, tune drives using modular configs, and instantiate base controller.

    Args:
        config: Parsed master configuration dictionary containing 'drivebase' and 'arm'.
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
    drivebase_cfg = config.get("drivebase", {})
    arm_cfg = config.get("arm", {})

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

    # --------------------------------------------------------------------------
    # 1. Configure Wheel Drives (Drivebase Module)
    # --------------------------------------------------------------------------
    wheels_cfg = drivebase_cfg.get("wheels", {
        "fl_wheel_joint": {"drive_type": "angular", "damping": 1000.0, "stiffness": 0.0, "max_force": 100.0},
        "bl_wheel_joint": {"drive_type": "angular", "damping": 1000.0, "stiffness": 0.0, "max_force": 100.0},
        "br_wheel_joint": {"drive_type": "angular", "damping": 1000.0, "stiffness": 0.0, "max_force": 100.0},
        "fr_wheel_joint": {"drive_type": "angular", "damping": 1000.0, "stiffness": 0.0, "max_force": 100.0},
    })
    for w_name, w_params in wheels_cfg.items():
        joint_prim_path = f"{robot_prim_path}/Physics/{w_name}"
        joint_prim = stage.GetPrimAtPath(joint_prim_path)
        if joint_prim.IsValid():
            dtype = w_params.get("drive_type", "angular")
            drive = UsdPhysics.DriveAPI.Get(joint_prim, dtype)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, dtype)
            drive.GetDampingAttr().Set(float(w_params.get("damping", 1000.0)))
            drive.GetStiffnessAttr().Set(float(w_params.get("stiffness", 0.0)))
            drive.GetMaxForceAttr().Set(float(w_params.get("max_force", 100.0)))
        else:
            print(f"[Walkie-Isaac:Robot] Warning: wheel joint prim not found at {joint_prim_path}")
    print(f"[Walkie-Isaac:Robot] Configured {len(wheels_cfg)} wheel drives from drivebase config")

    # --------------------------------------------------------------------------
    # 2. Configure Torso Lift Drive (Arm Module)
    # --------------------------------------------------------------------------
    lift_cfg = arm_cfg.get("lift", {})
    lift_name = lift_cfg.get("joint_name", "lift_joint")
    lift_prim_path = f"{robot_prim_path}/Physics/{lift_name}"
    lift_prim = stage.GetPrimAtPath(lift_prim_path)
    if lift_prim.IsValid():
        dtype = lift_cfg.get("drive_type", "linear")
        drive = UsdPhysics.DriveAPI.Get(lift_prim, dtype) or UsdPhysics.DriveAPI.Apply(lift_prim, dtype)
        drive.GetDampingAttr().Set(float(lift_cfg.get("damping", 10000.0)))
        drive.GetStiffnessAttr().Set(float(lift_cfg.get("stiffness", 100000.0)))
        drive.GetMaxForceAttr().Set(float(lift_cfg.get("max_force", 10000.0)))
        target_attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()
        target_attr.Set(float(lift_cfg.get("home_position", 0.0)))
        print(f"[Walkie-Isaac:Robot] Configured lift position drive: {lift_name}")

    # --------------------------------------------------------------------------
    # 3. Configure Head Servo Drive (Arm Module)
    # --------------------------------------------------------------------------
    head_cfg = arm_cfg.get("head", {})
    head_name = head_cfg.get("joint_name", "head_servo_joint")
    head_prim_path = f"{robot_prim_path}/Physics/{head_name}"
    head_prim = stage.GetPrimAtPath(head_prim_path)
    if head_prim.IsValid():
        dtype = head_cfg.get("drive_type", "angular")
        drive = UsdPhysics.DriveAPI.Get(head_prim, dtype) or UsdPhysics.DriveAPI.Apply(head_prim, dtype)
        drive.GetDampingAttr().Set(float(head_cfg.get("damping", 200.0)))
        drive.GetStiffnessAttr().Set(float(head_cfg.get("stiffness", 5000.0)))
        drive.GetMaxForceAttr().Set(float(head_cfg.get("max_force", 50.0)))
        target_attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()
        target_attr.Set(float(head_cfg.get("home_position_deg", 0.0)))
        print(f"[Walkie-Isaac:Robot] Configured head servo position drive: {head_name}")

    # --------------------------------------------------------------------------
    # 4. Configure Dual Arms (Arm Module)
    # --------------------------------------------------------------------------
    arm_count = 0
    for arm_key in ["left_arm", "right_arm"]:
        arm_section = arm_cfg.get(arm_key, {})
        defaults = arm_section.get("drive_defaults", {})
        dtype = defaults.get("drive_type", "angular")
        default_stiffness = float(defaults.get("stiffness", 5000.0))
        default_damping = float(defaults.get("damping", 200.0))
        default_max_force = float(defaults.get("max_force", 100.0))

        joints = arm_section.get("joints", {})
        for j_name, j_params in joints.items():
            joint_path = f"{robot_prim_path}/Physics/{j_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                drive = UsdPhysics.DriveAPI.Get(joint_prim, dtype) or UsdPhysics.DriveAPI.Apply(joint_prim, dtype)
                stiff = float(j_params.get("stiffness", default_stiffness))
                damp = float(j_params.get("damping", default_damping))
                mf = float(j_params.get("max_force", default_max_force))
                drive.GetStiffnessAttr().Set(stiff)
                drive.GetDampingAttr().Set(damp)
                drive.GetMaxForceAttr().Set(mf)
                target_deg = float(j_params.get("home_deg", 0.0))
                target_attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()
                target_attr.Set(target_deg)
                arm_count += 1
    print(f"[Walkie-Isaac:Robot] Configured {arm_count} arm joint position drives from arm config")

    # --------------------------------------------------------------------------
    # 5. Configure Gripper Fingers (Arm Module)
    # --------------------------------------------------------------------------
    grippers_cfg = arm_cfg.get("grippers", {})
    g_defaults = grippers_cfg.get("drive_defaults", {})
    g_dtype = g_defaults.get("drive_type", "linear")
    g_stiff = float(g_defaults.get("stiffness", 2000.0))
    g_damp = float(g_defaults.get("damping", 100.0))
    g_mf = float(g_defaults.get("max_force", 50.0))
    g_home = float(g_defaults.get("home_position", 0.0))

    finger_names = grippers_cfg.get("fingers", [
        "openarm_left_finger_joint1", "openarm_left_finger_joint2",
        "openarm_right_finger_joint1", "openarm_right_finger_joint2",
    ])
    for finger_name in finger_names:
        finger_path = f"{robot_prim_path}/Physics/{finger_name}"
        finger_prim = stage.GetPrimAtPath(finger_path)
        if finger_prim.IsValid():
            drive = UsdPhysics.DriveAPI.Get(finger_prim, g_dtype) or UsdPhysics.DriveAPI.Apply(finger_prim, g_dtype)
            drive.GetDampingAttr().Set(g_damp)
            drive.GetStiffnessAttr().Set(g_stiff)
            drive.GetMaxForceAttr().Set(g_mf)
            target_attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()
            target_attr.Set(g_home)
    print(f"[Walkie-Isaac:Robot] Configured {len(finger_names)} gripper finger position drives from arm config")

    # --------------------------------------------------------------------------
    # 6. Instantiate SingleArticulation and Controller
    # --------------------------------------------------------------------------
    robot_art = SingleArticulation(robot_prim_path)
    world.scene.add(robot_art)

    base_controller = OmniBaseController(
        articulation=robot_art,
        drivebase_cfg=drivebase_cfg,
    )

    return robot_art, base_controller
