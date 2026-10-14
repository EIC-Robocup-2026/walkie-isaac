#!/usr/bin/env python3
"""Interactive ROS 2 Joint Command test script for Walkie Bot in Isaac Sim.

Publishes target joint positions to `/joint_commands` (sensor_msgs/JointState)
to exercise:
  1. Torso Lift (lower and raise)
  2. Head Servo (pan left and right)
  3. Left & Right OpenArm (MoveIt home -> hands up -> wave -> home)
  4. Parallel Grippers (open and close)
"""
import time
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def publish_pose(pub, node, joint_dict, duration_sec=2.0, steps=20):
    msg = JointState()
    msg.name = list(joint_dict.keys())
    msg.position = [float(v) for v in joint_dict.values()]
    dt = duration_sec / steps
    for _ in range(steps):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(dt)


def main():
    rclpy.init()
    node = rclpy.create_node("walkie_joint_tester")
    pub = node.create_publisher(JointState, "/joint_commands", 10)

    print("=" * 60)
    print(" Walkie Bot Isaac Sim Joint Control Test")
    print(" Target topic: /joint_commands (sensor_msgs/msg/JointState)")
    print("=" * 60)
    time.sleep(1.0)

    try:
        # Step 1: Head Pan
        print("\n[1/5] Panning head servo (+0.5 rad / left)...")
        publish_pose(pub, node, {"head_servo_joint": 0.5}, duration_sec=1.5)
        print("      Panning head servo (-0.5 rad / right)...")
        publish_pose(pub, node, {"head_servo_joint": -0.5}, duration_sec=1.5)
        print("      Returning head to center (0.0 rad)...")
        publish_pose(pub, node, {"head_servo_joint": 0.0}, duration_sec=1.0)

        # Step 2: Torso Lift
        print("\n[2/5] Lowering lift joint (-0.3 m)...")
        publish_pose(pub, node, {"lift_joint": -0.3}, duration_sec=2.0)
        print("      Raising lift joint (0.0 m)...")
        publish_pose(pub, node, {"lift_joint": 0.0}, duration_sec=2.0)

        # Step 3: Dual Arms - "Hands Up" pose
        print("\n[3/5] Moving arms to 'Hands Up' pose...")
        hands_up = {
            "openarm_left_joint1": 0.6458,
            "openarm_left_joint4": 2.0769,
            "openarm_left_joint7": -0.2094,
            "openarm_right_joint1": -0.6458,
            "openarm_right_joint4": 2.0769,
            "openarm_right_joint7": 0.2094,
        }
        publish_pose(pub, node, hands_up, duration_sec=3.0)

        # Step 4: Left Arm Wave
        print("\n[4/5] Waving left wrist...")
        for _ in range(2):
            publish_pose(pub, node, {"openarm_left_joint6": 0.5}, duration_sec=0.6)
            publish_pose(pub, node, {"openarm_left_joint6": -0.5}, duration_sec=0.6)
        publish_pose(pub, node, {"openarm_left_joint6": 0.0}, duration_sec=0.5)

        # Step 5: Grippers test
        print("\n[5/5] Testing gripper finger joints...")
        publish_pose(pub, node, {
            "openarm_left_finger_joint1": 0.03,
            "openarm_right_finger_joint1": 0.03,
        }, duration_sec=1.5)
        publish_pose(pub, node, {
            "openarm_left_finger_joint1": 0.0,
            "openarm_right_finger_joint1": 0.0,
        }, duration_sec=1.5)

        # Return to Home Pose
        print("\nReturning dual arms to nominal 'Home' pose...")
        home_pose = {
            "openarm_left_joint1": 0.2618,
            "openarm_left_joint2": 0.0,
            "openarm_left_joint3": 0.0,
            "openarm_left_joint4": 0.2618,
            "openarm_left_joint5": 0.0,
            "openarm_left_joint6": 0.0,
            "openarm_left_joint7": 0.0,
            "openarm_right_joint1": -0.2618,
            "openarm_right_joint2": 0.0,
            "openarm_right_joint3": 0.0,
            "openarm_right_joint4": 0.2618,
            "openarm_right_joint5": 0.0,
            "openarm_right_joint6": 0.0,
            "openarm_right_joint7": 0.0,
            "lift_joint": 0.0,
            "head_servo_joint": 0.0,
        }
        publish_pose(pub, node, home_pose, duration_sec=2.5)

        print("\n" + "=" * 60)
        print(" Test routine finished successfully!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
