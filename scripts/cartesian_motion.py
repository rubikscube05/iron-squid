#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point


class GazeboCartesianArm(Node):

    def __init__(self):

        super().__init__('gazebo_cartesian_arm')

        # =========================================================
        # ROBOT
        # =========================================================

        self.num_sections = 20

        # Approximate total bending-chain length.
        #
        # This comes from the spacing between the 20 sections
        # in the current URDF generator.
        self.arm_length = 0.8075

        # Maximum joint angle from your URDF
        self.max_joint_angle = 0.5

        # =========================================================
        # ROS
        # =========================================================

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        self.target_sub = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            10
        )

        # =========================================================
        # JOINT STATE
        # =========================================================

        self.target_joint_angles = [
            0.0
        ] * (self.num_sections * 2)

        self.current_joint_angles = [
            0.0
        ] * (self.num_sections * 2)

        # =========================================================
        # SMOOTHING
        # =========================================================

        self.alpha = 0.04

        # =========================================================
        # TIMER
        # =========================================================

        self.timer = self.create_timer(
            1.0 / 50.0,
            self.publish_joints
        )

        self.get_logger().info(
            'Cartesian octopus arm controller started'
        )

        self.get_logger().info(
            f'Arm length = {self.arm_length:.4f} m'
        )

        self.get_logger().info(
            'Publish geometry_msgs/Point to /target_position'
        )

        self.get_logger().info(
            'x, y, z are in metres'
        )

    # =============================================================
    # TARGET CALLBACK
    # =============================================================

    def target_callback(self, msg):

        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)

        self.get_logger().info(
            f'Target: x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}'
        )

        joints = self.calculate_ik(x, y, z)

        if joints is None:
            self.get_logger().warn(
                'Target is unreachable'
            )
            return

        self.target_joint_angles = joints

    # =============================================================
    # CONSTANT CURVATURE IK
    # =============================================================

    def calculate_ik(self, x, y, z):

        # ---------------------------------------------------------
        # Distance from base to target
        # ---------------------------------------------------------

        r = math.sqrt(
            x * x +
            y * y
        )

        d = math.sqrt(
            x * x +
            y * y +
            z * z
        )

        # ---------------------------------------------------------
        # Special case:
        # target approximately straight above base
        # ---------------------------------------------------------

        if r < 1e-6:

            # Straight arm
            if abs(z - self.arm_length) < 0.05:

                return [
                    0.0
                ] * (self.num_sections * 2)

        # ---------------------------------------------------------
        # Target must be inside arm length
        # ---------------------------------------------------------

        if d > self.arm_length:

            self.get_logger().warn(
                f'Target distance {d:.3f} m exceeds '
                f'arm length {self.arm_length:.3f} m'
            )

            return None

        # ---------------------------------------------------------
        # Target too close to base
        # ---------------------------------------------------------

        if d < 0.02:

            self.get_logger().warn(
                'Target is too close to the base'
            )

            return None

        # ---------------------------------------------------------
        # Direction of bending in XY plane
        # ---------------------------------------------------------

        phi = math.atan2(y, x)

        # ---------------------------------------------------------
        # Solve:
        #
        # d = 2 L sin(theta/2) / theta
        #
        # where:
        #
        # L     = arm length
        # theta = total bending angle
        #
        # theta = 0 means straight arm
        # ---------------------------------------------------------

        target_ratio = d / self.arm_length

        theta = self.solve_theta(target_ratio)

        if theta is None:

            self.get_logger().warn(
                'Could not find a valid curvature solution'
            )

            return None

        # ---------------------------------------------------------
        # Constant curvature
        #
        # Each section gets equal bending.
        # ---------------------------------------------------------

        bend_per_section = theta / self.num_sections

        # ---------------------------------------------------------
        # Convert bending direction into pitch/yaw.
        #
        # Current URDF:
        #
        # pitch axis = X
        # yaw axis   = Y
        #
        # We use:
        #
        # pitch = -bend * sin(phi)
        # yaw   =  bend * cos(phi)
        # ---------------------------------------------------------

        pitch = (
            -bend_per_section *
            math.sin(phi)
        )

        yaw = (
            bend_per_section *
            math.cos(phi)
        )

        # ---------------------------------------------------------
        # Joint limit
        # ---------------------------------------------------------

        pitch = max(
            -self.max_joint_angle,
            min(self.max_joint_angle, pitch)
        )

        yaw = max(
            -self.max_joint_angle,
            min(self.max_joint_angle, yaw)
        )

        # ---------------------------------------------------------
        # Generate 40 joint positions
        #
        # pitch_0
        # yaw_0
        # pitch_1
        # yaw_1
        # ...
        # pitch_19
        # yaw_19
        # ---------------------------------------------------------

        joints = []

        for _ in range(self.num_sections):

            joints.append(pitch)
            joints.append(yaw)

        self.get_logger().info(
            f'Bend angle = {math.degrees(theta):.2f} deg, '
            f'per section = {math.degrees(bend_per_section):.2f} deg'
        )

        return joints

    # =============================================================
    # SOLVE BENDING ANGLE
    # =============================================================

    def solve_theta(self, ratio):

        # Straight arm
        if ratio >= 0.999999:

            return 0.0

        # Physical lower limit of this simple model
        if ratio <= 0.05:

            self.get_logger().warn(
                'Target is too close for the constant-curvature model'
            )

            return None

        # ---------------------------------------------------------
        # Binary search for theta in:
        #
        # 0 < theta < pi
        # ---------------------------------------------------------

        low = 1e-8
        high = math.pi

        for _ in range(80):

            theta = (
                low +
                high
            ) / 2.0

            calculated_ratio = (
                2.0 *
                math.sin(theta / 2.0) /
                theta
            )

            # calculated_ratio decreases with theta
            if calculated_ratio > ratio:

                low = theta

            else:

                high = theta

        return (
            low +
            high
        ) / 2.0

    # =============================================================
    # SMOOTH JOINT MOTION
    # =============================================================

    def publish_joints(self):

        msg = Float64MultiArray()

        for i in range(
            len(self.current_joint_angles)
        ):

            error = (
                self.target_joint_angles[i] -
                self.current_joint_angles[i]
            )

            self.current_joint_angles[i] += (
                self.alpha *
                error
            )

            msg.data.append(
                self.current_joint_angles[i]
            )

        self.joint_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = GazeboCartesianArm()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
