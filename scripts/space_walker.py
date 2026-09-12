#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class SpaceWalker(Node):

    def __init__(self):
        super().__init__('space_walker')

        # ============================================================
        # MOTION PARAMETERS
        # ============================================================

        self.control_frequency = 30.0

        # Circular stroke frequency
        self.stroke_frequency = 1.0

        # Circular motion radius
        # 60 mm radius = 120 mm diameter
        self.circle_radius_mm = 60.0

        # Maximum tendon command
        self.max_pull_mm = 80.0

        # Overall movement gain
        self.translation_gain = 1.0

        # Rotation gain
        self.rotation_gain = 0.8

        # Command deadband
        self.deadband = 0.01

        # Internal phase
        self.phase = 0.0

        # ============================================================
        # COMMAND
        # ============================================================

        self.cmd_linear = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }

        self.cmd_angular = {
            'x': 0.0,   # Roll
            'y': 0.0,   # Pitch
            'z': 0.0    # Yaw
        }

        # ============================================================
        # ROS INTERFACES
        # ============================================================

        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10
        )

        self.servo_pub = self.create_publisher(
            Float64MultiArray,
            '/servo_commands',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.update_motion
        )

        self.get_logger().info(
            'Space Walker started'
        )

        self.get_logger().info(
            '30 Hz | 1 Hz stroke | 60 mm radius'
        )

    # ================================================================
    # CMD_VEL CALLBACK
    # ================================================================

    def vel_callback(self, msg):

        self.cmd_linear['x'] = msg.linear.x
        self.cmd_linear['y'] = msg.linear.y
        self.cmd_linear['z'] = msg.linear.z

        self.cmd_angular['x'] = msg.angular.x
        self.cmd_angular['y'] = msg.angular.y
        self.cmd_angular['z'] = msg.angular.z

    # ================================================================
    # LIMIT
    # ================================================================

    def limit(self, value):

        return max(
            -self.max_pull_mm,
            min(self.max_pull_mm, value)
        )

    # ================================================================
    # ARM IK
    #
    # bend_up:
    #     Local vertical bending
    #
    # bend_left:
    #     Local sideways bending
    #
    # Converts bending into 3 tendon commands.
    # ================================================================

    def arm_ik(self, bend_up, bend_left):

        s1 = bend_up

        s2 = (
            -0.5 * bend_up
            + 0.8660254 * bend_left
        )

        s3 = (
            -0.5 * bend_up
            - 0.8660254 * bend_left
        )

        return [
            self.limit(s1),
            self.limit(s2),
            self.limit(s3)
        ]

    # ================================================================
    # CIRCULAR MOTION
    # ================================================================

    def circular_motion(
        self,
        phase,
        direction=1.0,
        amplitude=1.0
    ):

        up = (
            math.cos(phase * direction)
            * self.circle_radius_mm
            * amplitude
        )

        left = (
            math.sin(phase * direction)
            * self.circle_radius_mm
            * amplitude
        )

        return up, left

    # ================================================================
    # UPDATE MOTION
    # ================================================================

    def update_motion(self):

        dt = 1.0 / self.control_frequency

        # ============================================================
        # CHECK IF ROBOT IS MOVING
        # ============================================================

        moving = (
            abs(self.cmd_linear['x']) > self.deadband
            or
            abs(self.cmd_linear['y']) > self.deadband
            or
            abs(self.cmd_linear['z']) > self.deadband
            or
            abs(self.cmd_angular['x']) > self.deadband
            or
            abs(self.cmd_angular['y']) > self.deadband
            or
            abs(self.cmd_angular['z']) > self.deadband
        )

        # ============================================================
        # UPDATE PHASE
        # ============================================================

        if moving:

            self.phase += (
                2.0
                * math.pi
                * self.stroke_frequency
                * dt
            )

            if self.phase >= 2.0 * math.pi:
                self.phase -= 2.0 * math.pi

        else:

            self.phase = 0.0

        # ============================================================
        # INITIAL ARM COMMANDS
        # ============================================================

        # RED
        a1_up = 0.0
        a1_left = 0.0

        # GREEN
        a2_up = 0.0
        a2_left = 0.0

        # YELLOW
        a3_up = 0.0
        a3_left = 0.0

        # ============================================================
        # X TRANSLATION
        #
        # RED:
        #     ONLY UP/DOWN
        #
        # GREEN:
        #     Circular motion
        #
        # YELLOW:
        #     Opposite circular motion
        #
        # X direction is inverted because your current robot
        # physically moves opposite to the command.
        # ============================================================

        x_cmd = self.cmd_linear['x']

        if abs(x_cmd) > self.deadband:

            magnitude = min(abs(x_cmd), 1.0)

            # IMPORTANT:
            #
            # +X command -> reverse gait
            # -X command -> normal gait
            #
            direction = (
                -1.0
                if x_cmd > 0.0
                else 1.0
            )

            # --------------------------------------------------------
            # RED ARM
            #
            # ONLY UP/DOWN
            #
            # No circular motion.
            # No left/right bending.
            # --------------------------------------------------------

            r_up = (
                math.sin(self.phase * direction)
                * self.circle_radius_mm
                * magnitude
            )

            a1_up += r_up

            # Explicitly force red sideways bending to zero
            a1_left = 0.0

            # --------------------------------------------------------
            # GREEN ARM
            #
            # Circular paddle
            # --------------------------------------------------------

            g_up, g_left = self.circular_motion(
                self.phase,
                direction,
                magnitude
            )

            a2_up += g_up
            a2_left += g_left

            # --------------------------------------------------------
            # YELLOW ARM
            #
            # Opposite circular paddle
            # --------------------------------------------------------

            y_up, y_left = self.circular_motion(
                self.phase,
                -direction,
                magnitude
            )

            a3_up += y_up
            a3_left += y_left

        # ============================================================
        # Y TRANSLATION
        #
        # RED + GREEN active.
        # They rotate in opposite directions.
        #
        # YELLOW stays neutral.
        # ============================================================

        y_cmd = self.cmd_linear['y']

        if abs(y_cmd) > self.deadband:

            magnitude = min(abs(y_cmd), 1.0)

            direction = (
                1.0
                if y_cmd > 0.0
                else -1.0
            )

            # --------------------------------------------------------
            # RED
            # --------------------------------------------------------

            r_up, r_left = self.circular_motion(
                self.phase,
                direction,
                magnitude
            )

            # --------------------------------------------------------
            # GREEN
            # Opposite rotation
            # --------------------------------------------------------

            g_up, g_left = self.circular_motion(
                self.phase,
                -direction,
                magnitude
            )

            a1_up += r_up
            a1_left += r_left

            a2_up += g_up
            a2_left += g_left

        # ============================================================
        # Z TRANSLATION
        #
        # All three arms cooperate.
        #
        # 120 degree phase separation.
        # ============================================================

        z_cmd = self.cmd_linear['z']

        if abs(z_cmd) > self.deadband:

            magnitude = min(abs(z_cmd), 1.0)

            direction = (
                1.0
                if z_cmd > 0.0
                else -1.0
            )

            # RED
            z1_up, z1_left = self.circular_motion(
                self.phase,
                direction,
                magnitude
            )

            # GREEN
            z2_up, z2_left = self.circular_motion(
                self.phase + 2.0 * math.pi / 3.0,
                direction,
                magnitude
            )

            # YELLOW
            z3_up, z3_left = self.circular_motion(
                self.phase + 4.0 * math.pi / 3.0,
                direction,
                magnitude
            )

            a1_up += z1_up
            a1_left += z1_left

            a2_up += z2_up
            a2_left += z2_left

            a3_up += z3_up
            a3_left += z3_left

        # ============================================================
        # YAW ROTATION
        #
        # Differential motion around the body.
        # ============================================================

        yaw_cmd = self.cmd_angular['z']

        if abs(yaw_cmd) > self.deadband:

            magnitude = (
                min(abs(yaw_cmd), 1.0)
                * self.rotation_gain
            )

            direction = (
                1.0
                if yaw_cmd > 0.0
                else -1.0
            )

            # RED
            r_up, r_left = self.circular_motion(
                self.phase,
                direction,
                magnitude
            )

            # GREEN opposite
            g_up, g_left = self.circular_motion(
                self.phase,
                -direction,
                magnitude
            )

            # YELLOW follows RED
            y_up, y_left = self.circular_motion(
                self.phase,
                direction,
                magnitude
            )

            a1_up += r_up
            a1_left += r_left

            a2_up += g_up
            a2_left += g_left

            a3_up += y_up
            a3_left += y_left

        # ============================================================
        # ROLL
        #
        # GREEN and YELLOW move opposite.
        # ============================================================

        roll_cmd = self.cmd_angular['x']

        if abs(roll_cmd) > self.deadband:

            magnitude = (
                min(abs(roll_cmd), 1.0)
                * self.rotation_gain
            )

            direction = (
                1.0
                if roll_cmd > 0.0
                else -1.0
            )

            roll_motion = (
                math.sin(self.phase * direction)
                * self.circle_radius_mm
                * magnitude
            )

            a2_up += roll_motion
            a3_up -= roll_motion

        # ============================================================
        # PITCH
        #
        # RED + GREEN move together.
        # YELLOW moves opposite.
        # ============================================================

        pitch_cmd = self.cmd_angular['y']

        if abs(pitch_cmd) > self.deadband:

            magnitude = (
                min(abs(pitch_cmd), 1.0)
                * self.rotation_gain
            )

            direction = (
                1.0
                if pitch_cmd > 0.0
                else -1.0
            )

            pitch_motion = (
                math.sin(self.phase * direction)
                * self.circle_radius_mm
                * magnitude
            )

            a1_up += pitch_motion
            a2_up += pitch_motion
            a3_up -= pitch_motion

        # ============================================================
        # IMPORTANT ARM CONSTRAINT
        #
        # If X translation is commanded, RED must ONLY move
        # up/down.
        #
        # This prevents other gait components from accidentally
        # adding left/right motion to the red arm.
        # ============================================================

        if abs(x_cmd) > self.deadband:

            a1_left = 0.0

        # ============================================================
        # GLOBAL TRANSLATION GAIN
        # ============================================================

        a1_up *= self.translation_gain
        a1_left *= self.translation_gain

        a2_up *= self.translation_gain
        a2_left *= self.translation_gain

        a3_up *= self.translation_gain
        a3_left *= self.translation_gain

        # ============================================================
        # ARM IK
        # ============================================================

        arm1 = self.arm_ik(
            a1_up,
            a1_left
        )

        arm2 = self.arm_ik(
            a2_up,
            a2_left
        )

        arm3 = self.arm_ik(
            a3_up,
            a3_left
        )

        # ============================================================
        # 9 SERVO COMMANDS
        #
        # RED:
        #   servo 1, 2, 3
        #
        # GREEN:
        #   servo 4, 5, 6
        #
        # YELLOW:
        #   servo 7, 8, 9
        # ============================================================

        servos = (
            arm1
            + arm2
            + arm3
        )

        msg = Float64MultiArray()
        msg.data = servos

        self.servo_pub.publish(msg)

    # ================================================================
    # STOP ROBOT
    # ================================================================

    def stop_robot(self):

        neutral = Float64MultiArray()

        neutral.data = [0.0] * 9

        self.servo_pub.publish(neutral)


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = SpaceWalker()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
