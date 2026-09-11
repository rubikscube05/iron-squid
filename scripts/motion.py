#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class ComplexArmMotion(Node):

    def __init__(self):
        super().__init__('complex_arm_motion')

        # =========================================================
        # PARAMETERS
        # =========================================================

        # Maximum tendon displacement in mm
        self.amplitude = 12.0

        # Motion frequency
        self.frequency = 0.08

        # Publish frequency
        self.publish_rate = 50.0

        # Current motion time
        self.t = 0.0

        # =========================================================
        # MOTION MODE
        #
        # 0 = Circle
        # 1 = Figure 8
        # 2 = Wave
        # 3 = Spiral
        # 4 = Complex
        # 5 = Smooth random
        # =========================================================

        self.mode = 0

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.servo_pub = self.create_publisher(
            Float64MultiArray,
            '/servo_commands',
            10
        )

        # =========================================================
        # TIMER
        # =========================================================

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            'Complex arm motion controller started'
        )

        self.get_logger().info(
            'Publishing 3-servo commands on /servo_commands'
        )

        self.get_logger().info(
            'Mode 0: Circle'
        )

        self.get_logger().info(
            'Mode 1: Figure 8'
        )

        self.get_logger().info(
            'Mode 2: Wave'
        )

        self.get_logger().info(
            'Mode 3: Spiral'
        )

        self.get_logger().info(
            'Mode 4: Complex'
        )

        self.get_logger().info(
            'Mode 5: Smooth random'
        )

    # =========================================================
    # MAIN MOTION FUNCTION
    # =========================================================

    def update(self):

        self.t += 1.0 / self.publish_rate

        if self.mode == 0:
            servo = self.circle_motion()

        elif self.mode == 1:
            servo = self.figure_eight()

        elif self.mode == 2:
            servo = self.wave_motion()

        elif self.mode == 3:
            servo = self.spiral_motion()

        elif self.mode == 4:
            servo = self.complex_motion()

        elif self.mode == 5:
            servo = self.smooth_random_motion()

        else:
            servo = [0.0, 0.0, 0.0]

        self.publish_servo(servo)

    # =========================================================
    # MODE 0
    #
    # CIRCULAR BENDING
    # =========================================================

    def circle_motion(self):

        w = 2.0 * math.pi * self.frequency

        phase = w * self.t

        # Three phase-shifted tendon commands
        s1 = self.amplitude * math.sin(phase)

        s2 = self.amplitude * math.sin(
            phase - 2.0 * math.pi / 3.0
        )

        s3 = self.amplitude * math.sin(
            phase - 4.0 * math.pi / 3.0
        )

        return [s1, s2, s3]

    # =========================================================
    # MODE 1
    #
    # FIGURE 8
    # =========================================================

    def figure_eight(self):

        w = 2.0 * math.pi * self.frequency

        phase = w * self.t

        # Two-frequency combination creates a figure-8
        x = math.sin(phase)
        y = math.sin(2.0 * phase)

        # Convert bending vector into tendon commands
        s1 = self.amplitude * x

        s2 = self.amplitude * (
            -0.5 * x +
            0.8660254 * y
        )

        s3 = self.amplitude * (
            -0.5 * x -
            0.8660254 * y
        )

        return [s1, s2, s3]

    # =========================================================
    # MODE 2
    #
    # TRAVELING WAVE
    # =========================================================

    def wave_motion(self):

        w = 2.0 * math.pi * self.frequency

        phase = w * self.t

        s1 = self.amplitude * math.sin(phase)

        s2 = self.amplitude * math.sin(
            phase + 2.0 * math.pi / 3.0
        )

        s3 = self.amplitude * math.sin(
            phase + 4.0 * math.pi / 3.0
        )

        return [s1, s2, s3]

    # =========================================================
    # MODE 3
    #
    # SPIRAL
    # =========================================================

    def spiral_motion(self):

        w = 2.0 * math.pi * self.frequency

        phase = w * self.t

        # Slowly changing amplitude
        envelope = (
            0.5 +
            0.5 * math.sin(
                0.15 * phase
            )
        )

        amplitude = self.amplitude * envelope

        s1 = amplitude * math.sin(phase)

        s2 = amplitude * math.sin(
            phase - 2.0 * math.pi / 3.0
        )

        s3 = amplitude * math.sin(
            phase - 4.0 * math.pi / 3.0
        )

        return [s1, s2, s3]

    # =========================================================
    # MODE 4
    #
    # COMPLEX COMBINED MOTION
    # =========================================================

    def complex_motion(self):

        w = 2.0 * math.pi * self.frequency

        p = w * self.t

        # Multiple frequencies create complicated
        # but still smooth motion.

        s1 = self.amplitude * (
            0.55 * math.sin(p)
            + 0.25 * math.sin(2.3 * p)
            + 0.15 * math.sin(4.7 * p)
        )

        s2 = self.amplitude * (
            0.55 * math.sin(
                p + 2.0 * math.pi / 3.0
            )
            + 0.25 * math.sin(
                2.3 * p + 1.0
            )
            + 0.15 * math.sin(
                4.7 * p + 2.0
            )
        )

        s3 = self.amplitude * (
            0.55 * math.sin(
                p + 4.0 * math.pi / 3.0
            )
            + 0.25 * math.sin(
                2.3 * p + 2.0
            )
            + 0.15 * math.sin(
                4.7 * p + 4.0
            )
        )

        return [s1, s2, s3]

    # =========================================================
    # MODE 5
    #
    # SMOOTH RANDOM-LIKE MOTION
    # =========================================================

    def smooth_random_motion(self):

        p = self.t

        s1 = self.amplitude * (
            0.50 * math.sin(0.37 * p)
            + 0.30 * math.sin(0.83 * p + 1.2)
            + 0.15 * math.sin(1.71 * p)
        )

        s2 = self.amplitude * (
            0.50 * math.sin(0.43 * p + 2.0)
            + 0.30 * math.sin(0.71 * p)
            + 0.15 * math.sin(1.43 * p + 2.5)
        )

        s3 = self.amplitude * (
            0.50 * math.sin(0.31 * p + 4.0)
            + 0.30 * math.sin(0.91 * p + 1.5)
            + 0.15 * math.sin(1.57 * p + 3.0)
        )

        return [s1, s2, s3]

    # =========================================================
    # PUBLISH
    # =========================================================

    def publish_servo(self, values):

        msg = Float64MultiArray()

        msg.data = [
            float(values[0]),
            float(values[1]),
            float(values[2])
        ]

        self.servo_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ComplexArmMotion()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
