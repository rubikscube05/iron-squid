#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class OctopusTendonController(Node):

    def __init__(self):
        super().__init__('octopus_tendon_controller')

        # ========================================================
        # ROBOT PARAMETERS
        # ========================================================
        self.num_sections = 20
        self.tendon_radius_mm = 25.0
        self.max_bend_angle = math.radians(100.0)

        # ========================================================
        # SERVO COMMAND INPUT
        # ========================================================
        self.servo_sub = self.create_subscription(
            Float64MultiArray,
            '/servo_commands',
            self.servo_callback,
            10
        )

        # ========================================================
        # JOINT COMMAND OUTPUTS (Matches bot.yaml)
        # ========================================================
        self.pub_arm1 = self.create_publisher(Float64MultiArray, '/arm1_controller/commands', 10)
        self.pub_arm2 = self.create_publisher(Float64MultiArray, '/arm2_controller/commands', 10)
        self.pub_arm3 = self.create_publisher(Float64MultiArray, '/arm3_controller/commands', 10)

        # Current servo states: [Arm1[S1,S2,S3], Arm2[S1,S2,S3], Arm3[S1,S2,S3]]
        self.servo_values = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]

        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_commands)

        self.get_logger().info('Octopus 9-servo tendon controller started')
        self.get_logger().info('Input: /servo_commands [A1_S1..S3, A2_S1..S3, A3_S1..S3]')

    def servo_callback(self, msg):
        if len(msg.data) != 9:
            self.get_logger().error(f'Expected 9 servo values, got {len(msg.data)}')
            return

        self.servo_values[0] = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
        self.servo_values[1] = [float(msg.data[3]), float(msg.data[4]), float(msg.data[5])]
        self.servo_values[2] = [float(msg.data[6]), float(msg.data[7]), float(msg.data[8])]

    def calculate_joint_positions(self, servo_set):
        s1, s2, s3 = servo_set

        # Remove common displacement (equal pulling causes no bend)
        mean_pull = (s1 + s2 + s3) / 3.0
        p1 = s1 - mean_pull
        p2 = s2 - mean_pull
        p3 = s3 - mean_pull

        r = self.tendon_radius_mm
        if r <= 0.0:
            return [0.0] * (self.num_sections * 2)

        # Constant curvature kinematics calculation
        bend_x = (2.0 * p1 - p2 - p3) / (3.0 * r)
        bend_y = (p2 - p3) / (math.sqrt(3.0) * r)

        # Limit total bending safety cap
        bend_magnitude = math.sqrt(bend_x**2 + bend_y**2)
        if bend_magnitude > self.max_bend_angle:
            scale = self.max_bend_angle / bend_magnitude
            bend_x *= scale
            bend_y *= scale

        # Distribute equally across 20 segments
        pitch_per_section = bend_x / self.num_sections
        yaw_per_section = bend_y / self.num_sections

        joint_positions = []
        for _ in range(self.num_sections):
            joint_positions.append(pitch_per_section)
            joint_positions.append(yaw_per_section)

        return joint_positions

    def publish_joint_commands(self):
        msg1 = Float64MultiArray(data=self.calculate_joint_positions(self.servo_values[0]))
        msg2 = Float64MultiArray(data=self.calculate_joint_positions(self.servo_values[1]))
        msg3 = Float64MultiArray(data=self.calculate_joint_positions(self.servo_values[2]))

        self.pub_arm1.publish(msg1)
        self.pub_arm2.publish(msg2)
        self.pub_arm3.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    node = OctopusTendonController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
