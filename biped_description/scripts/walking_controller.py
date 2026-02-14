#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class DynamicWalkingController(Node):

    def __init__(self):
        super().__init__('dynamic_walking_controller')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/biped_controller/joint_trajectory',
            10
        )

        self.sub = self.create_subscription(
            Imu,
            '/simple_biped/imu',
            self.imu_cb,
            10
        )

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        # IMU state
        self.roll = 0.0
        self.pitch = 0.0

        # Walking state
        self.phase_time = 0.0
        self.step_time = 1.0
        self.left_support = True

        self.get_logger().info("Dynamic walking controller started")

    # ---------------- IMU CALLBACK ----------------
    def imu_cb(self, msg):
        q = msg.orientation
        roll, pitch, _ = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.roll = roll
        self.pitch = pitch

    # ---------------- MAIN LOOP ----------------
    def update(self):
        dt = 0.02
        self.phase_time += dt

        if self.phase_time > self.step_time:
            self.phase_time = 0.0
            self.left_support = not self.left_support

        s = self.phase_time / self.step_time
        s = max(0.0, min(s, 1.0))
        swing = math.sin(math.pi * s)

        traj = JointTrajectory()
        traj.joint_names = [
            'L_hip',
            'L_knee',
            'L_ankle_pitch',
            'L_ankle_roll',
            'R_hip',
            'R_knee',
            'R_ankle_pitch',
            'R_ankle_roll',
        ]

        p = JointTrajectoryPoint()
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(dt * 1e9)

        # ---- IMU STABILIZATION ----
        ankle_roll_corr = -2.0 * self.roll
        ankle_pitch_corr = -1.5 * self.pitch

        if self.left_support:
            # LEFT SUPPORT, RIGHT SWING
            L_hip = -0.1
            L_knee = 0.15
            L_ankle_pitch = ankle_pitch_corr
            L_ankle_roll = ankle_roll_corr

            R_hip = 0.4 * swing
            R_knee = -0.8 * swing
            R_ankle_pitch = 0.4 * swing + ankle_pitch_corr
            R_ankle_roll = 0.0
        else:
            # RIGHT SUPPORT, LEFT SWING
            R_hip = -0.1
            R_knee = 0.15
            R_ankle_pitch = ankle_pitch_corr
            R_ankle_roll = ankle_roll_corr

            L_hip = 0.4 * swing
            L_knee = -0.8 * swing
            L_ankle_pitch = 0.4 * swing + ankle_pitch_corr
            L_ankle_roll = 0.0

        p.positions = [
            L_hip,
            L_knee,
            L_ankle_pitch,
            L_ankle_roll,
            R_hip,
            R_knee,
            R_ankle_pitch,
            R_ankle_roll,
        ]

        traj.points.append(p)
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = DynamicWalkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

