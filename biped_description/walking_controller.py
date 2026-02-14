import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class SimpleWalker(Node):

    def __init__(self):
        super().__init__('simple_walker')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/biped_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'L_hip_roll',
            'L_hip_pitch',
            'L_knee',
            'L_ankle_roll',
            'L_ankle_pitch',
            'R_hip_roll',
            'R_hip_pitch',
            'R_knee',
            'R_ankle_roll',
            'R_ankle_pitch'
        ]

        time.sleep(2)
        self.walk()

    def send_pose(self, positions, duration_sec):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start = Duration(sec=int(duration_sec))

        msg.points.append(point)
        self.publisher.publish(msg)

    def walk(self):

        while rclpy.ok():

            # ==============================
            # STEP 1 — LEFT LEG SWING
            # ==============================

            self.send_pose([
                0.0,      # L_hip_roll
                0.2,      # L_hip_pitch forward
                -0.8,     # L_knee bend
                0.0,
                0.3,      # L_ankle lift
                0.0,
                0.0,
                -0.2,     # R_knee slight bend support
                0.0,
                0.0
            ], 2)

            time.sleep(2.5)

            # ==============================
            # STEP 2 — RETURN LEFT
            # ==============================

            self.send_pose([
                0.0,
                0.0,
                -0.3,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.3,
                0.0,
                0.0
            ], 2)

            time.sleep(2.5)

            # ==============================
            # STEP 3 — RIGHT LEG SWING
            # ==============================

            self.send_pose([
                0.0,
                0.0,
                -0.2,
                0.0,
                0.0,
                0.0,
                0.2,
                -0.8,
                0.0,
                0.3
            ], 2)

            time.sleep(2.5)

            # ==============================
            # STEP 4 — RETURN RIGHT
            # ==============================

            self.send_pose([
                0.0,
                0.0,
                -0.3,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.3,
                0.0,
                0.0
            ], 2)

            time.sleep(2.5)


def main():
    rclpy.init()
    node = SimpleWalker()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

