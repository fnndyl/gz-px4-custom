import math
import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

def quat2eul(q):
        """
        One would think there would be a library for this sigh
        """
        w = q[3]
        x = q[0]
        y = q[1]
        z = q[2]

        roll = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z)
        pitch = math.asin(-2.0*(x*z - w*y))
        yaw = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)

        return [roll, pitch, yaw]

def eul2quat(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def quaternion_multiply(q1, q2):
    """Quaternion multiplication: q = q1 * q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w])

def slerp(q0, q1, t):
    """Spherical linear interpolation between two quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t*(q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0*q0 + s1*q1

class BargeControllerNode(Node):
    def __init__(self):

        super().__init__('barge_controller_node')

        # QoS Policy
        qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.origin_x = 0
        self.origin_y = 0
        self.origin_z = 2

        # Create an odometry publisher to publish trajectories to the platform in Gazebo
        self.odometry_publisher = self.create_publisher(Odometry, "/barge_cmd", qos)

        # Load the position data
        df = pd.read_csv("baseline.csv")
        self.timestamps = df['unix_time_offset'].to_numpy()
        self.positions = df[['x_norm', 'y_norm', 'z']].to_numpy()
        self.orientations = df[['qx', 'qy', 'qz', 'qw']].to_numpy()

        # Apply 90 yaw rotation offset
        theta = np.pi  # 90 degrees
        q_offset = np.array([0.0, 0.0, np.sin(theta/2), np.cos(theta/2)])
        # q_offset = np.array([0.0, 0.0, 0.0, 1.0])

        for i in range(len(self.orientations)):
            rpy = quat2eul(self.orientations[i])
            rpy[2] = -rpy[2]

            quat = np.array(eul2quat(rpy[0], rpy[1], rpy[2]))

            self.orientations[i] = quat
            self.orientations[i] = quaternion_multiply(q_offset, self.orientations[i])

        #self.start_wall_time = self.get_clock().now().nanoseconds / 10e9
        self.start_wall_time = 0.0
        self.start_data_time = self.timestamps[0] # Can select other parts of data to start with

        # Create timer to handle publishing of setpoints to Gazebo
        # 10 Hz seems reasonable
        self.timer = self.create_timer(1.0/10.0, self.timer_callback)

    def timer_callback(self):
        
        #elapsed_time = (self.get_clock().now().nanoseconds / 10e9) - self.start_wall_time
        self.start_wall_time = self.start_wall_time + 0.10
        target_time = self.start_data_time + self.start_wall_time

        idx = np.searchsorted(self.timestamps, target_time)
        
        if idx == 0:
            pos = self.positions[0]
        
        else:
            t0 = self.timestamps[idx - 1]
            t1 = self.timestamps[idx]
            alpha = (target_time - t0) / (t1 - t0)

            pos0 = self.positions[idx - 1]
            pos1 = self.positions[idx]
            pos = pos0 + alpha * (pos1 - pos0)

            q0 = self.orientations[idx - 1]
            q1 = self.orientations[idx]
            ori = slerp(q0, q1, alpha)

        print(f"{self.start_wall_time:.1f}, {pos}")

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(x=pos[1], y=pos[0], z=pos[2])
        msg.pose.pose.orientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])

        self.odometry_publisher.publish(msg)

def main(args=None):
    # Define and spin up ROS2 node
    rclpy.init(args=args)
    barge_controller = BargeControllerNode()
    rclpy.spin(barge_controller)

    # Explicitly kill ROS2 node on exit
    barge_controller.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
