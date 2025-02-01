#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np
from scipy.interpolate import interp1d

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('csv_file', 'path_points.csv')  # Declare a parameter with a default value
        self.declare_parameter('flip_points', True)  # Declare a parameter to flip the order of points
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)  # Publish every second
        self.path = Path()
        self.path.header.frame_id = "map"
        # csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        csv_file = "/home/abdll/workspace/tamiyabot_ws/src/tamiyabot/tamiyabot_description/worlds/cone_track.csv"
        flip_points = self.get_parameter('flip_points').get_parameter_value().bool_value
        self.load_points_from_csv(csv_file, flip_points)

    def load_points_from_csv(self, csv_file, flip_points):
        points = []
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                points.append([x, y])

        points = np.array(points)

        if flip_points:
            points = np.flip(points, axis=0)  # Flip the order of the points

        # Generate equally spaced points
        equal_spaced_points = self.generate_equal_spaced_points(points, 0.3)

        # Convert points to ROS2 PoseStamped and add to the path
        for point in equal_spaced_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            self.path.poses.append(pose)

    def generate_equal_spaced_points(self, points, desired_distance=1.0):
        # Calculate the cumulative distance along the path
        distances = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)  # Insert a 0 at the beginning for the first point

        # Generate new distances at equal intervals
        num_points = int(distances[-1] / desired_distance)
        new_distances = np.linspace(0, distances[-1], num_points)

        # Interpolate to find new points at the equal intervals
        interp_func_x = interp1d(distances, points[:, 0], kind='linear')
        interp_func_y = interp1d(distances, points[:, 1], kind='linear')

        new_points = np.vstack((interp_func_x(new_distances), interp_func_y(new_distances))).T

        return new_points

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path)
        self.get_logger().info('Publishing Path with %d points' % len(self.path.poses))


def main(args=None):
    rclpy.init()
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()