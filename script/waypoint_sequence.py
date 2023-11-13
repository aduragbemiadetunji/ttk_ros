#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion, Pose
from tf.transformations import euler_from_quaternion
import tf
import math


class WaypointSequencer:
    def __init__(self):
        self.waypoints = [
            {'x': 1.6339, 'y': 1.5046, 'z': 1.096, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            {'x': 0.0634, 'y': -0.0462, 'z': 0.5959, 'roll': 0.0, 'pitch': 0.0, 'yaw': -3*math.pi/8},  
            {'x': 1.3634, 'y': 1.0462, 'z': 1.3959, 'roll': 0.0, 'pitch': 0.0, 'yaw': -2.094},  
            {'x': 0.0634, 'y': -0.0462, 'z': 0.5959, 'roll': 0.0, 'pitch': 0.0, 'yaw': math.pi/2},  
            {'x': 0.0634, 'y': -0.0462, 'z': 0.5959, 'roll': 0.0, 'pitch': 0.0, 'yaw': -2.094},  
            {'x': 0.0149, 'y': -0.0173, 'z': -0.225, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0}  
        ]
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.2 #threshold for getting distance error

        self.traj_publisher = rospy.Publisher("/rmf_obelix/command/trajectory", MultiDOFJointTrajectory, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, self.pose_callback)
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg

        if self.current_pose:
            self.check_and_publish_waypoint()

    def check_and_publish_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]

            # Convert Euler angles (roll, pitch, yaw) to a quaternion for ROS
            quat = Quaternion(*euler_to_quaternion(waypoint['roll'], waypoint['pitch'], waypoint['yaw']))

            distance = self.calculate_distance(self.current_pose.position, waypoint)
            if distance < self.waypoint_threshold:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached.")
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.publish_waypoint(self.waypoints[self.current_waypoint_index])
            else:
                # If the waypoint has not been reached, continue publishing it
                self.publish_waypoint(waypoint)

    def calculate_distance(self, current_position, waypoint):
        dx = current_position.x - waypoint['x']
        dy = current_position.y - waypoint['y']
        dz = current_position.z - waypoint['z']
        dist = (dx**2 + dy**2 + dz**2)**0.5
        # print(dist)
        return dist

    def publish_waypoint(self, waypoint):
        quat = Quaternion(*euler_to_quaternion(waypoint['roll'], waypoint['pitch'], waypoint['yaw']))

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = 'base_link'
        traj_msg.joint_names = ['base_link']  # Modify with the actual joint name

        traj_point = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = waypoint['x']
        transform.translation.y = waypoint['y']
        transform.translation.z = waypoint['z']
        transform.rotation = quat
        traj_point.transforms = [transform]

        traj_msg.points.append(traj_point)
        self.traj_publisher.publish(traj_msg)
        rospy.loginfo(f"Published waypoint {self.current_waypoint_index}")

def euler_to_quaternion(roll, pitch, yaw):
    # Convert a roll, pitch, yaw to a quaternion
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return q

if __name__ == "__main__":
    rospy.init_node("waypoint_sequencer")
    sequencer = WaypointSequencer()

    # Wait for everything to be set up.
    rospy.sleep(1)

    # Start the sequence by publishing the first waypoint
    if sequencer.waypoints:
        sequencer.publish_waypoint(sequencer.waypoints[0])

    rospy.spin()
