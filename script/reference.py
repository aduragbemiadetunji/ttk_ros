#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Transform
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_matrix, quaternion_matrix
from nav_msgs.msg import Odometry

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint



# PID gains
Kpos = np.diag([1.0, 1.0, 1.0])  # Position control gains
Kvel = np.diag([0.5, 0.5, 0.5])  # Velocity control gains

#Quadrotor mass
mass = 0.5  # kg
gravity = 9.81  # m/s^2

# Reference heading angle (yaw)
psi_ref = 0.0  # Assuming a constant reference yaw for simplicity

# Desired state
p_ref = np.array([1.0, 2.0, 3.0])  # Reference position (x, y, z)
v_ref = np.array([0.0, 0.0, 0.0])  # Reference velocity (vx, vy, vz)
a_ref = np.array([0.0, 0.0, 0.0])  # Reference acceleration (ax, ay, az)

# Global variable to store the current orientation
current_orientation = None
ades = None

# Initialize ROS node
rospy.init_node('reference_generator') #quadrotor_pid_controller

# Publisher for thrust and attitude commands
thrust_pub = rospy.Publisher('/quadrotor/thrust_command', Float64, queue_size=10)
attitude_pub = rospy.Publisher('/quadrotor/attitude_command', Quaternion, queue_size=10)
trajectory_pub = rospy.Publisher('/rmf_obelix/command/trajectory', MultiDOFJointTrajectory, queue_size=10)


def compute_desired_orientation(ades):
    zB_des = ades / np.linalg.norm(ades)
    xC_des = np.array([np.cos(psi_ref), np.sin(psi_ref), 0.0])
    yB_des = np.cross(zB_des, xC_des)
    yB_des /= np.linalg.norm(yB_des)  # Normalize
    xB_des = np.cross(yB_des, zB_des)
    R_des = np.vstack((xB_des, yB_des, zB_des)).T
    return R_des

def compute_thrust(ades, R):
    zB = R[:, 2]
    thrust = mass * (np.dot(ades, zB) + gravity)
    return thrust

def position_callback(msg):
    global p_ref, v_ref, a_ref, current_orientation, ades
    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    p_error = p - p_ref
    v_error = np.zeros(3)  # Assuming v is zero for simplicity
    ades = -np.dot(Kpos, p_error) - np.dot(Kvel, v_error) + a_ref - np.array([0, 0, gravity])
    R_des = compute_desired_orientation(ades)
    R_4x4 = np.eye(4)
    R_4x4[:3, :3] = R_des
    q_des = quaternion_from_matrix(R_4x4)

    # q_des = quaternion_from_matrix(R_des)
    # q_des = quaternion_from_euler(*euler_from_quaternion(R_des))

    # Create and populate the trajectory message
    trajectory_msg = MultiDOFJointTrajectory()
    point = MultiDOFJointTrajectoryPoint()
    transform = Transform()

    # Set the translation to the reference position
    transform.translation.x = p_ref[0]
    transform.translation.y = p_ref[1]
    transform.translation.z = p_ref[2]

    # Set the rotation to the desired quaternion
    transform.rotation = Quaternion(*q_des)

    # Populate the MultiDOFJointTrajectoryPoint
    point.transforms.append(transform)
    point.time_from_start = rospy.Duration(1)  # Set an appropriate duration
    trajectory_msg.points.append(point)

    # Publish the trajectory message
    trajectory_pub.publish(trajectory_msg)


    # Extract the quaternion
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    # Convert the quaternion to a rotation matrix
    current_orientation = quaternion_matrix(quaternion)[:3, :3]

    q_msg = Quaternion(*q_des)
    attitude_pub.publish(q_msg)
    # publishTrajectory(p_ref, quaternion)



def velocity_callback(msg):
    global ades, current_orientation
    v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    # print(v)

    if current_orientation is not None and ades is not None:
        # Use the current orientation to compute thrust
        thrust = compute_thrust(ades, current_orientation)
        thrust_msg = Float64()
        thrust_msg.data = thrust
        thrust_pub.publish(thrust_msg)
    # else:
    #     rospy.logwarn("Current orientation not available.")


rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, position_callback)
rospy.Subscriber('/rmf_obelix/ground_truth/odometry', Odometry, velocity_callback)
# rospy.Subscriber('/quadrotor/imu', Imu, orientation_callback)

# Spin ROS node
rospy.spin()
