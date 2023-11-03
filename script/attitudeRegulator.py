#!/usr/bin/env python3
# import rospy

# from geometry_msgs.msg import Pose
# from mav_msgs.msg import Actuators


# def attitude_callback(msg):
#     # att_msg = Pose()
#     pos_x = msg.position.x
#     # rospy.loginfo(pos_x)


# def speed_callback(msg):
#     # att_msg = Pose()
#     pos_x = msg.angular_velocities[0]
#     rospy.loginfo(pos_x)

# def send_thrust_values():
#     comm = Actuators()
#     comm.angular_velocities = [200, 0, 0, 0]
#     # comm.angular_velocities[1] = 200
#     # comm.angular_velocities[2] = 200
#     # comm.angular_velocities[3] = 200

#     command_publisher.publish(comm)
#     rospy.loginfo(comm)


# if __name__ == "__main__":
#     rospy.init_node("AttitudeRegulator")
#     rospy.Subscriber("/rmf_obelix/odometry_sensor1/pose", Pose, attitude_callback)
#     # rospy.Subscriber("/rmf_obelix/command/motor_speed", Actuators, speed_callback)
#     command_publisher = rospy.Publisher("/rmf_obelix/command/motor_speed", Actuators, queue_size=10)
#     send_thrust_values()

#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         pass



# import rospy
# from mav_msgs.msg import Actuators
# import time

# # Initialize the ROS node
# rospy.init_node('actuators_publisher', anonymous=True)

# # Create a message object
# actuators_msg = Actuators()

# # Example angular velocities values
# angular_velocities_values = [388.64, 388.64, 388.64, 388.64]  # Replace with your desired values

# # Assign the values to the angular_velocities field
# actuators_msg.angular_velocities = angular_velocities_values

# # Publish the message to a ROS topic
# pub = rospy.Publisher('/rmf_obelix/command/motor_speed', Actuators, queue_size=10)
# rate = rospy.Rate(10)  # 10 Hz
# while not rospy.is_shutdown():
#     pub.publish(actuators_msg)
#     rate.sleep()



import rospy
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Pose  # Import the appropriate message type PoseStamped
import time
import math


ref_roll, ref_yaw, ref_pitch = (0, 0, 0)
Kp = 0.5
throttle = 300

# global motor_one, motor_two, motor_three, motor_four

# motor_one, motor_two, motor_three, motor_four = (0, 0, 0, 0)

# Callback function to handle the received pose message
def pose_callback(pose_msg):
    global motor_one, motor_two, motor_three, motor_four
    # Extract and print the position and orientation components of the pose
    position = pose_msg.position
    orientation = pose_msg.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    # print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    # print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
    roll, pitch, yaw = quaternion_to_euler(*quaternion)
    # print(roll, pitch, yaw)


    err_roll, err_yaw, err_pitch = (ref_roll-roll, ref_yaw-yaw, ref_pitch-pitch)

    pid_roll = Kp * err_roll
    pid_yaw = Kp * err_yaw
    pid_pitch = Kp * err_pitch


    motor_one = throttle - pid_roll + pid_yaw + pid_pitch
    motor_two = throttle + pid_roll + pid_yaw - pid_pitch
    motor_three = throttle + pid_roll - pid_yaw + pid_pitch
    motor_four = throttle - pid_roll - pid_yaw - pid_pitch




# Define a function to convert a quaternion to Euler angles
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# # Example quaternion values
# quaternion = (0.1825742, 0.3651484, 0.5477226, 0.7302967)  # Replace with your quaternion values

# # Convert the quaternion to Euler angles
# roll, pitch, yaw = quaternion_to_euler(*quaternion)

# Initialize the ROS node
rospy.init_node('actuators_publisher', anonymous=True)

# Create a subscriber to the /rmf_obelix/ground_truth/pose topic
rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, pose_callback)

# Create a message object for the actuators
actuators_msg = Actuators()
print(motor_one, motor_two, motor_three, motor_four)


# Example angular velocities values
angular_velocities_values = [motor_one, motor_two, motor_three, motor_four]  # Replace with your desired values

# Assign the values to the angular_velocities field
actuators_msg.angular_velocities = angular_velocities_values

# Publish the message to a ROS topic
pub = rospy.Publisher('/rmf_obelix/command/motor_speed', Actuators, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    pub.publish(actuators_msg)
    rate.sleep()

# Spin to keep the node alive and handle callbacks
rospy.spin()