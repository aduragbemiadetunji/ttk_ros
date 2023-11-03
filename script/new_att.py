#!/usr/bin/env python3
import time
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from mav_msgs.msg import Actuators
import math



ref_roll, ref_yaw, ref_pitch = (0, 0, 0)
Kp = 1.8
throttle = 200

actuators_msg = Actuators()

throt_val = 0

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

def pose_callback(pose_msg):
    # print(pose_msg)
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

    # print(pid_roll, pid_pitch, pid_yaw)
    # print(throt_val)
    # print(type(throt_val))
    throt_new = throttle + throt_val
    # print(throt_new)


    motor_one = throt_new - pid_roll + pid_yaw + pid_pitch
    motor_two = throt_new + pid_roll + pid_yaw - pid_pitch 
    motor_three = throt_new + pid_roll - pid_yaw + pid_pitch
    motor_four = throt_new - pid_roll - pid_yaw - pid_pitch

    # Example angular velocities values
    angular_velocities_values = [motor_one, motor_two, motor_three, motor_four]
    actuators_msg.angular_velocities = angular_velocities_values

    motor_publisher.publish(actuators_msg)

def throttle_callback(throt_msg):
    global throt_val
    throt_val = throt_msg.data





def subscriber():
    rospy.Subscriber('/throttle_increase', Float32, throttle_callback)
    rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, pose_callback)


if __name__ == "__main__":
    rospy.init_node("Attitude_Regulator")
    motor_publisher = rospy.Publisher("/rmf_obelix/command/motor_speed", Actuators, queue_size=10)
    subscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass