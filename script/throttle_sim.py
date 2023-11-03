#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import sys, select, termios, tty

# Initialize the ROS node
rospy.init_node('throttle_sim')

# Define the publisher for the topic
pub = rospy.Publisher('throttle_increase', Float32, queue_size=10)

# Set the rate at which the values are published (in Hz)
rate = rospy.Rate(10)  # 10 Hz

# Define the initial value
value = 0

# Function to get the key press
def get_key():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main loop to publish values based on key presses
if __name__=="__main__":
    try:
        print("Publishing value updates. Press 'i' to increase value, 'd' to decrease value, and 'q' to quit.")
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'i':
                value += 10
            elif key == 'd':
                value -= 10
            elif key == 'q':
                break
            else:
                pass

            pub.publish(value)
            rate.sleep()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
