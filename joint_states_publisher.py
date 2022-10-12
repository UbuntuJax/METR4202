#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import random

# Always need this
import rospy
from numpy import *

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    x = pose.position[0]
    y = pose.position[1]
    z = pose.position[2] - L1
    L1 = ...
    L2 = 1
    L3 = 1
    L4 = 1
    L5 = z
    R = sqrt(x**2 + y**2)
    L6 = R - L4
    L7 = sqrt(L6**2 + L5**2)
    theta5 = arctan2(L5, L6)
    ctheta2 = (L2**2+L7**2-L3**2) / (2*L2*L7)
    theta2_1 = arctan2(ctheta2, sqrt(1-ctheta2))
    theta2_2 = arctan2(ctheta2, -sqrt(1-ctheta2))
    theta2 = matrix([theta2_1, theta2_2])
    alpha = theta2 + theta5
    ctheta3 = (L3**2 + L2**2 - L7**2) / (2*L2*L3)
    theta3_1 = arctan2(ctheta3, sqrt(1-ctheta3))
    theta3_2 = arctan2(ctheta3, -sqrt(1-ctheta3))
    theta3 = matrix([theta3_1, theta3_2])
    theta4 = -theta3-alpha+2*pi
    theta1 = arctan2(x, y)

    theta1offset = theta1
    theta2offset = theta2 + pi/2
    theta3offset = theta3
    theta4offset = theta4

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(create_message(theta1,theta2,theta3,theta4))


# Funny code
def create_message(theta1,theta2,theta3,theta4) -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    
    msg.position = [
        theta1,theta2,theta3,theta4
    ]
    return msg


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()