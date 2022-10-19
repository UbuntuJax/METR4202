#!/usr/bin/python3

import rospy
import tf
import time
import numpy as np
import  modern_robotics as mr
import transformations as tr
# Comment these when not working on Raspberry Pie
import rospy
from geometry_msgs.msg import Pose

# constants for position of camera in the frame of robot into position matrix
x_rc = 0.0
y_rc = -0.19
z_rc = -0.25
pos_rc = np.array([[x_rc],[y_rc],[z_rc]])

#Create rotational matrix. x_r = -y_c || y_r = x_c || z_r = z_c                  
rot_rc = np.array([[0.0, -1.0, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0]])    
                   
# Transformation matrix from robot to camera
T_cr = mr.RpToTrans(rot_rc, pos_rc)

# TEST
q_block = np.array([1.0, 0.0, 0.0, 0.0])
x_block = 1.0
y_block = 1.0
z_block = 1.0
"""
q_block = np.array([data.transforms[0].transform.rotation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z, data.transforms[0].transform.translation.w])
x_block = data.transforms[0].transform.translation.x
y_block = data.transforms[0].transform.translation.y
z_block = data.transforms[0].transform.translation.z
"""


def main():
    pub_pose = rospy.Publisher('pose', Pose, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    
    
    T = rob2block_transform(x_block, y_block, z_block, q_block)
    print(T)
    
    try:
        publish(T)
    except rospy:
        pass


def rob2block_transform(x_cb, y_cb, z_cb, q_cb): #Input values for the camera to block translation and rotation
    
    rot_cb = quat2rot(q_cb)
    print(rot_cb)
    rot_cb = tr.quaternion_matrix(q_cb)[0: 3, 0:3]  # <-- This may work better, tr likes to give back 4x4 matrices instead so only need top left 3x3
    print(rot_cb)
    
    # Create position matric of camera to block
    pos_cb = np.array([x_cb,y_cb,z_cb])
    # Create the transformation matrix of camera to block
    T_cb = mr.RpToTrans(rot_cb, pos_cb)
    #Transform of to camera robot
    T_rb = mr.TransInv(T_cr) @ T_cb
    print(T_rb)

    q_rb = tr.quaternion_from_matrix(mr.RpToTrans(T_rb[0: 3, 0:3], np.array([0, 0, 0])))
    pos_rb = T_rb[0: 3, 3]
    #print(q_rb)

    publish_pose(pos_rb, q_rb)
    
    return

def quat2rot(quart):
    """
    NOT MY CODE.
    This is taken from the following but editted.
    https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    # Extract the values from Q
    q1 = quart[0]
    q2 = quart[1]
    q3 = quart[2]
    q4 = quart[3]
     
    # First row of the rotation matrix
    r11 = 2 * (q1 * q1 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q1 * q4)
    r13 = 2 * (q2 * q4 + q1 * q3)
     
    # Second row of the rotation matrix
    r21 = 2 * (q2 * q3 + q1 * q4)
    r22 = 2 * (q1 * q1 + q3 * q3) - 1
    r23 = 2 * (q3 * q4 - q1 * q2)
     
    # Third row of the rotation matrix
    r31 = 2 * (q2 * q4 - q1 * q3)
    r32 = 2 * (q3 * q4 + q1 * q2)
    r33 = 2 * (q1 * q1 + q4 * q4) - 1
     
    # 3x3 rotation matrix
    rot_mat = np.array([[r11, r12, r13],
                           [r21, r22, r23],
                           [r31, r32, r33]])

    return rot_mat

    
def publish_pose(pos, quat):
    """
    NOT MY CODE.
    This is taken from the following but editted.
    https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/
    """
    
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        p = Pose()
        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]
        # Make sure the quaternion is valid and normalized
        p.orientation.x = quat[0]
        p.orientation.y = quat[1]
        p.orientation.z = quat[2]
        p.orientation.w = quat[3]
        #pub_pose.publish(p)
        rospy.loginfo(p)
        rate.sleep()


if __name__ == '__main__':
    main()