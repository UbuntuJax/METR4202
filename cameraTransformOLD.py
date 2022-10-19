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
"""x_rc = 0.0
y_rc = -0.19
z_rc = -0.25
pos_rc = np.array([[x_rc],[y_rc],[z_rc]])


#Create rotational matrix. x_r = -y_c || y_r = x_c || z_r = z_c                  
rot_rc = np.array([[0.0, -1.0, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0]])    
                   
# Transformation matrix from robot to camera
T_cr = mr.RpToTrans(rot_rc, pos_rc)"""

# TEST
"""
q_block = np.array([1.0, 0.0, 0.0, 0.0])
x_block = 1.0
y_block = 1.0
z_block = 1.0"""



def main(rotation, translation):
    q_block = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
    x_block = translation.x
    y_block = translation.y
    z_block = translation.z
    #T = rob2block_transform(x_block, y_block, z_block, q_block)
    T = get_pos(np.array([x_block, y_block, z_block]))

    Pcam_base = np.array([0.0, -0.19, -0.44])

def get_pos(Pcam_box):
    """
    Takes some numpy array pbox and finds the distance from the base of the 
    robot to the cube in the robot's frame of reference 
    """
    Pcam_base = np.array([0.0, -0.19, -0.44]) #m --> mm in the camera's base frame
    
    Pbase_box = (Pcam_base - Pcam_box) #distance from base to box in the camera's reference frame
    T_rb = Rotation(Pbase_box[0], Pbase_box[1], Pbase_box[2]) #distance from base to box in the robot's reference frame
    #print(f'Translation: {Translation}')
    distance = np.array([T_rb[0, 3], T_rb[1, 3], T_rb[2, 3]])
    
    
    q_rb = np.array([0,0,0,0]) #tr.quaternion_from_matrix(mr.RpToTrans(T_rb[0: 3, 0:3], np.array([0, 0, 0])))
    pos_rb = T_rb[0: 3, 3]
    try:
        publish_pose(pos_rb, q_rb)
    except rospy:
        pass

def Rotation(x,y,z):
    T_camera = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])
    T_camera_robot = np.array([
        [0, -1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    product = T_camera_robot @ T_camera
    return product


def rob2block_transform(x_cb, y_cb, z_cb, q_cb): #Input values for the camera to block translation and rotation
    
    #rot_cb = quat2rot(q_cb)
    #print(rot_cb)
    rot_cb = tr.quaternion_matrix(q_cb)[0: 3, 0:3]  # <-- This may work better, tr likes to give back 4x4 matrices instead so only need top left 3x3
    #print(rot_cb)
    
    # Create position matric of camera to block
    pos_cb = np.array([x_cb,y_cb,z_cb])
    # Create the transformation matrix of camera to block
    T_cb = mr.RpToTrans(rot_cb, pos_cb)
    #Transform of to camera robot
    T_rb = mr.TransInv(T_cr) @ T_cb
    #rint(T_rb)

    q_rb = tr.quaternion_from_matrix(mr.RpToTrans(T_rb[0: 3, 0:3], np.array([0, 0, 0])))
    pos_rb = T_rb[0: 3, 3]
    #print(q_rb)
    try:
        publish_pose(pos_rb, q_rb)
    except rospy:
        pass
 

    
def publish_pose(pos, quat):
    """
    NOT MY CODE.
    This is taken from the following but editted.
    https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/
    """
    
    rate = rospy.Rate(2) # Hz
    
    p = Pose()
    p.position.x = pos[0]
    p.position.y = pos[1]
    p.position.z = pos[2]
    # Make sure the quaternion is valid and normalized
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    pub_pose.publish(p)
    rospy.loginfo(p)
    rate.sleep()



from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray


def fiducial_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.transforms[0].transform.translation)
    print("hi")
    main(data.transforms[0].transform.rotation, data.transforms[0].transform.translation)


def listener():

    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        pub_pose = rospy.Publisher('pose', Pose, queue_size=1)
        rospy.init_node('pose_publisher', anonymous=True)
        listener()
    
     







# class RobotVision():
#     """
#     A class to represent the detection of blocks using a camera
#     """
#     def __init__(self):
#         """
#         Constructs all the necessary attributes for ComputeIk object
#         """    
#         # rospy.init_node("scara_cv", anonymous=True)
#         # self.block_transform_pub = rospy.Publisher("block_transform", FiducialTransform, queue_size=1)
#         # self.find_colour_pub = rospy.Publisher("id", Int32, queue_size=1)
#         # self.scara_home_sub = rospy.Subscriber("/scara_home", Bool, self.ready_callback, queue_size=1)
#         self.fid_transform_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback, queue_size=1)
#         # self.colour_sub = rospy.Subscriber("/block_colour", String, self.colour_callback, queue_size=1)
#         # self.listner = tf.TransformListener()
#         # self.tf_msg = FiducialTransform()
#         # self.rate = rospy.Rate(1)

#         # self.Tbase_fiducial = "42"
#         # self.SCARA_ARM_RADIUS = 220e-3 
#         self.fiducial_transforms = None
#         # self.ready_to_pickup = False
#         self.read_cv_data = True

#         # self.prev_transform = None
#         # self.prev_fid_id = None
#         # self.same_pos_counter = 0
#         # self.colour = None
#         # self.colour_map = {"red": 0, "green": 1, "blue": 2, "yellow": 3}
#         pass

#     def fiducial_callback(self, data):
#         """
#         Function that is called when data is published to /fiducial_transforms.
#         The data contains the transformations of visible fiducials.
#         Args:
#             data (fiducial_msgs/FiducialTransformArray): the visible fiducial transformations
#         """        
#         # Remove base fiducial from data 
#         # as this is not a valid block
#         for (i, _tf) in enumerate(data.transforms):
#             if _tf.fiducial_id == self.Tbase_fiducial:
#                 del data.transforms[i]
#         if self.read_cv_data and len(data.transforms) > 1:
#             # Now, check a visible fiducial
#             self.fiducial_transforms = data.transforms
#             current_id = data.transforms[0].fiducial_id
#             current_transform = data.transforms[0]

#             # Check if prev fiducial id is still in frame
#             if current_id != self.prev_fid_id:
#                 self.ready_to_pickup = False
#                 self.prev_fid_id = current_id
#                 self.prev_transform = current_transform
#                 return
            
#             self.ready_to_pickup = self.is_moving(current_transform)
#             self.prev_fid_id = data.transforms[0].fiducial_id
#             self.prev_fid_id = data.transforms[0]
#         else:
#             # Reset the values for next callback
#             self.prev_transform = None
#             self.prev_fid_id = None
#             self.same_pos_counter = 0
#             self.fiducial_transforms = data.transforms
#         self.rate.sleep()

#     if __name__ == "__main__":
#         try:
#             rv = RobotVision()
#             rv.run()
#         except rospy.ROSInterruptException:
#             print("An error occured running the Robot vision node.")
