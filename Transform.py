import numpy as np
# Trot = np.array([
    #     [0, -1, 0, (Pcam_box[0] - Pcam_base[0])],
    #     [-1, 0, 0, (Pcam_box[1] - Pcam_base[1])],
    #     [0, 0, -1, (Pcam_box[2] - Pcam_base[2])],
    #     [0, 0, 0, 1],
    # ])

def main(Pcam_box):
    """
    Takes some numpy array pbox and finds the distance from the base of the 
    robot to the cube in the robot's frame of reference 
    """
    Pcam_base = np.array([0.0, -0.19, -0.25]) #m --> mm in the camera's base frame
    Pbase_box = (Pcam_base - Pcam_box) #distance from base to box in the camera's reference frame
    Translation = Rotation(Pbase_box[0], Pbase_box[1], Pbase_box[2]) #distance from base to box in the robot's reference frame
    print(f'Translation: {Translation}')
    distance = np.array([Translation[0, 3], Translation[1, 3], Translation[2, 3]])
    return distance



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

if __name__ == "__main__":
    print(f'distance: {main(np.array([0.1, 0.2, -0.15]))}')
