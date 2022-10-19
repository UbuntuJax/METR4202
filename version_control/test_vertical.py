#Global Length Values
L1 = 100.0 #mm
L2 = 114.4 #mm                                                      
L3 = 93.3 #mm
L4 = 92.2 #mm

import numpy as np
from math import *

def inverse_kinematics(position):
    global pub
    
    # x = pose.position.x*1000 #m -> mm
    # y = pose.position.y*1000 #m -> mm
    # z = pose.position.z*1000 - L1

    x = position[0]*1000 #m -> mm
    y = position[1]*1000 #m -> mm
    z = position[2]*1000 - L1

    if invalid_pose(x,y,z):
        #rospy.loginfo(f'invalid pose')
        #break out of the function, return nothing
        return
    L5 = z + L4
    R = np.sqrt(x**2 + y**2)
    s = R
    Q = np.sqrt(s**2 + L5**2)
    #print(f"L5:", L5, ", s:", s)
    f = np.arctan2(L5, s)
    #print("f (radians):", f)
    cg = (L2**2+Q**2-L3**2) / (2*L2*Q)
    #print("cg:", cg)
    g_1 = atan2(np.sqrt(1-cg**2), cg)
    g_2 = atan2(-np.sqrt(1-cg**2), cg)
    #print(f"g(1)(rads):", g_1, ", g(2)(rads)", g_2)
    g = np.array([g_1, g_2])
    print("g:", g)
    alpha = g + f
    print("alpha:", alpha)
    cb = (L3**2 + L2**2 - Q**2) / (2*L2*L3)
    #print("cb:", cb) yep
    b_1 = atan2(np.sqrt(1-cb**2), cb)
    b_2 = atan2(-np.sqrt(1-cb**2), cb)

    print(f"b(1)(rads):", b_1, ", b(2)(rads)", b_2)
    b = np.array([b_1, b_2])
    c = -b-alpha+2*np.pi
    print("c:", c)
    
    theta1 = np.arctan2(y, x)
    theta2 = alpha - np.pi/2 
    theta3 = -np.pi + b 
    theta4 = -np.pi*(3/4) + c

    print("calculated angles:\ntheta1:", np.degrees(theta1), "theta2:", np.degrees(theta2), "theta3:", np.degrees(theta3), "theta4:", np.degrees(theta4), "\n")

    #choose which angle to use
    theta2final,theta3final,theta4final = angle_check(theta1, theta2, theta3, theta4)
    
    #hardcoding angles
    # theta2final,theta3final,theta4final = [0,0,0]
    
    if(hitting_ground(theta2final, theta3final, theta4final)):
        #break out of the function, return nothing
        return

    print(f'theta1: {np.degrees(theta1)}, theta2: {np.degrees(theta2final)}, theta3: {np.degrees(theta3final)}, \
        theta4: {np.degrees(theta4final)}')

    #rospy.loginfo(f'published angles:\ntheta1: {np.degrees(theta1)}\n theta2: {np.degrees(theta2final)}\n theta3: {np.degrees(theta3final)}\n theta4: {np.degrees(theta4final)}')
    #pub.publish(create_message(theta1,theta2final,theta3final,theta4final))

def invalid_pose(x, y, z):
    x_workspace = L2 + L3 + L4
    y_workspace = L2 + L3 + L4
    z_workspace = L2 + L3 + L4

    if np.abs(x) > x_workspace or np.abs(y) > y_workspace or np.abs(z) > z_workspace:
        return True
    else:
        return False

def angle_check(theta1, theta2, theta3, theta4):
    """
    Takes in angles produced by inverse kinematics solution and filters the output while 
    checking that given solutions are achievable by the dynamixel
    Note: function returns nothing on failure, allowing for checks like...
    if !anglecheck(...):
        print(error has occured...)
    """
    thetalist = np.array([theta2,theta3,theta4])
    thetafinallist = np.array([0.0, 0.0, 0.0])
    firstsolutioninvalid = 0

    #adjusting angles that are out of range for the dynamixel
    for k in range(3):
        for a in range(2):
            if k == 2:
                critvalue = [110, -110]
            else:
                critvalue = [145, -145]
            
            if thetalist[k,a] > np.radians(critvalue[0]):
                print(f'{np.degrees(thetalist[k,a])} is more than {critvalue[0]}')
                thetalist[k,a] -= 2*np.pi

            elif thetalist[k,a] < np.radians(critvalue[1]):
                print(f'{np.degrees(thetalist[k,a])} is more than {critvalue[1]}')
                thetalist[k,a] += 2*np.pi

    if theta1 > np.radians(145) or theta1 < np.radians(-145):
        print(f'invalid theta1')
        return

    #checking if either solution is valid (if first solution is valid, does not check the second solution)
    for i in range(3):
        if i == 2:
            critvalue = [110, -110]
        else:
            critvalue = [145, -145]

        if thetalist[i,0] < np.radians(critvalue[0]) and thetalist[i,0] > np.radians(critvalue[1]):
            print("theta[0] works")
            thetafinallist[i] = thetalist[i,0]
            #print(thetafinallist)
        else:
            print("theta[0] doesnt work")
            print(np.degrees(thetalist[i,0]))
            firstsolutioninvalid = 1
            break

    if firstsolutioninvalid:
        for j in range(3):
            if j == 2:
                critvalue = [110, -110]
            else:
                critvalue = [140, -140]
            if thetalist[j,1] < np.radians(critvalue[0]) and thetalist[j,1] > np.radians(critvalue[1]):
                print("theta[1] works")
                thetafinallist[j] = thetalist[j,1]
                #print(thetafinallist)
            else:
                print("theta[1] doesnt work")
                print(np.degrees(thetalist[j-2,1]))
                return

    theta2 = thetafinallist[0]
    theta3 = thetafinallist[1]
    theta4 = thetafinallist[2]
    print("here")
    print(f'theta2: {np.degrees(theta2)}\ntheta3: {np.degrees(theta3)}\ntheta4: {np.degrees(theta4)}')
    return theta2, theta3, theta4

def hitting_ground(theta2, theta3, theta4):
    z_end_effector = L2*np.sin(theta2) + L3*np.sin(theta3) + L4*np.sin(theta4)
    z_theta4 = L2*np.sin(theta2) + L3*np.sin(theta3)
    z_theta3 = L2*np.sin(theta2)
    if z_end_effector <= -L1 or z_theta4 <= -L1 or z_theta3 <= -L1:
        #rospy.loginfo(f'Hitting Ground')
        return True
    else:
        return False


def hitting_luggage_wall(x, y, z, theta2, theta3, theta4):
    z_end_effector = L2*np.sin(theta2) + L3*np.sin(theta3) + L4*np.sin(theta4)
    z_theta4 = L2*np.sin(theta2) + L3*np.sin(theta3)
    z_theta3 = L2*np.sin(theta2)

    y_theta4 = L2*cos(theta2) + L3*cos(theta3)
    y_theta3 = L2*cos(theta2)

    if (x < 100 and x > -100 and y > 57.2 and y < 62.2 and z < 50) or (z_theta3 < 50 and y_theta3 > 57.2 and y_theta3 < 62.2) or \
    (z_theta4 < 50 and y_theta4 > 57.2 and y_theta4 < 62.2): #the area to the luggage_wall
        return True
    else:
        return False





if __name__ == "__main__":
    position = np.array([0.1, 0, 0.1])
    inverse_kinematics(position)