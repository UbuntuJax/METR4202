import numpy as np

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
                critvalue = [145, -145]
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


if __name__ == "__main__":
    theta1 = np.radians(100)
    theta2 = np.array([np.radians(55), np.radians(-130)])
    theta3 = np.array([np.radians(150), np.radians(-250)])
    theta4 = np.array([np.radians(55), np.radians(-250)])
    angle_check(theta1, theta2, theta3, theta4)