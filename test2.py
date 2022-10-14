import numpy as np
def angle_check(theta1, theta2, theta3, theta4):
    #check invalid
    if theta1 > np.radians(280) or theta1 < np.radians(20):
        print(f'invalid theta1')
        return

    #checks vald
    thetalist = np.array([theta2,theta3,theta4])
    thetafinallist = np.array([0.0, 0.0, 0.0])
    for i in range(3):
        for a in range(2):
            print(thetalist[i,a])
            if thetalist[i,a] < np.radians(280) and thetalist[i,a] > np.radians(20):
                print("here")
                thetafinallist[i] = thetalist[i,a]
                print(thetafinallist)
                break

    print(len(thetafinallist))
    if len(thetafinallist) != 3:
        print(f'invalid angle')
        return

    print(theta2, theta3, theta4)
    theta2 = thetafinallist[0]
    theta3 = thetafinallist[1]
    theta4 = thetafinallist[2]
    return theta2, theta3, theta4

if __name__ == "__main__":
    theta1 = np.radians(45)
    theta2 = np.array([np.radians(500), np.radians(80)])
    theta3 = np.array([np.radians(10), np.radians(200)])
    theta4 = np.array([np.radians(60), np.radians(300)])
    
    theta2, theta3, theta4 = angle_check(theta1, theta2, theta3, theta4)
    print(theta2, theta3, theta4)