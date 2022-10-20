from sympy import *
from numpy import *

def main(x,y,z):
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

    theta1 = theta1 * 180/pi
    theta2 = theta2 * 180/pi
    theta3 = theta3 * 180/pi
    theta4 = theta4 * 180/pi
    
    print(theta1, theta2, theta3, theta4)


if __name__ == "__main__":
    main(100,100,100)