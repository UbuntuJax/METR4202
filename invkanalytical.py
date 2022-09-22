import numpy as np
from sympy import *

def main():
    # Robot Arm length
    l1 = 1
    l2 = 1
    l3 = 1
    l4 = 1

    theta1 = Symbol('theta1',real = True)
    theta2 = Symbol('theta2',real = True)
    theta3 = Symbol('theta3',real = True)
    theta4 = pi/2

    #trigonometric equations

    e1= Eq(cos(theta1)*(l2*sin(theta2)+l3*sin(theta2+theta3)+l4*sin(theta2+theta3+theta4)), 0.5)
    e2= Eq(sin(theta1)*(l2*sin(theta2)+l3*sin(theta2+theta3)+l4*sin(theta2+theta3+theta4)), 0.5)
    e3= Eq(l2*sin(theta2)+l3*sin(theta2+theta3)+l4*sin(theta2+theta3+theta4), 0.5)

    solve([e1,e2,e3],theta1,theta2,theta3)

    #x1 = degrees(x1)
    #x2 = degrees(x2)
    #x3 = degrees(x3)

    print("degree values : ",theta1,theta2,theta3)

    #return 1

if __name__ == "__main__":
    main()

