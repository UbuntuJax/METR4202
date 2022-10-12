from numpy import *
from matplotlib.pyplot import *
from math import *

def main(x,y,z):
    x = x
    y = y
    L1 = 100
    z = z - L1
    L2 = 114.4
    L3 = 93.3
    L4 = 92.2
    L5 = z
    R = sqrt(x**2 + y**2)
    s = R - L4
    Q = sqrt(s**2 + L5**2)
    #print(f"L5:", L5, ", s:", s)
    f = arctan2(L5, s)
    #print("f (radians):", f)
    cg = (L2**2+Q**2-L3**2) / (2*L2*Q)
    #print("cg:", cg)
    g_1 = arctan2(cg, sqrt(1-cg))
    g_2 = arctan2(cg, -sqrt(1-cg))
    #print(f"g(1)(rads):", g_1, ", g(2)(rads)", g_2)
    g = matrix([g_1, g_2])
    print("g:", g)
    alpha = g + f
    print("alpha:", alpha)
    cb = (L3**2 + L2**2 - Q**2) / (2*L2*L3)
    #print("cb:", cb) yep
    b_1 = arccos((L3**2 + L2**2 - Q**2)/(2*L2*L3))
    b_2 = b_1 + pi/4

    #old b1
    #b_1 = arctan2(cb, sqrt(1-cb))
    #b_2 = arctan2(cb, -sqrt(1-cb))

    print(f"b(1)(rads):", b_1, ", b(2)(rads)", b_2)
    b = matrix([b_1, b_2])
    c = -b-alpha+2*pi
    print("c:", c)
    
    theta1 = arctan2(y, x)
    theta2 = alpha.item(0) - pi/2 #choosing first soln
    theta3 = pi - b.item(0) #choosing first soln
    theta4 = pi - c.item(0)

    print("theta1:", theta1, "theta2:", theta2, "theta3:", theta3, "theta4:", theta4)

    theta1dynamixeloffset = 150 * pi / 180 + theta1 #moves anticlockwise
    theta2dynamixeloffset = 150 * pi / 180 + theta2 #moves clockwise
    theta3dynamixeloffset = 150 * pi / 180 + theta3 #moves clockwise
    theta4dynamixeloffset = 150 * pi / 180 + theta4 #moves clockwise

    print("theta1dyna:", theta1dynamixeloffset, "theta2dyna:", theta2dynamixeloffset, \
        "theta3dyna:", theta3dynamixeloffset, "theta4dyna:", theta4dynamixeloffset)

    theta1deg = degrees(theta1dynamixeloffset) #moves anticlockwise
    theta2deg = degrees(theta2dynamixeloffset) #moves clockwise
    theta3deg = degrees(theta3dynamixeloffset) #moves clockwise
    theta4deg = degrees(theta4dynamixeloffset) #moves clockwise

    print("theta1deg:", theta1deg, "theta2deg:", theta2deg, \
        "theta3deg:", theta3deg, "theta4deg:", theta4deg)

    #x = [0, L2*cos(theta2), L2*cos(theta2)+L3*cos(theta3), \
        #L2*cos(theta2)+L3*cos(theta3)+L4*cos(theta4)]

    #y = [0, L2*sin(theta2), L2*sin(theta2)+L3*sin(theta3), \
        #L2*sin(theta2)+L3*sin(theta3)+L4*sin(theta4)]

    #print("x:", x, "y:", y)
    #plot(x,y)
    #show()

if __name__ == "__main__":
    main(100,100,200)