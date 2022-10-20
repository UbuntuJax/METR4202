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
