#!/usr/bin/env python3
#import rospy
import numpy as np
from numpy.linalg import inv

def main(Tsd):
    #M matrix (Tsb)?? how do we compute this?
    Tsb = np.matrix([[1,0,0,0.22],[0,1,0,0],[0,0,1,0.419],[0,0,0,1]])
    
    #initial guess
    theta0 = 0.1 #radians
    invTsb = inv(Tsb)
    TsbTsd = np.matmul(invTsb, Tsd) #matrix multiplication
    print("inv(Tsb)*Tsd: ")
    print(TsbTsd)
    Vb = np.log(TsbTsd) #computing screw axis


#desired configuration
Tsd = np.matrix([[-1,0,0,-0.5],[0,-1,0,0.1],[0,0,-1,0.1],[0,0,0,1]])
if __name__ == '__main__':
    main(Tsd)