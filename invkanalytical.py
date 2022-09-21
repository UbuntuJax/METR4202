import numpy as np
import math
from sympy import *

def main():
    x = Symbol('x')
    a = limit(sin(x)/x,x,0)
    print(a)

    #return 1

#if __name__ == "main":
main()

