# provides some sanity check routine for monocopter design
import numpy as np
import scipy.integrate as integrate
from math import pi

g = 9.81
# air density
rho = 1.2

def fun_cord(r):
# design function, give cord length as a function of r
    if r < 0.4:
        return 0.15
    else:
        return 0.0

# from paper Pitch and Heave ..... Evan, Journal of Aircraft
def samara_cord(r):
# design function, give cord length as a function of r
    if r < 0.17:
        return 0.02
    else:
        return 0.057
# THOR, not exactly a monocopter
def thor_cord(r):
    return 0.16

# from paper that use light sensor for state estimate
def optical_cord(r):
    if r>0.2 and r<0.6:
        return 0.35
    else:
        return 0.2

# fly-by-wire project paper
def wire_cord(r):
    return 0.2-0.25*r

def cl(omega,m,cord, R):
# find required lift coeff for given parameters
    C_l = m*g/(0.5*rho*omega**2*integrate.quad(lambda r:r**2*cord(r),0,R)[0])
    return C_l

# from paper Pitch and Heave ..... Evan, Journal of Aircraft
#print(cl(80.5,0.075,samara_cord,0.27))

# from photo sensor paper
#print(cl(16.6,1.2,optical_cord,1.0))

# THOR
#print(cl(38,0.532,thor_cord,0.5)/2)

#print(cl(80,0.7,fun_cord,0.4))

# fly-by-wire control project
print(cl(40,0.175,wire_cord,0.4))



