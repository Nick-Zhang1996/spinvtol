#!/usr/bin/python3
# provides some sanity check routine for monocopter design
import numpy as np
import scipy.integrate as integrate
from math import pi,sqrt

g = 9.81
# air density
rho = 1.2
# kinematic viscosity
nu = 1.48E-5


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
    return 0.16*2

# from paper that use light sensor for state estimate
def optical_cord(r):
    if r>0.2 and r<0.6:
        return 0.35
    else:
        return 0.2

# fly-by-wire project paper
def wire_cord(r):
    return 0.2-0.25*r

def analyze(omega,m,cord, R):
# find required lift coeff for given parameters
    C_l = m*g/(0.5*rho*omega**2*integrate.quad(lambda r:r**2*cord(r),0,R)[0])
# Renold #
    Re = omega*R*cord(R/2.0) / nu
# theoretical power required form momentum theory
    P = sqrt((m*g)**3/(2*rho*pi*R**2))
# Disk loading
    DL = m*g/(pi*R**2)
# Hover efficiency
    eta = m/P
# Tip acceleration
    acc = omega**2*R/g
# Rotor solidity
    rs = integrate.quad(cord, 0, R)[0]/pi/R**2
    print("---- Analysis case ----")
    print("-- input --")
    print("rev/s =" + "%.2f"%(omega/2/pi) + " m= " + "%.2f"%(m) + " R=" + "%.2f"%(R))
    print("-- result --")
    print("C_l = " + "%.2f"%(C_l))
    print("Mid span Re = %.0f"%Re)
    print("Disk Loading = " + "%.2f"%(DL))
    print("Power(min) = "+"%.2f"%(P))
    print("Hover efficiency (g/w) = "+"%.2f"%(eta*1000)+" ("+"%.3f"%(1/(eta*1000)) +" w/g)")
    print("Tip acceleration(g) = " + "%.2f"%(acc))
    print("Tip vel(kph) = " + "%.2f"%(omega*R*3.6))
    print("Solidity = %.2f"%rs)
    print("")
    return 

# from paper Pitch and Heave ..... Evan, Journal of Aircraft
analyze(80.5,0.075,samara_cord,0.27)

# from photo sensor paper
analyze(16.6,1.2,optical_cord,1.0)
#
# THOR
#analyze(38,0.532,thor_cord,0.5)

## analyze(omega,m,cord, R):

# fly-by-wire control project
analyze(40,0.175,wire_cord,0.4)

def my_cord(r):
    return 0.19

# original, no controller
analyze(2*pi*5,0.33,my_cord, 0.45) 

# add ctrl sys, no wing ext,  barely take off
#analyze(2*pi*5,0.43,my_cord, 0.45) 

# add ext, tips over
#analyze(2*pi*2.5,0.468,my_cord, 0.8) 

# 55cm wing, barely takes off with full throttle and 7 deg installation angle
#analyze(3.3*pi*2,0.468,my_cord, 0.5) 

# 55cm wing, good power, 3s lipo, and 7 deg installation angle
analyze(3.85*pi*2,0.468+0.052,my_cord, 0.55) 

# 55cm wing, good power, 3s lipo, and 5 deg installation angle
analyze(5*pi*2,0.468+0.052,my_cord, 0.55) 

# 55cm wing, good power, 3s lipo, and 3 deg installation angle
analyze(6.25*pi*2,0.468+0.052,my_cord, 0.55) 
