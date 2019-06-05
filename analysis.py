# provides some sanity check routine for monocopter design
import numpy as np
import scipy.integrate as integrate

g = 9.81
# air density
rho = 1.2

def fun_cord(r):
# design function, give cord length as a function of r
    if r < 0.17:
        return 0.02
    else:
        return 0.057

def cl(omega,m, R):
# find required lift coeff for given parameters
    C_l = m*g/(0.5*rho*omega**2*integrate.quad(lambda r:r**2*fun_cord(r),0,R)[0])
    return C_l

print(cl(80.5,0.075,0.27))



