#!/usr/bin/python3
from xfoil import XFoil
import matplotlib.pyplot as plt
xf = XFoil()

xf.load("revclarky.dat")
xf.Re = 1e5
xf.M = 0
xf.max_iter = 100
a, cl, cd, cm, cp = xf.aseq(-2,2,0.5)
plt.plot(a,cl)
plt.title("alfa vs cl")
plt.show()

#plt.plot(xf.airfoil.x,xf.airfoil.y)
#plt.axis('equal')
#plt.show()
