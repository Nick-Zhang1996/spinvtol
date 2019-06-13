#add circular hollow section in xfoil airfoil dat file for foamcut_gui
# Nick Zhang


# works only with labeled dat file (1st line name, rest coordinates)
# data order: LE at (0,0), TE at (1,0), start from TE to LE (upper), then back to TE(lower)
# other representations, as used by UIUC airfoil database, follows different convention and require additional attention

import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin,pi

# ---------- Settings --------
filename = "clarky.dat"
output_filename = "clarky16mm.dat"

# unit cm, doesn't really matter so long we stay consistent
cord = 20.0

# note we do not check if the circle is within the airfoil profile
# verify with plot
# (x,y,R), in converted coordinate system
circle = (0.25*cord,0.5,0.8)

# ---------- End Settings --------



#1st line is airfoil name
f = open(filename, 'r')
data = f.read().splitlines()
f.close()
airfoil_name = data[0]
datapoints = data[1:]
parsed = []

for line in datapoints:
    parsed.append([float(val) for val in line.split()])

parsed = np.array(parsed)

# roughly distance between two adjacent points, default seem to be too coarse
# but too fine may lead to problems for formcutter's control
step = 0.005 * cord
parsed *= cord


angular_step = step/circle[2]
circle_points = []

# generate datapoints for the circular hole
for angle in np.linspace(1.5*pi,-0.5*pi,2*pi/angular_step + 1):
    circle_points.append([cos(angle),sin(angle)])
circle_points = np.array(circle_points)
circle_points[:,0] += circle[0]
circle_points[:,1] += circle[1]

# determine location to bottom of the circle
insert_index = np.searchsorted(parsed[:,0],circle_points[0,0])
start_point = np.array([circle[0],parsed[insert_index-1,1]])
parsed = np.vstack([parsed[:insert_index,:],start_point,circle_points,start_point,parsed[insert_index:,:]])



#plt.plot(circle_points[:,0],circle_points[:,1],'o')

plt.plot(parsed[:,0],parsed[:,1])
plt.axis('equal')
plt.show()

parsed /= cord
f = open(output_filename,'w')
f.write(airfoil_name + " with circle\r\n")
parsed = list(parsed)
for entry in parsed:
    f.write("%f %f\r\n" % (entry[0],entry[1]))

