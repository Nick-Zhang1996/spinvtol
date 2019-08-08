#!/usr/bin/python3
# reverse a wing for reverse flow analysis
# Nick Zhang


# works only with labeled dat file (1st line name, rest coordinates)
# data order: LE at (0,0), TE at (1,0), start from TE to LE (upper), then back to TE(lower)
# other representations, as used by UIUC airfoil database, follows different convention and require additional attention

import sys
import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin,pi

# ---------- Settings --------
if (len(sys.argv) != 2):
    print('usage: '+sys.argv[0] +' dat file to be reversed')
    exit(0)

filename = sys.argv[1]
output_filename = "rev"+filename

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
# verify
plt.plot(parsed[:,0],parsed[:,1])
plt.axis('equal')
plt.show()


# separate upper and lower
midpoint = np.argmin(parsed[:,0])
upper = parsed[:midpoint+1,:]
lower = parsed[midpoint:,:]

# reverse upper
upper[:,0] = 1-upper[:,0]
upper[:,:] = upper[::-1,:]

# reverse lower
lower[:,0] = 1-lower[:,0]
lower[:,:] = lower[::-1,:]

# stitch together
rev = np.vstack([upper,lower])

# verify
plt.plot(rev[:,0],rev[:,1])
plt.axis('equal')
plt.show()

f = open(output_filename,'w')
f.write(airfoil_name + " reversed\r\n")
rev = list(rev)
for entry in rev:
    f.write("%f %f\r\n" % (entry[0],entry[1]))

