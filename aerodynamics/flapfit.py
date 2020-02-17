# fit aerodynamic coefficients of a deflecting flap
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# prepare data
filenames = ['up5','0','down5','down10','down15','down20','down25','down30']

# in deg
aoa = np.linspace(-5,10,16)
# down positive, in deg
flap = np.linspace(-5,30,8)
aoam,flapm = np.meshgrid(aoa,flap)
# cl[flap(seq),aoa(seq)]
cl = np.zeros_like(aoam)
cd = np.zeros_like(aoam)
cm = np.zeros_like(aoam)

for flapIndex in range(8):
    filename = filenames[flapIndex]
    handle = open('flapdata/'+filename+'.txt','r')
    # remove headers
    for j in range(12):
        handle.readline()
    for aoaIndex in range(16):
        line = handle.readline()
        data = [float(val) for val in line.split()]

        cl[flapIndex][aoaIndex] = data[1]
        cd[flapIndex][aoaIndex] = data[2]
        cm[flapIndex][aoaIndex] = data[4]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(flapm, aoam, cl*100, edgecolor='none')
ax.set_title('cl')
plt.show()



