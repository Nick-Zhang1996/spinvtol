# retrieve aerodynamic coefficients from xfoil polar output files
# fit aerodynamic coefficients of a deflecting flap
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from scipy.optimize import curve_fit

def _fitfunc1(x,a,b,c,d,e):
    flap,aoa = x
    return a*flap**2+b*flap+c*aoa**2+d*aoa+e

def fitNplot(func, data,name):
    xdata = (flapm.flatten(),aoam.flatten())
    ydata = data.flatten()
    popt, pcov = curve_fit(func, xdata,ydata)
    print(name + '  %e*flap**2+%e*flap+%e*aoa**2+%e*aoa+%e'%tuple(popt))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_trisurf(flapm.flatten(), aoam.flatten(), data.flatten(), linewidth=0.2,antialiased=True,cmap=plt.cm.CMRmap)
    ax.set_title(name)
    plt.show()

    # Plot the 3D figure of the fitted function and the residuals.
    fit = np.zeros_like(flapm)
    for i in range(flapm.shape[0]):
        for j in range(flapm.shape[1]):
            fit[i][j] = func((flapm[i][j],aoam[i][j]),*popt)

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    #ax.plot_surface(flapm, aoam, fit, cmap='plasma')
    ax.plot_trisurf(flapm.flatten(), aoam.flatten(), fit.flatten(), linewidth=0.2,antialiased=True,cmap=plt.cm.CMRmap)
    #cset = ax.contourf(flapm, aoam, data-fit, zdir='z', offset=residual_offset, cmap='plasma')
    ax.plot_trisurf(flapm.flatten(), aoam.flatten(), data.flatten()-fit.flatten(), linewidth=0.2,antialiased=True,cmap=plt.cm.CMRmap)
    print(name+' max err = '+str(max(data.flatten()-fit.flatten())))
    #ax.set_zlim(residual_offset,np.max(fit))
    plt.show()

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

fitNplot(_fitfunc1,cl,'cl')
fitNplot(_fitfunc1,cd,'cd')
fitNplot(_fitfunc1,cm,'cm')
