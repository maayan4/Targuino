__author__ = 'raziele'

__author__ = 'raziele'

## Analyze motor measurement and determine the transfer function
## Each file contains several system measurements.
## This file organize the file an estimate the transfer function with kalman-filter

import cmath, io, datetime, math
import numpy as np
import pylab
import scipy.io as IO

fname = '/home/maayan4/Reps/Targuino/RawData/CumVelocity.dat'

fmode = 'U'

TEXT_SPACE = "\t"
END_OF_LINE = "\r"

rope_radius = 2.23 #[cm]

time = np.array([])
u = np.array([])
vK = np.array([])
AV = np.array([])
Rev = np.array([])
D = np.array([])
K = np.array([])
R = np.array([])
CU = np.array([])
Q = np.array([])
maxV = np.array([])
PS = np.array([])
vN = np.array([])

with open(fname,fmode) as fid:
    print("Opening file. Processing...")
    for line in fid:
        rowTmp=line.split()
        time = np.r_[time,float(rowTmp[0])/1000]
        u = np.r_[u,float(rowTmp[1])]
        vK = np.r_[vK, float(rowTmp[2])]
        AV = np.r_[AV,float(rowTmp[3])]
        Rev = np.r_[Rev,float(rowTmp[4])]
        CU = np.r_[CU,float(rowTmp[5])]
        PS = np.r_[PS,float(rowTmp[6])]
        D = np.r_[D,float(rowTmp[7])]
        K = np.r_[K,float(rowTmp[8])]
        R = np.r_[R,float(rowTmp[9])]
        Q = np.r_[Q,float(rowTmp[10])]
        maxV = np.r_[maxV,float(rowTmp[11])]
        vN = np.r_[vN,float(rowTmp[12])]
        
""" ax = pylab.figure(1)
ax1 = pylab.subplot(2,4,1)
pylab.plot(time,PH,'k+')
pylab.xlabel('time')
pylab.ylabel('$PH$')
ax2 = pylab.subplot(2,4,2)
pylab.plot(time,DT,'k+')
pylab.xlabel('time')
pylab.ylabel('$DT$')
ax3 = pylab.subplot(2,4,3)
pylab.plot(time,LV,'k+')
pylab.xlabel('time')
pylab.ylabel('$LV$')
ax4 = pylab.subplot(2,4,4)
pylab.plot(time,AV,'k+')
pylab.xlabel('time')
pylab.ylabel('$AV$')
ax5 = pylab.subplot(2,4,5)
pylab.plot(time,CU,'k+')
pylab.xlabel('time')
pylab.ylabel('$Current$')
ax6 = pylab.subplot(2,4,6)
pylab.plot(time,PS,'k+')
pylab.xlabel('time')
pylab.ylabel('$Power Supply$')
ax7 = pylab.subplot(2,4,7)
pylab.plot(PH,LV,'k+')
pylab.xlabel('PH')
pylab.ylabel('$LV$')
ax8 = pylab.subplot(2,4,8)
pylab.plot(PH, CU,'k+')
pylab.xlabel('PH')
pylab.ylabel('$Current$') """

# Kalman filter

QO = [1e-2]  #process variance
RO = [0.1] # estimate of measurement variance, change to see effect

# initial parameters
n_iter = vN.size
sz = (n_iter,) # size of array
z = AV * 255 / 157.07 # observations

xhatO = [0.0] # initial guess

for R in RO:
    for Q in QO:
        for xhati in xhatO:

            # allocate space for arrays
            xhat=np.zeros(sz)      # a posteri estimate of x
            P=np.zeros(sz)         # a posteri error estimate
            xhatminus=np.zeros(sz) # a priori estimate of x
            Pminus=np.zeros(sz)    # a priori error estimate
            K=np.zeros(sz)         # gain or blending factor

            xhat[0] = xhati
            P[0] = 1.0 # initial guess
            Pminus[0] = P[0] + Q

            for k in range(1,n_iter):
                # time update
                xhatminus[k] = xhat[k-1]
                Pminus[k] = P[k-1]+Q

                # measurement update
                K[k] = Pminus[k]/( Pminus[k]+R )
                xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
                P[k] = (1-K[k])*Pminus[k]

            pylab.figure()
            pylab.plot(time,z,'r.',label='noisy measurements')
            pylab.plot(time,xhat,'b-',label='a posteri estimate')
            pylab.plot(time,u, 'k--', label='input') #
            pylab.legend()
            pylab.title('Q = ' + str(Q) + ', R = ' + str(R) + ', xhat =' + str(xhati))
            pylab.xlabel('time')
            pylab.ylabel('Velocity')
            print('R=' + str(R) + ' Q=' + str(Q) + ' minX=' + str(xhat.min()) + '\n')



pylab.show()

print("finish")

data={"xhat":AV * 255 / 157.07, "time":time, "PH":u}

IO.savemat("/home/maayan4/Reps/Targuino/switchVNoisy.mat",data)