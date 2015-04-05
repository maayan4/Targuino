__author__ = 'raziele'

__author__ = 'raziele'

## Analyze motor measurement and determine the transfer function
## Each file contains several system measurements.
## This file organize the file an estimate the transfer function with kalman-filter

import cmath, io, datetime, math
import numpy as np
import pylab
import scipy.io as IO

fname = '/Users/raziele/Reps/Targuino/RawData/logR2_MA2_lastVal_updated.dat'

fmode = 'U'

TEXT_SPACE = "\t"
END_OF_LINE = "\r"

rope_radius = 2.23 #[cm]

time = np.array([])
PH = np.array([])
PL = np.array([])
R = np.array([])
DT = np.array([])
LV = np.array([])
AV = np.array([])
CU = np.array([])
FA = np.array([])
FB = np.array([])
PS = np.array([])

with open(fname,fmode) as fid:
    print("Opening file. Processing...")
    for line in fid:
        rowTmp=line.split()
        time = np.r_[time,float(rowTmp[0])/1000]
        PH = np.r_[PH,float(rowTmp[1])]
        PL = np.r_[PL, float(rowTmp[2])]
        R = np.r_[R,float(rowTmp[3])]
        DT = np.r_[DT,float(rowTmp[4])]
        LV = np.r_[LV,float(rowTmp[5])]
        AV = np.r_[AV,float(rowTmp[6])]
        CU = np.r_[CU,((float(rowTmp[7])-2.5)/0.066)]
        FA = np.r_[FA,float(rowTmp[8])]
        FB = np.r_[FB,float(rowTmp[9])]
        PS = np.r_[PS,float(rowTmp[10])]

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

QO = [1e-6, 1e-5, 1e-4]  # process variance
RO = [0.5**2, 0.7**2, 0.9**2] # estimate of measurement variance, change to see effect

# initial parameters
n_iter = LV.size
sz = (n_iter,) # size of array
z = LV # observations

xhatO = [0.0, z[0]] # initial guess

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
            pylab.plot(time,z,'k+',label='noisy measurements')
            pylab.plot(time,xhat,'b-',label='a posteri estimate')
            #pylab.plot(PH/PH.max(), 'k--', label='input') #
            pylab.legend()
            pylab.title('Q = ' + str(Q) + ', R = ' + str(R) + ', xhat =' + str(xhati))
            pylab.xlabel('time')
            pylab.ylabel('Velocity')



pylab.show()

print("finish")
