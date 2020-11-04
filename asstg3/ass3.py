import modern_robotics as mr
import numpy as np


## relevant kinematic and intertial parameters if UR5 robot given in assignment pdf

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

## my own code from here on out

g = [0,0,-9.81]
taulist = [0,0,0,0,0,0]
Ftip = [0,0,0,0,0,0]

## All we need to use is the mr.forwardynamics function to find thetadd
## taulist is going to be equal to 0
## ftip is also going to be 0

## for part 1
thetalist = np.array([0,0,0,0,0,0])
dthetalist = np.array([0,0,0,0,0,0])

## for part 2
thetalist = np.array([-1,-1,0,0,0,0])
dthetalist = np.array([0,0,0,0,0,0])

############## core.ForwardDynamics --> inputs needed are: #################
##
## thetalist -> n list of joint angles
## dthetalist -> n list of joint velocities
## taulist --> n list of required joint forces/troques (will be and remain zero for our case)
## g --> gravity in x,y and z
## Ftip --> wrench applied to the end effector frame (will be and remain zero for our case)
## Mlist --> list of link frames i relative to i-1 at the home position
## Glist --> spatial intertia matrices of the links (will remain the same throughout)
## Slist --> screw axis of the joints in a space frame

thetalist_iter = np.array(thetalist.copy())

while 1:
    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
    [thetalist,dthetalist] = mr.EulerStep(thetalist,dthetalist,ddthetalist,0.01)
    thetalist_iter = np.vstack([thetalist_iter,thetalist])

    