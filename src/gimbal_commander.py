import time
import serial
import numpy as np

# Import LSS library
import lss
import lss_const as lssc

# Constants
#CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
CST_LSS_Port = "COM7"				# For windows platforms
CST_LSS_Baud = lssc.LSS_DefaultBaud

# Create and open a serial port
lss.initBus(CST_LSS_Port, CST_LSS_Baud)

# Create an LSS object

# these need to be set to match the servo positions of your gimbal
Bphi = lss.LSS(1)
Btheta = lss.LSS(0)
Bgamma = lss.LSS(2)
axies = [Bphi, Btheta, Bgamma]

# Initialize LSS to position 0.0 deg

# Wait for it to get there
time.sleep(2)


def init_gimbals():

    for axis in axies:

        axis.setAngularAcceleration(1, lssc.LSS_SetConfig)
        axis.setAngularDeceleration(1, lssc.LSS_SetConfig)
        axis.setAngularHoldingStiffness(-3, lssc.LSS_SetConfig)
        axis.setMaxSpeed(60)

        axis.move(0)


def quaternion_movement(attitude_mat):

    a1, b1, c1, d1 = attitude_mat
    b1, c1, d1 = -b1, -c1, -d1
    a2, b2, c2, d2 = [0, 0, 0, 1]

    # this is just the hamiltonian product
    r0, r1, r2, r3 = np.array([a1*a2 - b1*b2 - c1*c2 - d1*d2,
                               a1*b2 + b1*a2 + c1*d2 - d1*c2,
                               a1*c2 - b1*d2 + c1*a2 + d1*b2,
                               a1*d2 + b1*c2 - c1*b2 + d1*a2
                              ], dtype=np.float64)

    a1, b1, c1, d1 = [r0, r1, r2, r3]
    b1, c1, d1 = -b1, -c1, -d1
    a2, b2, c2, d2 = [0, 0, 0, 1]

    # this is just the hamiltonian product
    r0, r1, r2, r3 = np.array([a1*a2 - b1*b2 - c1*c2 - d1*d2,
                               a1*b2 + b1*a2 + c1*d2 - d1*c2,
                               a1*c2 - b1*d2 + c1*a2 + d1*b2,
                               a1*d2 + b1*c2 - c1*b2 + d1*a2
                              ], dtype=np.float64)
    
    
    normal = np.linalg.norm([r1,r2,r3])
    
    
    print(r3)
    if normal == 0:
        spherical_theta = 0
        spherical_phi = 0
    else:
        spherical_theta = np.arccos(r3/normal)
        spherical_phi = np.arctan2(r2, r1)


    Bphi.move(int(np.degrees*10))
    Btheta.move(int(np.rad2deg(spherical_theta)*10))
    Bgamma.move(int(np.rad2deg(spherical_phi)*10))

