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
try:
    lss.initBus(CST_LSS_Port, CST_LSS_Baud)
except:
    print("COM port " + CST_LSS_Port + " is empty or busy, try another port" )
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


def gimbal_movement(z_axis, velocity_mat):

  
    r = np.linalg.norm(z_axis)
    spherical_theta = np.arccos(z_axis[2]/r)
    spherical_phi = np.arctan2(z_axis[1], z_axis[0])

    print(spherical_theta)
    print(spherical_phi)
    print(velocity_mat[2]*(30/np.pi))
    

    Bphi.move(int(np.rad2deg(spherical_phi))*10)
    Btheta.move(int(np.rad2deg(spherical_theta))*10)
    Bgamma.wheelRPM(int(velocity_mat[2])*(30/np.pi))

