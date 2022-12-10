import sys, os
import numpy as np
from matplotlib import pyplot as plt
#dir_path = os.path.dirname(os.path.realpath(__file__))
#print(dir_path)
#sys.path.insert(0, dir_path)
from euler_dynamics import *
import unittest


class TestDynamics(unittest.TestCase):

    def test_full_rotation(self):
        rotation = quaternion_rotation([1, 0, 0, 0], [0, 0, 0, 1])

        assert rotation == [180, 0, 0, 1]


if(__name__ == "__main__"):

    # Time stuff
    run_time = 10
    dt = 0.1

    # initial conditions
    Itensor = np.array([5,
                          5,
                            2])  # kg*m^2
    initial_velocity = [3, 0, 2]
    initial_attitude = [1, 0, 0, 0]  # this will eventually be cube verticies
    moments = [0, 0, 0]

    velocity_mat, attitude_mat = simulate_dynamics(Itensor, initial_velocity,
                                                   initial_attitude, moments,
                                                   run_time, dt, 
                                                   just_last=False)

    bigO = ((Itensor[2]-Itensor[0])/Itensor[0])*2

    time = np.arange(0, (run_time)+dt, dt)
    errorx = np.empty((0, 2), float)
    errory = np.empty((0, 2), float)

    for t in range(len(time)):

        errorx = np.append(errorx,
                           [3*np.cos(bigO * time[t])-velocity_mat[t][0]])
        errory = np.append(errory,
                           [3*np.sin(bigO * time[t])-velocity_mat[t][1]])
        # print("quat")
        # print(attitude_mat[t][0], attitude_mat[t][3])
        # print("angle")
        # print(np.arccos(attitude_mat[t][0])*2 - np.arccos(attitude_mat[t-1][0])*2, np.arcsin(attitude_mat[t][3])*2 - np.arcsin(attitude_mat[t-1][3])*2)
        # print("rotation")
        # print(quaternion_rotation(attitude_mat[t-1],attitude_mat[t]))

    fig, ax = plt.subplots(2, 3)
    ax[0][0].plot(time, velocity_mat[:, 0])
    ax[0][0].set_title("Numerical Integration w1")
    ax[0][0].set_xlabel("Time (s)")
    ax[0][0].set_ylabel("Angular Velocity (rad/s)")
    ax[0][1].plot(time, velocity_mat[:, 1])
    ax[0][1].set_title("Numerical ntegration w2")
    ax[0][1].set_xlabel("Time (s)")
    ax[0][1].set_ylabel("Angular Velocity (rad/s)")


    ax[1][0].plot(time, 3 * np.cos(bigO * time))
    ax[1][0].set_title("Analytical Solution w1")
    ax[1][0].set_xlabel("Time (s)")
    ax[1][0].set_ylabel("Angular Velocity (rad/s)")
    ax[1][1].plot(time, 3 * np.sin(bigO * time) )
    ax[1][1].set_title("Analytical Solution w2")
    ax[1][1].set_xlabel("Time (s)")
    ax[1][1].set_ylabel("Angular Velocity (rad/s)")

    ax[0][2].plot(errorx, errory)
    ax[0][2].set_title("Accumulated Error over time")
    ax[0][2].set_xlabel("w1 Error (rad)")
    ax[0][2].set_ylabel("w2 Error (rad)")

    ax[1][2].plot(velocity_mat[:, 0], velocity_mat[:, 1])
    ax[1][2].plot(3 * np.cos(bigO * time), 3 * np.sin(bigO * time))
    ax[1][2].set_title("Conservation of Angular Momentumf(w1,w2,t)")
    ax[1][2].set_xlabel("W1 Angular Velocity (rad/s)")
    ax[1][2].set_ylabel("W2 Angular Velocity (rad/s)")

    fig.suptitle("Numerically integrated vs Analytical Solutions for λ1=λ2 > λ3")
    plt.show()

