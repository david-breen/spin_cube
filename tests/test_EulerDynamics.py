import numpy as np
import unittest
from matplotlib import pyplot as plt
from src.euler_dynamics import euler_dynamics



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
    initial_velocity = [1, 1, -1]
    initial_attitude = [1, 0, 0, 0]  # this will eventually be cube verticies
    moments = [0, 0, 0]

    velocity_mat, attitude_mat = simulate_dynamics(Itensor, initial_velocity,
                                                   initial_attitude, moments,
                                                   run_time, dt, 
                                                   just_last=False)

    bigO = ((Itensor[2]-Itensor[0])/Itensor[0])*2

    time = np.arange(1, (run_time), dt)
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

    fig, ax = plt.subplots(3, 2)
    ax[0][0].plot(time, velocity_mat[:, 0])
    ax[0][0].set_title("Numerical integration")
    ax[1][0].plot(time, velocity_mat[:, 1])
    ax[2][0].plot(time, velocity_mat[:, 3])
    ax[2][0].set_title("w3 Velocity")

    ax[0][1].plot(time, 3 * np.cos(bigO * time))
    ax[0][1].set_title("Analytical integration")
    ax[1][1].plot(time, 3 * np.sin(bigO * time) )

    ax[2][1].plot(velocity_mat[:, 0], velocity_mat[:, 1])
    ax[2][1].plot(3 * np.cos(bigO * time), 3 * np.sin(bigO * time))
    ax[2][1].set_title("f(w1,w2,t)")

    fig.suptitle("Numerically integrated vs Analytical Solutions for λ1=λ2 > λ3")
    plt.show()

