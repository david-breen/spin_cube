from locale import normalize
import numpy as np
from scipy.spatial.transform import Rotation as R


def Euler_motion(w, M, Idiag, t):

	wdhold = np.matmul((M - np.cross(w,(np.matmul(Idiag, w)))),
	                    np.linalg.inv(Idiag))

	return wdhold


def w_to_Qdot(velocity, quat):
    
    Rq = np.array([[1-(2*quat[2]**2)-(2*quat[3]**2), 
                    2*quat[1]*quat[2]-2*quat[0]*quat[3], 
                    2*quat[1]*quat[3]+2*quat[0]*quat[2]], 
                    
                    [2*quat[1]*quat[2]+2*quat[0]*quat[3], 
                     1-(2*quat[1]**2)-(2*quat[3]**2),
                     2*quat[2]*quat[3]-2*quat[0]*quat[1]], 
                    
                    [2*quat[1]*quat[3]-2*quat[0]*quat[2],
                     2*quat[2]*quat[3]+2*quat[0]*quat[1],
                     1-(2*quat[1]**2)-(2*quat[2]**2)]],
                     float)

    world_velocity = np.matmul(Rq, velocity)
    v2 = [quat[1], quat[2], quat[3]]
    qdot = np.append(-(np.matmul(world_velocity, v2)),
                     (quat[0]*world_velocity)+
                     np.cross(world_velocity,v2))

    return qdot


def RK45_step(w, M, Idiag, time, deltat, attitude_quat):

    kq1 = deltat * w_to_Qdot(w, attitude_quat)
    kw1 = deltat * Euler_motion(w, M, Idiag, time)
    kq2 = deltat * w_to_Qdot(w + kw1, attitude_quat + .5*kq1)
    kw2 = deltat * Euler_motion(w + 0.5*kw1, M, Idiag, time + 0.5*deltat)
    kq3 = deltat * w_to_Qdot(w + kw2, attitude_quat + .5*kq2)
    kw3 = deltat * Euler_motion(w + 0.5*kw2, M, Idiag, time + 0.5*deltat)
    kq4 = deltat * w_to_Qdot(w + kw3, attitude_quat + kq3)
    kw4 = deltat * Euler_motion(w + kw3, M, Idiag, time + deltat)
    new_w = w + ((kw1 + 2*kw2 + 2*kw3 + kw4) / 6)
    new_Q = attitude_quat + ((kq1 + 2*kq2 + 2*kq3 + kq4) / 6)
    new_Q = new_Q/np.linalg.norm(new_Q)
    
    return new_w, new_Q


def simulate_dynamics(I_diag, init_velo, attitude, moments, run_time, dt,
                      just_last=True):

    # Time stuff
    time = np.arange(0, run_time, dt)

    # initial conditions
    Itensor = np.array([[I_diag[0], 0, 0], [0, I_diag[1], 0], [0, 0, I_diag[2]]])  # kg*m^2
    initial_velocity = np.array(init_velo, float)
    initial_attitude = np.array(attitude, float)

    M = np.array(moments)

    # Matrix initalization
    velocity_mat = np.empty((0, 3), float)
    attitude_mat = np.empty((0, 4), float)

    velocity_mat = np.append(velocity_mat, [initial_velocity], axis=0)
    attitude_mat = np.append(attitude_mat, [initial_attitude], axis=0)

    for t in time:

        w_hold, Q_hold = RK45_step(velocity_mat[-1], M, Itensor, time, dt,
                                   attitude_mat[-1])

        velocity_mat = np.append(velocity_mat, [w_hold] , axis=0)
        attitude_mat = np.append(attitude_mat, [Q_hold], axis=0)

    if(just_last == False):
        return velocity_mat, attitude_mat
    else:
        return velocity_mat[-1], attitude_mat[-1]


#function to rotate the verticies by the new quat value
# Q0-1 = Q0 * -Q1
def quaternion_rotation(quat0, quat1):

    a1, b1, c1, d1 = quat0
    b1, c1, d1 = -b1, -c1, -d1
    a2, b2, c2, d2 = quat1

    # this is just the hamiltonian product
    r0, r1, r2, r3 = np.array([a1*a2 - b1*b2 - c1*c2 - d1*d2,
                               a1*b2 + b1*a2 + c1*d2 - d1*c2,
                               a1*c2 - b1*d2 + c1*a2 + d1*b2,
                               a1*d2 + b1*c2 - c1*b2 + d1*a2
                              ], dtype=np.float64)
    
    # find the vector and angle to rotate about
    normal = np.linalg.norm([r1, r2, r3])
    theta = np.arctan2(normal, r0)

    if normal == 0:
        r_vector = [0, 0, 0]
    else:
        r_vector = np.divide([r1, r2, r3], np.sin(theta))

    return np.append(np.degrees(2*theta), r_vector)



if __name__ == "__main__":

    print("testing")
    
