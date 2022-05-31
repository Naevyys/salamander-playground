import numpy as np


def compute_velocity(pos, start_time=-100, end_time=-1):

    if end_time == -1:
        end_time = pos.shape[0] - 1
    if start_time < 0:
        start_time = pos.shape[0] + start_time

    pos_start = np.mean(pos[start_time], axis=0)
    pos_end = np.mean(pos[end_time], axis=0)

    distance = np.sqrt(np.sum(np.square(pos_end - pos_start)))
    delta_time = end_time - start_time

    return distance / delta_time


def compute_energy(joint_torque, joint_velocities, start_time=-100, end_time=-1):
    # joint_torque and joint_velocities are of shape (n_timepoints, n_joints)
    return np.sum(np.multiply(joint_torque[start_time:end_time], joint_velocities[start_time:end_time]))


def convert_nd_matrix_to_nd_plot_coordinates(m, x_vals=None, y_vals=None):

    coordinates = np.zeros((np.prod(m.shape), len(m.shape) + 1))
    for i, c in enumerate(np.ndindex(m.shape)):
        if x_vals is not None:
            coordinates[i, 0] = x_vals[c[0]]
        else:
            coordinates[i, 0] = c[0]
        if len(c) > 1 and y_vals is not None:
            coordinates[i, 1] = y_vals[c[1]]
        elif len(c) > 1:
            coordinates[i, 1] = c[1]
        coordinates[i, len(m.shape)] = m[c]

    return coordinates