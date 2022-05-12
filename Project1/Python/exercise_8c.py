"""Exercise 8c"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d

def compute_velocity(pos, start_time=400, end_time=-1):

    if end_time == -1:
        end_time = pos.shape[0] - 1

    pos_start = np.mean(pos[start_time], axis=0)
    pos_end = np.mean(pos[end_time], axis=0)

    distance = np.sqrt(np.sum(np.square(pos_end - pos_start)))
    delta_time = end_time - start_time

    return distance / delta_time


def compute_energy(joint_torque, joint_velocities):
    return np.sum(np.multiply(joint_torque, joint_velocities))

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

def exercise_8c(timestep):
    """Exercise 8c"""


    # Grid search parameters
    n_vals = 6
    amplitude_vals = np.linspace(0, 1.5, num=n_vals)  # TODO: choose good values
    #amplitude_vals = 2  # TODO: choose good values

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=3.5,  # ??? CHECK
            amplitude_gradient= [amplitude_1,amplitude_2] ,  # Just an example  # TODO: check if name of key is correct...
            phase_lag_body= ((2*np.pi)/8),  # TODO: check if this value makes sense, change if needed
            turn=0,  # Another example
            # ...
        )
        for amplitude_1 in amplitude_vals
        for amplitude_2 in amplitude_vals
    ]

    velocities = np.zeros((n_vals, n_vals))
    energies = np.zeros((n_vals, n_vals))

    # Grid search
    directory = './logs/exercise8c'  # TODO: check if this value makes sense, change if needed
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = directory + '/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Swimming
            fast=True,  # For fast mode (not real-time)  # TODO: Set this to True if simulation takes too much time
            headless=True,  # For headless mode (No GUI, could be faster)  # TODO: same
            # record=True,  # Record video
        )
        # Log robot data
        data.to_file(filename.format(simulation_i, 'h5'), sim.iteration)
        # Log simulation parameters
        with open(filename.format(simulation_i, 'pickle'), 'wb') as param_file:
            pickle.dump(sim_parameters, param_file)

        links_positions = data.sensors.links.urdf_positions()
        joints_velocities = data.sensors.joints.velocities_all()
        joints_torques = data.sensors.joints.motor_torques_all()

        amp_1 = simulation_i // n_vals
        amp_2 = simulation_i % n_vals 

        velocities[amp_1, amp_2] = compute_velocity(start_time = 0, pos = links_positions)
        energies[amp_1, amp_2] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities,x_vals = amplitude_vals, y_vals= amplitude_vals) 
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies,x_vals = amplitude_vals, y_vals= amplitude_vals) 

    plot_2d(coordinates_velocities, ("Amplitude head", "Amplitude tail", "Velocity"))
    plot_2d(coordinates_energy, ("Amplitude head", "Amplitude tail" , "Energy"))



if __name__ == '__main__':
    exercise_8c(timestep=1e-2)

