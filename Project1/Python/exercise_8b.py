"""Exercise 8b"""

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


def exercise_8b(timestep):
    """Exercise 8b"""

    # Grid search parameters
    amp_n_vals = 5
    phase_n_vals = 8
    amplitude_vals = np.linspace(-0.5, 0.5, num=amp_n_vals)  # TODO: change
    phase_lag_vals = np.linspace(0, 2*np.pi/4, num=phase_n_vals)

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,
            amplitude_gradient=amplitudes,  # Just an example
            phase_lag_body=phase_lag,  # or np.zeros(n_joints) for example
            turn=0,  # Another example
            # ...
        )
        for amplitudes in amplitude_vals
        for phase_lag in phase_lag_vals
    ]

    velocities = np.zeros((amp_n_vals, phase_n_vals))
    energies = np.zeros((amp_n_vals, phase_n_vals))

    # Grid search
    directory = './logs/exercise8b'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = directory + '/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Swimming
            fast=True,  # For fast mode (not real-time)
            headless=True,  # For headless mode (No GUI, could be faster)
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

        amp_i = simulation_i // phase_n_vals
        phase_i = simulation_i % phase_n_vals

        velocities[amp_i, phase_i] = compute_velocity(links_positions)
        energies[amp_i, phase_i] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities, x_vals=amplitude_vals, y_vals=phase_lag_vals)
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies, x_vals=amplitude_vals, y_vals=phase_lag_vals)

    plot_2d(coordinates_velocities, ("Amplitude", "Phase", "Velocity"))
    plot_2d(coordinates_energy, ("Amplitude", "Phase", "Energy"))


if __name__ == '__main__':
    exercise_8b(timestep=1e-2)


# TODO: Questions
# - 8a:
#   -
# - amplitudes = a factor, is that correct?
# - what is a good range for the amplitudes?
# - Is the calculation of the speed correct? Should we start at 0 or when it has stabilized?
# - Is the energy calculation correct?
# - What is meant by trying out other energy calculations?
# - Are there sources of randomness
# - 8c: what does the frequence of 1Hz mean?
