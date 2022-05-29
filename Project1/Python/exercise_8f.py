"""Exercise 8f"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d, plt


# Helper functions
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


# Exercise
def exercise_8f(timestep):
    """Exercise 8f"""

    # Grid search parameters
    coupling_weight_n_vals = 6
    feedback_weight_n_vals = 6
    coupling_weight_vals = np.linspace(0, 5, num=coupling_weight_n_vals)
    feedback_weight_vals = np.linspace(2, 5, num=feedback_weight_n_vals)

    duration = 10

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # Good value cause it is high enough to produce swimming behavior
            amplitude_scaling=1.5,  # Good value from ex. 8b
            amplitude_gradient=None,
            phase_lag_body=2*np.pi/16,  # Good value from ex. 8b
            turn=0,
            updown_coupling_weight=coupling_weight,  # Different coupling weights
            feedback_weight=feedback_weight,  # Different feedback weights
            initial_phases=np.array([(20-n)*(2*np.pi)/20 for n in np.arange(20)]),
        )
        for coupling_weight in coupling_weight_vals
        for feedback_weight in feedback_weight_vals
    ]

    velocities = np.zeros((coupling_weight_n_vals, feedback_weight_n_vals))
    energies = np.zeros((coupling_weight_n_vals, feedback_weight_n_vals))

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

        amp_i = simulation_i // feedback_weight_n_vals
        phase_i = simulation_i % feedback_weight_n_vals

        velocities[amp_i, phase_i] = compute_velocity(links_positions)
        energies[amp_i, phase_i] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities, x_vals=coupling_weight_vals,
                                                                      y_vals=feedback_weight_vals)
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies, x_vals=coupling_weight_vals,
                                                                  y_vals=feedback_weight_vals)

    # Plot velocity, energy and wavelength as a function of amplitude and phase lag
    plt.figure('8f_Amplitude_Phase_Velocity')
    plot_2d(coordinates_velocities, ("Amplitude [a.u.]", "Phase [rad]", "Velocity [m/s]"), plot=False)
    plt.show()
    plt.figure('8f_Amplitude_Phase_Energy')
    plot_2d(coordinates_energy, ("Amplitude [a.u.]", "Phase [rad]", "Energy [J]"), plot=False)
    plt.show()


if __name__ == '__main__':
    exercise_8f(timestep=1e-2)

