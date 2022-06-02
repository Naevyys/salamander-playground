"""Exercise 8b"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d, plt
from utils import compute_energy, compute_velocity, convert_nd_matrix_to_nd_plot_coordinates, compute_salamander_wavelength


def exercise_8b(timestep):
    """Exercise 8b"""

    # Grid search parameters
    amp_n_vals = 10
    phase_n_vals = 8
    amplitude_vals = np.linspace(0, 5, num=amp_n_vals)
    phase_lag_vals = np.linspace(0, 2*np.pi/4, num=phase_n_vals)

    duration = 10

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,
            amplitude_scaling=amplitudes,  # Just an example
            amplitude_gradient=None,
            phase_lag_body=phase_lag,  # or np.zeros(n_joints) for example
            turn=0,  # Another example
            # ...
        )
        for amplitudes in amplitude_vals
        for phase_lag in phase_lag_vals
    ]

    velocities = np.zeros((amp_n_vals, phase_n_vals))
    energies = np.zeros((amp_n_vals, phase_n_vals))
    salamander_wavelength = np.zeros((amp_n_vals, phase_n_vals))

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
        head_pos, tail_pos = links_positions[:, 0, :], links_positions[:, 7, :]
        joints_velocities = data.sensors.joints.velocities_all()
        joints_torques = data.sensors.joints.motor_torques_all()

        amp_i = simulation_i // phase_n_vals
        phase_i = simulation_i % phase_n_vals

        velocities[amp_i, phase_i] = compute_velocity(links_positions, timestep=timestep)
        energies[amp_i, phase_i] = compute_energy(joints_torques, joints_velocities)
        salamander_wavelength[amp_i, phase_i] = compute_salamander_wavelength(head_pos, tail_pos)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities, x_vals=amplitude_vals, y_vals=phase_lag_vals)
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies, x_vals=amplitude_vals, y_vals=phase_lag_vals)
    coordinates_wavelengths = convert_nd_matrix_to_nd_plot_coordinates(salamander_wavelength, x_vals=amplitude_vals, y_vals=phase_lag_vals)

    # Plot velocity, energy and wavelength as a function of amplitude and phase lag
    plt.figure('8b_Amplitude_Phase_Velocity')
    plot_2d(coordinates_velocities, ("Amplitude [a.u.]", "Phase [rad]", "Velocity [m/s]"), plot=False)
    plt.show()
    plt.figure('8b_Amplitude_Phase_Energy')
    plot_2d(coordinates_energy, ("Amplitude [a.u.]", "Phase [rad]", "Energy [J]"), plot=False)
    plt.show()
    plt.figure('8b_Amplitude_Phase_Wavelength')
    plot_2d(coordinates_wavelengths, ("Amplitude [a.u.]", "Phase [rad]", "Wavelength [m]"), plot=False)
    plt.show()

    # Plot wavelength as a function of velocity and energy
    coordinates = np.stack([matrix.flatten() for matrix in [velocities, energies, salamander_wavelength]], axis=-1)
    plt.figure('8b_Velocity_Energy_Wavelength')
    plot_2d(coordinates, ("Velocity [m/s]", "Energy [J]", "Wavelength [m]"), plot=False)
    plt.show()


if __name__ == '__main__':
    exercise_8b(timestep=1e-2)
