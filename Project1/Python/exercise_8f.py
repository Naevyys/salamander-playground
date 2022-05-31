"""Exercise 8f"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d, plt
from utils import compute_energy, compute_velocity, convert_nd_matrix_to_nd_plot_coordinates


# Exercise
def exercise_8f(timestep):
    """Exercise 8f"""

    # Grid search parameters
    coupling_weight_n_vals = 10
    feedback_weight_n_vals = 10
    coupling_weight_vals = np.linspace(0, 10, num=coupling_weight_n_vals)
    feedback_weight_vals = np.linspace(0, 5, num=feedback_weight_n_vals)

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
            initial_phases=None, #np.concatenate([np.array([(16-n)*(2*np.pi)/16 for n in np.arange(16)]), np.zeros(4)]),
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

        velocities[amp_i, phase_i] = compute_velocity(links_positions, timestep=timestep)
        energies[amp_i, phase_i] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities, x_vals=coupling_weight_vals,
                                                                      y_vals=feedback_weight_vals)
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies, x_vals=coupling_weight_vals,
                                                                  y_vals=feedback_weight_vals)

    # Plot velocity, energy and wavelength as a function of amplitude and phase lag
    plt.figure('8f_Coupling_Feedback_Velocity')
    plot_2d(coordinates_velocities, ("Coupling weight [a.u.]", "Feedback weight [a.u.]", "Velocity [m/s]"), plot=False)
    plt.show()
    plt.figure('8f_Coupling_Feedback_Energy')
    plot_2d(coordinates_energy, ("Coupling weight [a.u.]", "Feedback weight [a.u.]", "Energy [J]"), plot=False)
    plt.show()


if __name__ == '__main__':
    exercise_8f(timestep=1e-2)

