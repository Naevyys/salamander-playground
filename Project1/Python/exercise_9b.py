"""Exercise 9b"""

import os
import pickle
import numpy as np
from farms_core import pylog
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d, plt, plot_1d
from utils import compute_energy, compute_velocity, convert_nd_matrix_to_nd_plot_coordinates


def exercise_9b(timestep):
    """Exercise 9b"""

    # Use exercise_example.py for reference
    # Additional hints:
    # sim_parameters = SimulationParameters(
    #     ...,
    #     spawn_position=[0, 0, 0.1],  # For land to water
    #     spawn_orientation=[0, 0, 0],
    #     # Or
    #     spawn_position=[4, 0, 0.0],  # For water to land
    #     spawn_orientation=[0, 0, np.pi],
    # )
    # _sim, _data = simulation(
    #     sim_parameters=sim_parameters,
    #     arena='amphibious',
    #     fast=True,
    #     record=True,
    #     record_path='walk2swim',  # or swim2walk
    # )

    duration = 10  # Needs longer to see the full transition

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=spawn_position,  # Robot position in [m]
            spawn_orientation=spawn_orientation,  # Orientation in Euler angles [rad]
            drive=drive,
            phase_lag_limb2body=3*np.pi/2,
            amplitude_scaling=1.4,
            amplitude_gradient=None,
            initial_phases=None,
            turn=0,  # Another example
            # ...
        )
        for spawn_position, spawn_orientation, drive in zip([[0, 0, 0.1], [2, 0, 0]], [[0, 0, 0], [0, 0, np.pi]], [3, 3.5])
    ]

    # Grid search
    directory = './logs/exercise9b'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        record_path = ['walk2swim', 'swim2walk'][simulation_i]
        filename = directory + '/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='amphibious',
            fast=True,  # For fast mode (not real-time)
            # headless=True,  # For headless mode (No GUI, could be faster)
            record=True,  # Record video
            record_path=record_path
        )
        # Log robot data
        data.to_file(filename.format(simulation_i, 'h5'), sim.iteration)
        # Log simulation parameters
        with open(filename.format(simulation_i, 'pickle'), 'wb') as param_file:
            pickle.dump(sim_parameters, param_file)

        links_positions = data.sensors.links.urdf_positions()
        joints_torques = data.sensors.joints.motor_torques_all()

        plt.figure('9b_X_GPS_coordinate')
        plt.plot(links_positions[:, :, 0] + np.arange(links_positions.shape[1]))
        plt.xlabel('Time [ms]')
        plt.ylabel('X GPS coordinate [m]')
        plt.title('X GPS coordinates over time')
        plt.savefig('9b_X_GPS_coordinate_{}.pdf'.format(record_path))
        plt.show()
        plt.figure('9b_Joint_torques')
        plt.plot(joints_torques[:, :] + np.arange(joints_torques.shape[1]))
        plt.xlabel('Time [ms]')
        plt.ylabel('Joint torques [rad]')
        plt.title('Joint torques over time')
        plt.savefig('9b_Joint_torques_{}.pdf'.format(record_path))
        plt.show()


if __name__ == '__main__':
    exercise_9b(timestep=1e-2)

