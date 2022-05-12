"""Exercise 8e"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_trajectory, plot_positions
import matplotlib.pyplot as plt


def exercise_8e1(timestep,duration):
    """Exercise 8e1"""

    times = np.arange(0, duration, timestep)

        # Use exercise_example.py for reference
        #pass
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0,
            contra_coupling_weight = 0, 
            amplitude_gradient = None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
            #turn=0.5,  # Another example
            # ...
        )
        # for drive in np.linspace(3, 4, 2)
        # for amplitudes in ...
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8e1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8e1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
            # fast=True,  # For fast mode (not real-time)
            # headless=True,  # For headless mode (No GUI, could be faster)
            # record=True,  # Record video
        )
        # Log robot data
        data.to_file(filename.format(simulation_i, 'h5'), sim.iteration)
        # Log simulation parameters
        with open(filename.format(simulation_i, 'pickle'), 'wb') as param_file:
            pickle.dump(sim_parameters, param_file)

    links_positions = data.sensors.links.urdf_positions()

    head_positions = links_positions[:, 0, :]
    tail_positions = links_positions[:, 7, :]

    plot_trajectory(head_positions)
    plot_positions(times, head_positions)


def exercise_8e2(timestep,duration):
    """Exercise 8e2"""

    # Use exercise_example.py for reference
    pass


if __name__ == '__main__':
    exercise_8e1(timestep=1e-2,duration=10)
    exercise_8e2(timestep=1e-2,duration=10)

