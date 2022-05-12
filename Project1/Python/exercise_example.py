"""Exercise example"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters


def exercise_example(timestep):
    """Exercise example"""

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive= drive,  # An example of parameter part of the grid search
            amplitude_scaling=1,  # Just an example
            updown_coupling_weight = 0, 
            amplitude_gradient = None,
            #amplitude_gradient = [0.5,2],
            #amplitude_gradient_scaling = True,
            phase_lag_body= 2*np.pi/8,  # or np.zeros(n_joints) for example
            turn=0,  # Another example
            # ...
        )
        for drive in np.linspace(3, 4, 2)
        # for amplitudes in ...
        # for ...
    ]

    # Grid search
    os.makedirs('./logs/example/', exist_ok=True)
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/example/simulation_{}.{}'
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


if __name__ == '__main__':
    exercise_example(timestep=1e-2)

