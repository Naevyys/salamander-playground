"""Exercise 8b"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
import plot_results
import matplotlib.pyplot as plt
from salamandra_simulation.data import SalamandraData


def exercise_8b(timestep):
    """Exercise 8b"""

    # Grid search parameters  # TODO: Try first with 2 values for each to see if the rest of the code fully works
    amplitude_vals = np.stack([np.linspace(i, i+3, num=3) for i in range(4)], axis=0)  # TODO: choose good values
    phase_lag_vals = np.linspace(0, 9, num=10)  # TODO: choose good values

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=3,  # TODO: check if this value makes sense, change if needed
            amplitudes=amplitudes,  # Just an example  # TODO: check if name is correct...
            phase_lag_body=phase_lag,  # or np.zeros(n_joints) for example
            turn=0,  # Another example
            # ...
        )
        for amplitudes in amplitude_vals
        for phase_lag in phase_lag_vals
    ]

    # Grid search
    directory = './logs/exercise8b'
    os.makedirs(directory, exist_ok=True)
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = directory + '/simulation_{}_amp_{}_phase_lag_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
            # fast=True,  # For fast mode (not real-time)  # TODO: Set this to True if simulation takes too much time
            # headless=True,  # For headless mode (No GUI, could be faster)  # TODO: same
            # record=True,  # Record video
        )
        # Log robot data
        data.to_file(filename.format(simulation_i, sim_parameters.amplitudes, sim_parameters.phase_lag_body, 'h5'), sim.iteration)
        # Log simulation parameters
        with open(filename.format(simulation_i, sim_parameters.amplitudes, sim_parameters.phase_lag_body, 'pickle'), 'wb') as param_file:
            pickle.dump(sim_parameters, param_file)

        timestep = data.timestep
        n_iterations = np.shape(data.sensors.links.array)[0]
        times = np.arange(
            start=0,
            stop=timestep * n_iterations,
            step=timestep,
        )
        osc_phases = data.state.phases()
        osc_amplitudes = data.state.amplitudes()

        # num_of_points = len(times) * osc_amplitudes.shape[1]
        # osc_phases_with_time = np.zeros((num_of_points, 3))
        # osc_amplitudes_with_time = np.zeros((num_of_points, 3))
        # for i in range(osc_amplitudes.shape[1]):
        #     int_begin = len(times) * i
        #     int_end = len(times)*(i+1)
        #     osc_phases_with_time[int_begin:int_end, 0], osc_amplitudes_with_time[int_begin:int_end, 0] = times, times
        #     osc_phases_with_time[int_begin:int_end, 2], osc_amplitudes_with_time[int_begin:int_end, 2] = osc_phases[:,i], osc_amplitudes[:, i]
        #     osc_phases_with_time[int_begin:int_end, 1], osc_amplitudes_with_time[int_begin:int_end, 1] = i, i
        #
        # labels_phases = ("Time", "Oscillator index", "Phase")
        # labels_amplitudes = ("Time", "Oscillator index", "Amplitude")
        # plot_results.plot_2d(osc_phases_with_time, labels_phases, n_data=len(times))
        # plot_results.plot_2d(osc_amplitudes_with_time, labels_amplitudes, n_data=len(times))

        # TODO: move to plot_results.py? Leave here? Is that even correct?
        for i, phases in enumerate(osc_phases.T):
            plt.figure('Simulation {}, oscillator phases'.format(simulation_i))
            plot_results.plot_1d(np.stack([times, phases], axis=1), ("Time", "Phase"))
        plt.show()

        for i, amp in enumerate(osc_amplitudes.T):
            plt.figure('Simulation {}, oscillator amplitudes'.format(simulation_i))
            plot_results.plot_1d(np.stack([times, amp], axis=1), ("Time", "Phase"))
        plt.show()


if __name__ == '__main__':
    exercise_8b(timestep=1e-2)

