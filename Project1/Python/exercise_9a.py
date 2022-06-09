import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
import matplotlib.pyplot as plt
from utils import compute_velocity


# Phase lag sweep
n_vals_phase_lag=17
n_vals_drive=9
phase_lags=np.linspace(0,2*np.pi,n_vals_phase_lag)
drives=np.linspace(1,3,n_vals_drive)


def exercise_9a3(timestep=1e-2, duration=10):
    """Exercise 9a3"""

    times = np.arange(0, duration, timestep)

    velocities = np.zeros((n_vals_phase_lag, n_vals_drive))

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=drive,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_limb2body=phase_lag,  # or np.zeros(n_joints) for example
            # ...
        )
        for drive in drives
        for phase_lag in phase_lags
        # for ...
    ]

    # Grid search
    directory = './logs/exercise9a3'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise9a3/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='ground',  # Can also be 'ground', give it a try!
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

        drive_i = simulation_i // n_vals_phase_lag
        phase_i = simulation_i % n_vals_phase_lag

        velocities[phase_i, drive_i] = compute_velocity(links_positions, timestep=timestep)

    fig, ax = plt.subplots(1)
    for i, drive in enumerate(drives):
        ax.plot(phase_lags, velocities[:, i], '-o', label='drive={}'.format(drive))
    ax.set_xticks(np.arange(0, 2 * np.pi + 0.01, np.pi / 4))
    labels = ['$0$', r'$\pi/4$', r'$\pi/2$', r'$3\pi/4$', r'$\pi$',
              r'$5\pi/4$', r'$3\pi/2$', r'$7\pi/4$', r'$2\pi$']
    ax.set_xticklabels(labels)
    ax.set_xlabel('Phase lag [rad]')
    ax.set_ylabel('Speed [m/s]')
    ax.legend()
    plt.show()
    plt.savefig('9a3.pdf', bbox_inches='tight')

    idxMax = np.argwhere(velocities == velocities.max())[0]
    optPhase_lag = phase_lags[idxMax[0]]
    optDrive = drives[idxMax[1]]
    print('The highest speed was {} m/s with a phase lag = {} and drive = {}'.format(np.max(velocities), optPhase_lag,
                                                                                     optDrive))

    return optPhase_lag, optDrive

n_vals_amplitude=21
amplitudes = np.linspace(0,2,n_vals_amplitude)

def exercise_9a4(timestep=1e-2, duration=10,optPhase_lag=3*np.pi/2,optDrive=3):
    """Exercise 9a4"""

    times = np.arange(0, duration, timestep)

    velocities = np.zeros(n_vals_amplitude)

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=optDrive,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            amplitude_scaling=amplitude_scaling,
            phase_lag_limb2body=optPhase_lag,  # or np.zeros(n_joints) for example
            # ...
        )
        for amplitude_scaling in amplitudes
        # for ...
    ]

    # Grid search
    directory = './logs/exercise9a4'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise9a4/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='ground',  # Can also be 'ground', give it a try!
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

        velocities[simulation_i] = compute_velocity(links_positions, timestep=timestep)

    fig, ax = plt.subplots(1)
    ax.plot(amplitudes, velocities, '-o')
    ax.set_xlabel('Nominal amplitude scaling [1]')
    ax.set_ylabel('Speed [m/s]')
    ax.set_title(r'Drive = 3 and phase lag between limb and body of $3\pi/2$')
    plt.show()
    plt.savefig('9a4.pdf', bbox_inches='tight')

    optAmplitude=amplitudes[np.argmax(velocities)]
    print('The highest speed was {} m/s with a nominal amplitude scaling of {}'.format(np.max(velocities), optAmplitude))

if __name__ == '__main__':
    optPhase_lag,optDrive=exercise_9a3()
    exercise_9a4(optPhase_lag=optPhase_lag,optDrive=optDrive)

