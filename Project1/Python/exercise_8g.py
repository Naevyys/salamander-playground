"""Exercise 8g"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
import matplotlib.pyplot as plt
from utils import compute_velocity

def num_pieces(num,length,seed): # With inspiration from: https://stackoverflow.com/questions/53494200/how-to-split-a-number-into-12-random-in-python-and-all-random-number-sum-to-be-e
    all_list = []
    np.random.seed(seed=seed)
    for i in range(length-1):
        n = np.random.randint(0, num+1)
        all_list.append(n)
        num -= n
    all_list.append(num)
    return all_list

# Set repetitions and the maximum number of disruptions
repetitions=5
max_disruptions=9



def exercise_8g(timestep=1e-2, duration=10, feedback_weight=2, updown_coupling_weight=10, plot_column=None, set_seed=True):
    """Exercise 8g"""

    rng = np.random.default_rng(seed=42)

    times = np.arange(0, duration, timestep)

    velocities=np.zeros(len(np.repeat(np.arange(max_disruptions),repetitions)))

    ####################
    ## Mute couplings ##
    ####################

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
            updown_coupling_weight=updown_coupling_weight,
            feedback_weight=feedback_weight,
            turn=0,  # Another example
            randseed=rng.integers(123456789),
            set_seed=set_seed,
            n_disruption_couplings=n_disruption_couplings,  # Scalar between 0 and 7
            n_disruption_oscillators=0,  # Scalar between 0 and 8
            n_disruption_sensors=0,  # Scalar between 0 and 8
            # ...
        )
        for n_disruption_couplings in np.repeat(np.arange(max_disruptions-1), repetitions)
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8g1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8g1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
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

        velocities[simulation_i] = compute_velocity(links_positions)

    velocities_reshaped = velocities[:-1 * repetitions].reshape(max_disruptions - 1, repetitions)

    print(velocities_reshaped)

    axes[1,plot_column].errorbar(np.arange(max_disruptions - 1), np.mean(velocities_reshaped, axis=1),
                 yerr=np.std(velocities_reshaped, axis=1), fmt='-o')



    ######################
    ## Mute oscillators ##
    ######################

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
            updown_coupling_weight=updown_coupling_weight,
            feedback_weight=feedback_weight,
            turn=0,  # Another example
            randseed=rng.integers(123456789),
            set_seed=set_seed,
            n_disruption_couplings=0,  # Scalar between 0 and 7
            n_disruption_oscillators=n_disruption_oscillators,  # Scalar between 0 and 8
            n_disruption_sensors=0,  # Scalar between 0 and 8
            # ...
        )
        for n_disruption_oscillators in np.repeat(np.arange(max_disruptions), repetitions)
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8g1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8g1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
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

        velocities[simulation_i] = compute_velocity(links_positions)

    velocities_reshaped = velocities.reshape(max_disruptions, repetitions)

    print(velocities_reshaped)

    axes[2,plot_column].errorbar(np.arange(max_disruptions), np.mean(velocities_reshaped, axis=1),
                 yerr=np.std(velocities_reshaped, axis=1), fmt='-o')



    ##################
    ## Mute sensors ##
    ##################

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
            updown_coupling_weight=updown_coupling_weight,
            feedback_weight=feedback_weight,
            turn=0,  # Another example
            randseed=rng.integers(123456789),
            set_seed=set_seed,
            n_disruption_couplings=0,  # Scalar between 0 and 7
            n_disruption_oscillators=0,  # Scalar between 0 and 8
            n_disruption_sensors=n_disruption_sensors,  # Scalar between 0 and 8
            # ...
        )
        for n_disruption_sensors in np.repeat(np.arange(max_disruptions), repetitions)
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8g1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8g1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
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

        velocities[simulation_i] = compute_velocity(links_positions)

    velocities_reshaped = velocities.reshape(max_disruptions, repetitions)

    print(velocities_reshaped)

    axes[0,plot_column].errorbar(np.arange(max_disruptions), np.mean(velocities_reshaped, axis=1),
                 yerr=np.std(velocities_reshaped, axis=1), fmt='-o')

    ##############
    ## Mute mix ##
    ##############

    mixed_disruptions=np.zeros((max_disruptions-1,3))
    for i in range(max_disruptions-1):
        mixed_disruptions[i, :] = num_pieces(i, 3,rng.integers(123456789))

    mixed_disruptions = mixed_disruptions.astype(int)

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
            updown_coupling_weight=updown_coupling_weight,
            feedback_weight=feedback_weight,
            turn=0,  # Another example
            randseed=rng.integers(123456789),
            set_seed=set_seed,
            n_disruption_couplings=n_disruption_couplings,  # Scalar between 0 and 7
            n_disruption_oscillators=n_disruption_oscillators,  # Scalar between 0 and 8
            n_disruption_sensors=n_disruption_sensors,  # Scalar between 0 and 8
            # ...
        )
        for [n_disruption_couplings,n_disruption_oscillators,n_disruption_sensors] in mixed_disruptions
        # for ...
    ]



    # Grid search
    directory = './logs/exercise8g1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8g1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
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

        velocities[simulation_i] = compute_velocity(links_positions)

    velocities_reshaped = velocities[:-1 * repetitions].reshape(max_disruptions - 1, repetitions)

    print(velocities_reshaped)

    axes[3,plot_column].errorbar(np.arange(max_disruptions - 1), np.mean(velocities_reshaped, axis=1),
                 yerr=np.std(velocities_reshaped, axis=1), fmt='-o')

##################
## Plot results ##
##################

if __name__ == '__main__':
    fig,axes=plt.subplots(4,3,sharex=True,sharey=True,figsize=(12,9))

    # Define y and x labels
    for i in range(4):
        axes[i,0].set_ylabel('Speed [m/s]')
    for j in range(3):
        axes[-1,j].set_xlabel('Number of neural disruptions')

    # Define and assign row and column labels: https://stackoverflow.com/questions/25812255/row-and-column-headers-in-matplotlibs-subplots
    cols = ['CPG only','Decoupled','Combined']
    rows = ['Muted\nsensors','Removed\ncouplings','Muted\noscillators','Mixed\ndisruptions']
    pad = 5 # in points
    for ax, col in zip(axes[0], cols):
        ax.annotate(col, xy=(0.5, 1), xytext=(0, pad),
                    xycoords='axes fraction', textcoords='offset points',
                    size='large', ha='center', va='baseline')

    for ax, row in zip(axes[:,0], rows):
        ax.annotate(row, xy=(0, 0.5), xytext=(-ax.yaxis.labelpad - pad, 0),
                    xycoords=ax.yaxis.label, textcoords='offset points',
                    size='large', ha='right', va='center')
    fig.tight_layout()
    fig.subplots_adjust(left=0.15, top=0.95)

    # CPG only
    exercise_8g(timestep=1e-2, duration=10, feedback_weight=0, updown_coupling_weight=10, plot_column=0)
    # Decoupled
    exercise_8g(timestep=1e-2, duration=10, feedback_weight=2, updown_coupling_weight=0, plot_column=1)
    # Combined
    exercise_8g(timestep=1e-2, duration=10, feedback_weight=2, updown_coupling_weight=10, plot_column=2)

    plt.show()

    plt.savefig('8g_disruptions.pdf', bbox_inches='tight')
