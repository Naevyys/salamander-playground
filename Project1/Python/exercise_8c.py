"""Exercise 8c"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_2d
from plot_results import plot_trajectory, plot_positions
from utils import compute_energy, compute_velocity, convert_nd_matrix_to_nd_plot_coordinates



def exercise_8c(timestep):
    """Exercise 8c"""

    "Part 1"

    # Grid search parameters
    n_vals = 6
    #amplitude_vals = np.linspace(0, 3.2, num=n_vals)  # with scaling
    #amplitude_vals = np.linspace(0, 5, num=n_vals)  # with scaling
    amplitude_vals = np.linspace(0, 1.5, num=n_vals)  # TODO: choose good values
    print(amplitude_vals)
    #amplitude_vals = 2  # TODO: choose good values

    #print(amplitude_vals)

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=3.5,  # ??? CHECK
            #amplitude_gradient= [0,1.5] ,
            amplitude_gradient= [amplitude_1,amplitude_2] ,  # Just an example  # TODO: check if name of key is correct...
            #amplitude_gradient_scaling = True,
            phase_lag_body= ((2*np.pi)/8),  # TODO: check if this value makes sense, change if needed
            turn=0,  # Another example
            # ...
        )
        for amplitude_1 in amplitude_vals
        for amplitude_2 in amplitude_vals
    ]

    velocities = np.zeros((n_vals, n_vals))
    energies = np.zeros((n_vals, n_vals))

    # Grid search
    directory = './logs/exercise8c'  # TODO: check if this value makes sense, change if needed
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = directory + '/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Swimming
            fast=True,  # For fast mode (not real-time)  # TODO: Set this to True if simulation takes too much time
            headless=True,  # For headless mode (No GUI, could be faster)  # TODO: same
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

        head_positions = links_positions[:, 0, :]
        tail_positions = links_positions[:, 7, :]

        amp_1 = simulation_i // n_vals
        amp_2 = simulation_i % n_vals 

        velocities[amp_1, amp_2] = compute_velocity(start_time = 0, pos = links_positions, timestep = timestep)
        energies[amp_1, amp_2] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities,x_vals = amplitude_vals, y_vals= amplitude_vals) 
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies,x_vals = amplitude_vals, y_vals= amplitude_vals) 
 
    plot_2d(coordinates_velocities, ("Amplitude head", "Amplitude tail", "Velocity"))
    plot_2d(coordinates_energy, ("Amplitude head", "Amplitude tail" , "Energy"))



    "Part 2: Using ampltitude gradient scaling"

    '''


    # Grid search parameters
    n_vals = 6
    #amplitude_vals_1 = [0,0.3,0.3,0.6,0.9,1.2,1.5]
    #amplitude_vals_2 = [0,0.3,0.9,0.6,0.3,1.2,1.5]
    amplitude_vals_2 = np.zeros(6)
    amplitude_vals_1 = np.linspace(0, 1.5, num=n_vals)



    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=10,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=3.5,  # ??? CHECK
            amplitude_gradient= [amplitude_1,amplitude_2] ,  # Just an example  # TODO: check if name of key is correct...
            #amplitude_gradient_scaling = True,
            phase_lag_body= ((2*np.pi)/8),  # TODO: check if this value makes sense, change if needed
            turn=0,  # Another example
            # ...
        )
        for amplitude_1,amplitude_2  in zip(amplitude_vals_1,amplitude_vals_2)
    ]

    velocities = np.zeros((n_vals, n_vals))
    energies = np.zeros((n_vals, n_vals))

    # Grid search
    directory = './logs/exercise8c'  # TODO: check if this value makes sense, change if needed
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = directory + '/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Swimming
            #fast=True,  # For fast mode (not real-time)  # TODO: Set this to True if simulation takes too much time
            #headless=True,  # For headless mode (No GUI, could be faster)  # TODO: same
            # record=True,  # Record video
        )
        # Log robot data
        data.to_file(filename.format(simulation_i, 'h5'), sim.iteration)
        # Log simulation parameters
        with open(filename.format(simulation_i, 'pickle'), 'wb') as param_file:
            pickle.dump(sim_parameters, param_file)

    '''





if __name__ == '__main__':
    exercise_8c(timestep=1e-2)

