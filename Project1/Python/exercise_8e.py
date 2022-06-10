"""Exercise 8e"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_trajectory, plot_positions, plot_2d 
import matplotlib.pyplot as plt
from utils import compute_energy, compute_velocity, convert_nd_matrix_to_nd_plot_coordinates, plot_semilog_2d



def exercise_8e1(timestep,duration):
    """Exercise 8e1"""

    times = np.arange(0, duration, timestep)

    '''Part 1: no initial state'''

    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            #amplitude_scaling = 2,
            amplitude_gradient = None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
        )
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
    joints_velocities = data.sensors.joints.velocities_all()
    joints_torques = data.sensors.joints.motor_torques_all()

    head_positions = links_positions[:, 0, :]
    tail_positions = links_positions[:, 7, :]

    plot_trajectory(head_positions)
    plot_positions(times, head_positions)
    print(compute_velocity(links_positions, timestep= timestep))
    print(compute_energy(joints_torques, joints_velocities))



def exercise_8e2(timestep,duration):
    """Exercise 8e2"""

    times = np.arange(0, duration, timestep)

    '''

    "PART 1: Model with hydrodynamic forces" 

    i_ph_h = np.array([(16-n)*(2*np.pi)/16 for n in np.arange(16)])
    i_ph = np.concatenate((i_ph_h,np.zeros(4)))


    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            #amplitude_scaling = 2,
            initial_phases = i_ph,
            #initial_phases_rdn = 0.6,
            feedback_weight = 2, 
            amplitude_gradient = None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
        ) 
    ]

        # Grid search
    directory = './logs/exercise8e2'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8e2/simulation_{}.{}'
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
    joints_velocities = data.sensors.joints.velocities_all()
    joints_torques = data.sensors.joints.motor_torques_all()

    head_positions = links_positions[:, 0, :]
    tail_positions = links_positions[:, 7, :]

    plot_trajectory(head_positions)
    plot_positions(times, head_positions)
    print(compute_velocity(links_positions, timestep = timestep))
    print(compute_energy(joints_torques, joints_velocities))

    '''


    "PART 3: Model with correct initial phases + changing wbf"
    
    j = 10
    i = 7
    #i_ph_vec = [1,2,3,4,5,6,7]
    #i_ph_vec = [0.5*1e-4,1e-3, 0.5*1e-3, 1e-2, 0.5*1e-2, 0.1, 0.5]
    #i_ph_vec = np.linspace(0.1,1,7)
    i_ph_vec = np.linspace(0.1,2*np.pi,i)
    w_fb = np.linspace(0,3,j)
    print(i_ph_vec)
  
    
    parameter_set = [
        SimulationParameters(
            duration=20, # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            feedback_weight = wfb,
            #feedback_weight = 2,
            initial_phases_rnd = i_ph, 
            amplitude_gradient = None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
        )
        for wfb in w_fb
        #for i_ph in 0.6*np.ones(i)
        for i_ph in i_ph_vec
    ]


    velocities = np.zeros((j,i))
    energies = np.zeros((j,i))

        # Grid search
    directory = './logs/exercise8e2'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8e2/simulation_{}.{}'
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
        joints_velocities = data.sensors.joints.velocities_all()
        joints_torques = data.sensors.joints.motor_torques_all()

        amp_1 = simulation_i // i
        amp_2 = simulation_i % i 

        velocities[amp_1, amp_2] = compute_velocity(start_time = 0, pos = links_positions, timestep = timestep)
        energies[amp_1, amp_2] = compute_energy(joints_torques, joints_velocities)

    coordinates_velocities = convert_nd_matrix_to_nd_plot_coordinates(velocities,x_vals = w_fb, y_vals= i_ph_vec) 
    coordinates_energy = convert_nd_matrix_to_nd_plot_coordinates(energies,x_vals = w_fb, y_vals= i_ph_vec)
 
    plot_2d(coordinates_velocities, ("Feedback weight","Amplitude of random initial phases", "Velocity"))
    plot_2d(coordinates_energy, ("Feedback weight","Amplitude of random initial phases", "Energy"))

    

    '''

    "PART 4: Model comparison: open and closed loop"
    #REMARK: slose loop achieves the same speed (if not better) but using much lower energy !"
    k = 16
    #i_ph_h = np.linspace((16*2*np.pi)/k,(2*np.pi)/k,16)
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))


    parameter_set = [
    SimulationParameters(
        duration=duration,  # Simulation duration in [s]
        timestep=timestep,  # Simulation timestep in [s]
        spawn_position=[0, 0, 0.1],  # Robot position in [m]
        spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
        drive=4,  # An example of parameter part of the grid search
        updown_coupling_weight = udcw, 
        feedback_weight = wfb,
        #initial_phases = i_ph, 
        amplitude_gradient = None,  # Just an example
        phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
    )

    for udcw, wfb in zip(np.array([10,10,0]),np.array([0,2,2]))
    #initial phases ??? changes from closed to open ?
    ]


    velocities = np.zeros(3)
    energies = np.zeros(3)


        # Grid search
    directory = './logs/exercise8e2'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8e2/simulation_{}.{}'
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
        joints_velocities = data.sensors.joints.velocities_all()
        joints_torques = data.sensors.joints.motor_torques_all()

        velocities[simulation_i] = compute_velocity(links_positions, timestep = timestep)
        energies[simulation_i] = compute_energy(joints_torques, joints_velocities)
        
    print(velocities)
    print(energies)

    '''


if __name__ == '__main__':
    #exercise_8e1(timestep=1e-2,duration=10)
    exercise_8e2(timestep=1e-2,duration=10)
