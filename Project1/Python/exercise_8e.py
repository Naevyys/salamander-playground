"""Exercise 8e"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_trajectory, plot_positions
import matplotlib.pyplot as plt
from utils import compute_energy, compute_velocity


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

    '''Part 2: initial state'''

    k = 16
    i_ph_h = np.linspace((16*2*np.pi)/k,(2*np.pi)/k,16)
    i_ph = np.concatenate((i_ph_h,np.zeros(4)))

    times = np.arange(0, duration, timestep)

    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            initial_phases = i_ph,
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

    "PART 1: Model with hydrodynamic forces"
    '''
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            #amplitude_scaling = 2,
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

    #i_ph_h = np.array([(2*np.pi)/8, (2*np.pi)*2/8, (2*np.pi)*3/8, (2*np.pi)*4/8, (2*np.pi)*5/8, (2*np.pi)*6/8, (2*np.pi)*7/8, 0 ])
    #i_ph = np.concatenate((i_ph_h,i_ph_h,np.zeros(4)))

    #i_ph_h = np.array([(16-n)*(2*np.pi)/16 for n in np.arange(16)])
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))

    #i_ph_h = ((2*np.pi)/8)*np.ones(16)
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))

    #i_ph_h = ((2*np.pi)/8)*np.ones(8)
    #i_ph = np.concatenate((i_ph_h,2*i_ph_h,np.zeros(4)))

    #i_ph = np.array([(20-n)*(2*np.pi)/20 for n in np.arange(20)])

    #i_ph_h = ((2*np.pi)/16)*np.ones(8)
    #i_ph = np.concatenate((i_ph_h,8*i_ph_h,np.zeros(4)))

    #i_ph = np.array([(20-n)*(2*np.pi)/20 for n in np.arange(20)])

    #k = 16
    #i_ph_h = np.linspace((16*2*np.pi)/k,(2*np.pi)/k,16)
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))

    '''
    Remarks so far: 
    - when random initialisation of init_phases even with very small amplitude --> see it moving, but only a little (like spped is slow 1 direction is random)
    - when use a phase gradient but the init_phase are identical modulo 8, doesn't work ! 
    - when use a phase gradient over the 16 oscillator works fast ! but turn a littlebit
    - when use a cst value over the 16 oscillators, doesn't work ! 
    - when use a cts value over 8 and then change for the other 8, swims but in aweird way + slowly
    - when use a phase gradient over the 20 oscillator works fast ! not as fast as the 16 (i.e. phase difference btw consecutive are smaller)
    - when use a cts value over 8 and then change for the other 8, for bigger phase difference in adj joint swims but in aweird way + slowly
    - pour le gradient, plus on augmente la difference de phase (i.e plus on reduit le denominateur, plus elle va vite)

    PROBLEM, IN FORUM IS SAID: the initialization of the phases should not affect the capability of the model to produce movement in general
    PROBLEM; THE change in weighth doesn't seem to change anything, just energy, no effect on speed ! so even with 0 feedback weights... ! 
    '''
    '''
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            feedback_weight = 2,
            initial_phases = i_ph, 
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
            #fast=True,  # For fast mode (not real-time)
            #headless=True,  # For headless mode (No GUI, could be faster)
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
    #k = 16
    #i_ph_h = np.linspace((16*2*np.pi)/k,(2*np.pi)/k,16)
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))
    j = 10

    
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            updown_coupling_weight = 0, 
            feedback_weight = wfb,
            #initial_phases = i_ph, 
            amplitude_gradient = None,  # Just an example
            phase_lag_body=((2 * np.pi) / 8),  # or np.zeros(n_joints) for example
        ) 
        for wfb in np.linspace(0,10,j)
    ]


    velocities = np.zeros(j)
    energies = np.zeros(j)

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
    print(np.linspace(0,5,j))



    "PART 4: Model comparison: open and closed loop"
    #REMARK: slose loop achieves the same speed (if not better) but using much lower energy !"
    k = 16
    #i_ph_h = np.linspace((16*2*np.pi)/k,(2*np.pi)/k,16)
    #i_ph = np.concatenate((i_ph_h,np.zeros(4)))

    '''

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
