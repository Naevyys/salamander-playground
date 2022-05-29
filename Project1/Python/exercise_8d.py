"""Exercise 8d"""

import os
import pickle
import numpy as np
from salamandra_simulation.simulation import simulation
from simulation_parameters import SimulationParameters
from plot_results import plot_trajectory, plot_positions
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.offsetbox
from matplotlib.lines import Line2D

# Define scale indicator: https://stackoverflow.com/questions/70895390/insert-scale-bar-inside-a-plot
class AnchoredHScaleBar(matplotlib.offsetbox.AnchoredOffsetbox):
    """ size: length of bar in data units
        extent : height of bar ends in axes units """
    def __init__(self, size=1, extent = 0.03, label="", loc=2, ax=None,
                 pad=0.4, borderpad=0.5, ppad = 0, sep=2, prop=None,
                 frameon=True, linekw={}, **kwargs):
        if not ax:
            ax = plt.gca()
        trans = ax.get_yaxis_transform()
        size_bar = matplotlib.offsetbox.AuxTransformBox(trans)
        line = Line2D([0,0],[size,0], **linekw)
        hline1 = Line2D([-extent/2.,extent/2.],[0,0], **linekw)
        hline2 = Line2D([-extent/2.,extent/2.],[size,size], **linekw)
        size_bar.add_artist(line)
        size_bar.add_artist(hline1)
        size_bar.add_artist(hline2)


        txt = matplotlib.offsetbox.TextArea(label, minimumdescent=False)
        self.vpac = matplotlib.offsetbox.VPacker(children=[size_bar,txt],
                                 align="center", pad=ppad, sep=sep)
        matplotlib.offsetbox.AnchoredOffsetbox.__init__(self, loc, pad=pad,
                 borderpad=borderpad, child=self.vpac, prop=prop, frameon=frameon,
                 **kwargs)

def exercise_8d1(timestep,duration):
    """Exercise 8d1"""

    times = np.arange(0, duration, timestep)

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
            turn=0.5,  # Another example
            # ...
        )
        # for drive in np.linspace(3, 4, 2)
        # for amplitudes in ...
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8d1'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8d1/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
            fast=True,  # For fast mode (not real-time)
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

    plt.rcParams.update({'font.size': 16})

    plt.figure()
    plot_trajectory(head_positions)
    plt.savefig('8d1_trajectory.pdf', bbox_inches='tight')





    osc_phases = data.state.phases()
    osc_amplitudes = data.state.amplitudes()

    joint_angles = np.zeros((len(times), 8))
    joint_angles[:, :] = osc_amplitudes[:, :8] * (1 + np.cos(osc_phases[:, :8])) - osc_amplitudes[:, 8:16] * (
                1 + np.cos(osc_phases[:, 8:16]))

    ob = AnchoredHScaleBar(size=np.pi/(3*5), label=r"$\pi$/3", loc=2, frameon=False,
                           pad=1, sep=4, linekw=dict(color="black"), )  # For some reason, the scale bar is 5 times as long as intended. Hence, the size is divided by 5

    fig = plt.figure()
    ax = fig.gca()
    ax.plot(times, joint_angles - np.arange(8))
    ax.legend([r'$q_1$',r'$q_2$',r'$q_3$',r'$q_4$',r'$q_5$',r'$q_6$',r'$q_7$',r'$q_8$'],loc="lower right")
    ax.add_artist(ob)
    ax.set_yticks([])
    ax.set_ylabel('Joint angles')
    ax.set_xlabel('Time [s]')

    plt.savefig('8d1_jointangles.pdf', bbox_inches='tight')





def exercise_8d2(timestep,duration):
    """Exercise 8d2"""

    times = np.arange(0, duration, timestep)

    # Parameters
    parameter_set = [
        SimulationParameters(
            duration=duration,  # Simulation duration in [s]
            timestep=timestep,  # Simulation timestep in [s]
            spawn_position=[0, 0, 0.1],  # Robot position in [m]
            spawn_orientation=[0, 0, 0],  # Orientation in Euler angles [rad]
            drive=4,  # An example of parameter part of the grid search
            amplitude_gradient=None,  # Just an example
            phase_lag_body=-((2*np.pi)/8),  # or np.zeros(n_joints) for example
            turn=0,  # Another example
            # ...
        )
        #for drive in np.linspace(3, 4, 2)
        # for amplitudes in ...
        # for ...
    ]

    # Grid search
    directory = './logs/exercise8d2'
    os.makedirs(directory, exist_ok=True)
    for f in os.listdir(directory):
        os.remove(os.path.join(directory, f))  # Delete all existing files before running the new simulations
    for simulation_i, sim_parameters in enumerate(parameter_set):
        filename = './logs/exercise8d2/simulation_{}.{}'
        sim, data = simulation(
            sim_parameters=sim_parameters,  # Simulation parameters, see above
            arena='water',  # Can also be 'ground', give it a try!
            fast=True,  # For fast mode (not real-time)
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

    plt.figure()
    plot_trajectory(head_positions)
    plt.savefig('8d2_trajectory.pdf', bbox_inches='tight')

    osc_phases = data.state.phases()
    osc_amplitudes = data.state.amplitudes()

    joint_angles=np.zeros((len(times),8))
    joint_angles[:,:] = osc_amplitudes[:,:8]*(1+np.cos(osc_phases[:,:8])) - osc_amplitudes[:,8:16]*(1+np.cos(osc_phases[:,8:16]))

    ob = AnchoredHScaleBar(size=np.pi / (3*5), label=r"$\pi$/3", loc=2, frameon=False,
                           pad=1, sep=4, linekw=dict(color="black"), )

    fig = plt.figure()
    ax = fig.gca()
    ax.plot(times, joint_angles-np.arange(8))
    ax.legend([r'$q_1$',r'$q_2$',r'$q_3$',r'$q_4$',r'$q_5$',r'$q_6$',r'$q_7$',r'$q_8$'],loc="lower right")
    ax.add_artist(ob)
    ax.set_yticks([])
    ax.set_ylabel('Joint angles')
    ax.set_xlabel('Time [s]')

    plt.savefig('8d2_jointangles.pdf', bbox_inches='tight')
    plt.show()



if __name__ == '__main__':
    exercise_8d1(timestep=1e-2,duration=10)
    exercise_8d2(timestep=1e-2,duration=10)

