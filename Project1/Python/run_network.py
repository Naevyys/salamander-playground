"""Run network without MuJoCo"""

import time
import numpy as np
import matplotlib.pyplot as plt
from farms_core import pylog
from salamandra_simulation.data import SalamandraState
from salamandra_simulation.parse_args import save_plots
from salamandra_simulation.save_figures import save_figures
from simulation_parameters import SimulationParameters
from network import SalamandraNetwork


def run_network(duration, update=False, drive=0, R_head = 0,R_tail = 0):
    """Run network without MuJoCo and plot results
    Parameters
    ----------
    duration: <float>
        Duration in [s] for which the network should be run
    update: <bool>
        description
    drive: <float/array>
        Central drive to the oscillators
    """

    # Simulation setup
    timestep = 1e-2
    times = np.arange(0, duration, timestep)
    drivedt=(5/40)*times+0.5
    n_iterations = len(times)
    sim_parameters = SimulationParameters(
        drive=drive,
        #amplitudes =[R_head,R_tail],
        amplitudes= 20.0,
        phase_lag_body= ((2*np.pi)/8),
        turn=0,
    )
    state = SalamandraState.salamandra_robotica_2(n_iterations)
    network = SalamandraNetwork(sim_parameters, n_iterations, state)
    osc_left = np.arange(8)
    osc_right = np.arange(8, 16)
    osc_legs = np.arange(16, 20)

    # Logs
    phases_log = np.zeros([
        n_iterations,
        len(network.state.phases(iteration=0))
    ])
    phases_log[0, :] = network.state.phases(iteration=0)
    amplitudes_log = np.zeros([
        n_iterations,
        len(network.state.amplitudes(iteration=0))
    ])
    amplitudes_log[0, :] = network.state.amplitudes(iteration=0)
    freqs_log = np.zeros([
        n_iterations,
        len(network.robot_parameters.freqs)
    ])
    freqs_log[0, :] = network.robot_parameters.freqs
    nomamp_log = np.zeros([
        n_iterations,
        len(network.robot_parameters.nominal_amplitudes)
    ])
    nomamp_log[0, :] = network.robot_parameters.nominal_amplitudes
    instfreq_log = np.zeros([
        n_iterations,
        len(network.state.amplitudes(iteration=0))
    ])
    instfreq_log[0, :] = network.get_inst_freq(iteration=0)
    x_outputs_log = np.zeros([
        n_iterations,
        len(network.get_x_output(iteration=0))

    ])
    x_outputs_log[0, :] = network.get_x_output(iteration=0)
    outputs_log = np.zeros([
        n_iterations,
        len(network.get_motor_position_output(iteration=0))

    ])
    outputs_log[0, :] = network.get_motor_position_output(iteration=0)

    # Run network ODE and log data
    tic = time.time()
    for i, time0 in enumerate(times[1:]):
        if update:
            network.robot_parameters.update(
                SimulationParameters(
                    drive = drivedt[i],
                    #amplitudes=[R_head,R_tail],
                    amplitudes= 20.0,
                    phase_lag_body= ((2*np.pi)/8),
                    
                )
            )
        network.step(i, time0, timestep, loads = None)
        phases_log[i+1, :] = network.state.phases(iteration=i+1)
        amplitudes_log[i+1, :] = network.state.amplitudes(iteration=i+1)
        x_outputs_log[i+1, :] = network.get_x_output(iteration=i+1)
        outputs_log[i + 1, :] = network.get_motor_position_output(iteration=i + 1)
        freqs_log[i+1, :] = network.robot_parameters.freqs
        nomamp_log[i+1, :] = network.robot_parameters.nominal_amplitudes
        instfreq_log[i + 1, :] = network.get_inst_freq(iteration=i+1)

    # # Alternative option
    # phases_log[:, :] = network.state.phases()
    # amplitudes_log[:, :] = network.state.amplitudes()
    # outputs_log[:, :] = network.get_motor_position_output()
    toc = time.time()

    # Network performance
    pylog.info('Time to run simulation for {} steps: {} [s]'.format(
        n_iterations,
        toc - tic
    ))

    # Implement plots of network results
    #pylog.warning('Implement plots')

    #'''
    #For coupled oscillators 

    fig,ax=plt.subplots(4)
    ax[0].plot(times, x_outputs_log[:, 0]+1)
    ax[0].plot(times, x_outputs_log[:, 4]-1)
    ax[0].set_ylabel('x Body')
    ax[1].plot(times,x_outputs_log[:,-4] + 1)
    ax[1].plot(times,x_outputs_log[:,-3] - 1)
    ax[1].set_ylabel('x Limb')
    ax[2].plot(times,instfreq_log)
    ax[2].set_ylabel('Freqs [Hz]')
    ax[3].plot(times,drivedt)
    ax[3].set_ylabel('drive d')
    ax[3].set_xlabel('Time [s]')
    #print(instfreq_log[:,-1])


    fig0,ax0=plt.subplots()
    ax0.plot(times,outputs_log[:,0])
    ax0.set_ylabel('q')
    ax0.set_xlabel('Time [s]')
    ax0.set_title('Spinal joint angle 1')

    
    '''
    #For independnet oscillators i.e. 0 weights

    fig1p1,ax1p1=plt.subplots(4)
    ax1p1[0].plot(times,x_outputs_log)
    ax1p1[0].set_ylabel('x')
    ax1p1[1].plot(times,instfreq_log)
    ax1p1[1].set_ylabel('Freq [Hz]')
    ax1p1[2].plot(times,amplitudes_log)
    ax1p1[2].set_ylabel('r')
    ax1p1[3].plot(times,drivedt)
    ax1p1[3].set_ylabel('drive d')
    ax1p1[3].set_xlabel('Time [s]')
    ax1p1[0].set_title('For all oscillators')

    fig1p2,ax1p2=plt.subplots(2)
    ax1p2[0].plot(drivedt,freqs_log)
    ax1p2[0].set_ylabel('v [Hz]')
    ax1p2[1].plot(drivedt,nomamp_log)
    ax1p2[1].set_ylabel('R')
    ax1p2[1].set_xlabel('drive d')
    ax1p2[0].set_title('For all oscillators')
    
    '''
    
    

def main(plot):
    """Main"""

    run_network(duration=40, update=True,drive=0.5)
    # Show plots
    if plot:
        plt.show()
    else:
        save_figures()


if __name__ == '__main__':
    main(plot=not save_plots())

