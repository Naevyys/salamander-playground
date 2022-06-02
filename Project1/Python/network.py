"""Oscillator network ODE"""

import numpy as np
from scipy.integrate import ode
from robot_parameters import RobotParameters
from math import *


def network_ode(_time, state, robot_parameters, loads = np.zeros(12)):
    """Network_ODE

    Parameters
    ----------
    _time: <float>
        Time
    state: <np.array>
        ODE states at time _time
    robot_parameters: <RobotParameters>
        Instance of RobotParameters

    Returns
    -------
    :<np.array>
        Returns derivative of state (phases and amplitudes)

    """
    n_oscillators = robot_parameters.n_oscillators
    phases = state[:n_oscillators]
    amplitudes = state[n_oscillators:2*n_oscillators]


    # Import parameters
    freqs=robot_parameters.freqs
    weights=robot_parameters.coupling_weights
    phase_bias=robot_parameters.phase_bias
    nominal_amplitudes=robot_parameters.nominal_amplitudes
    rates=robot_parameters.rates

    # Implement equation here
    phases_repeated=np.tile(phases,(len(phases),1))
    #reshape_loads = np.concatenate((loads[:robot_parameters.n_body_joints],loads[:robot_parameters.n_body_joints],loads[robot_parameters.n_body_joints:]))
    reshape_loads = np.concatenate((-loads[:robot_parameters.n_body_joints],loads[:robot_parameters.n_body_joints],loads[robot_parameters.n_body_joints:]))
    #reshape_loads = np.concatenate((loads[:robot_parameters.n_body_joints],loads[robot_parameters.n_body_joints:],loads[robot_parameters.n_body_joints:],loads[robot_parameters.n_body_joints:]))

    dtheta=2*pi*freqs+np.sum(amplitudes*weights*np.sin(phases-phases_repeated.T-phase_bias),axis=1) + robot_parameters.feedback_weight*reshape_loads*np.cos(phases)
    dr=rates*(nominal_amplitudes-amplitudes)

    return np.concatenate([dtheta, dr])



def motor_output(phases, amplitudes, iteration):
    """Motor output

    Parameters
    ----------
    phases: <np.array>
        Phases of the oscillator
    amplitudes: <np.array>
        Amplitudes of the oscillator

    Returns
    -------
    : <np.array>
        Motor outputs for joint in the system.

    """
    # Implement equation here
    q = np.zeros(12)
    q[:8] = amplitudes[:8]*(1+np.cos(phases[:8])) - amplitudes[8:16]*(1+np.cos(phases[8:16]))
    q[8:] = phases[16:]
    return q


def x_output(phases,amplitudes):
    """x output

    Parameters
    ----------
    amplitudes: <np.array>
        Amplitudes of the oscillator

    Returns
    -------
    : <np.array>
        x outputs for all oscillators in the system.

    """
    x = amplitudes*(1+np.cos(phases))
    return x

def inst_freq(array = None):
    """Inst_freq


    Parameters
    ----------
    dtheta: <np.array>
    Derivates of the phases of the oscillator

    Returns
    -------
    : <np.array>
        Instantaneous frequencies

    """
    dtheta = array[:20]

    i_freq = dtheta/(2*pi)
    return i_freq




class SalamandraNetwork:
    """Salamandra oscillator network"""

    def __init__(self, sim_parameters, n_iterations, state):
        super().__init__()
        self.n_iterations = n_iterations
        # States
        self.state = state
        # Parameters
        self.robot_parameters = RobotParameters(sim_parameters)
        # Set initial state

        # Replace your oscillator phases here 
        
        if sim_parameters.initial_phases is not None: 
            self.state.set_phases(
                iteration=0,
                value=sim_parameters.initial_phases,
            )
        else: 
            self.state.set_phases(
                iteration=0,
                value=1e-4*np.random.ranf(self.robot_parameters.n_oscillators),
            )

        # Set solver
        self.solver = ode(f=network_ode)
        self.solver.set_integrator('dopri5')
        self.solver.set_initial_value(y=self.state.array[0], t=0.0)

    def step(self, iteration, time, timestep, loads):
        """Step"""

        if iteration + 1 >= self.n_iterations:
            return
        self.solver.set_f_params(self.robot_parameters, loads)
        self.state.array[iteration+1, :] = self.solver.integrate(time+timestep)  

    def outputs(self, iteration=None):
        """Oscillator outputs"""
        # Implement equation here
        return np.zeros(12)

    def get_motor_position_output(self, iteration=None):
        """Get motor position"""
        return motor_output(
            self.state.phases(iteration=iteration),
            self.state.amplitudes(iteration=iteration),
            iteration=iteration,
        )

    def get_x_output(self, iteration=None):
        """Get x for all oscillators"""
        return x_output(self.state.phases(iteration=iteration),
        self.state.amplitudes(iteration=iteration)
        )

    def get_inst_freq(self, iteration=None):
        """Get instantaneous frequency"""
        state = np.concatenate((self.state.phases(iteration=iteration),
                        self.state.amplitudes(iteration=iteration)))
        return inst_freq(array=network_ode(_time=0, state=state, robot_parameters=self.robot_parameters)
        )



