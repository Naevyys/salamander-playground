"""Robot parameters"""

import numpy as np
from farms_core import pylog


class RobotParameters(dict):
    """Robot parameters"""

    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__

    def __init__(self, parameters):
        super(RobotParameters, self).__init__()

        # Initialise parameters
        self.n_body_joints = parameters.n_body_joints
        self.n_legs_joints = parameters.n_legs_joints
        self.initial_phases = parameters.initial_phases
        self.n_joints = self.n_body_joints + self.n_legs_joints
        self.n_oscillators_body = 2*self.n_body_joints
        self.n_oscillators_legs = self.n_legs_joints
        self.n_oscillators = self.n_oscillators_body + self.n_oscillators_legs
        self.freqs = np.zeros(self.n_oscillators)
        self.coupling_weights = np.zeros([
            self.n_oscillators,
            self.n_oscillators,
        ])
        self.phase_bias = np.zeros([self.n_oscillators, self.n_oscillators])
        self.rates = np.zeros(self.n_oscillators)
        self.nominal_amplitudes = np.zeros(self.n_oscillators)
        self.feedback_gains = np.zeros(self.n_oscillators)

        self.update(parameters)

    def update(self, parameters):
        """Update network from parameters"""
        self.set_frequencies(parameters)  # f_i
        self.set_coupling_weights(parameters)  # w_ij
        self.set_phase_bias(parameters)  # phi_ij
        self.set_amplitudes_rate(parameters)  # a_i
        self.set_nominal_amplitudes(parameters)  # R_i
        self.set_feedback_gains(parameters)  # K_fb

    def set_frequencies(self, parameters):
        """Set frequencies"""
        assert parameters.freqs.shape[0] == self.n_oscillators, \
            "Length of frequencies does not match number of oscillators!"
        self.freqs = parameters.freqs

    def set_coupling_weights(self, parameters):
        """Set coupling weights"""
        assert parameters.coupling_weights.shape[0] == self.n_oscillators and \
            parameters.coupling_weights.shape[1] == self.n_oscillators, \
            "Shape of coupling weights does not match the number of oscillators!"
        self.coupling_weights = parameters.coupling_weights

    def set_phase_bias(self, parameters):
        """Set phase bias"""
        assert parameters.phase_bias.shape[0] == self.n_oscillators and \
               parameters.phase_bias.shape[1] == self.n_oscillators, \
               "Shape of phase bias does not match the number of oscillators!"

        self.phase_bias = parameters.phase_bias

    def set_amplitudes_rate(self, parameters):
        """Set amplitude rates"""
        assert parameters.rates.shape[0] == self.n_oscillators, \
            "Length of amplitude rates does not match the number of oscillators!"
        self.rates = parameters.rates

    def set_nominal_amplitudes(self, parameters):
        """Set nominal amplitudes"""
        assert parameters.nominal_amplitudes.shape[0] == self.n_oscillators, \
            "Length of nominal amplitudes does not match the number of oscillators!"
        self.nominal_amplitudes = parameters.nominal_amplitudes

    def set_feedback_gains(self, parameters):
        """Set feeback gains"""
        assert parameters.feedback_gain.shape[0] == self.n_oscillators, \
            "Length of feedback gain does not match the number of oscillators!"
        self.feedback_gains = parameters.feedback_gain

