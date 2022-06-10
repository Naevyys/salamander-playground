"""Simulation parameters"""
import numpy as np


class SimulationParameters:
    """Simulation parameters"""

    def __init__(self, **kwargs):
        super(SimulationParameters, self).__init__()
        # Default parameters
        self.n_body_joints = 8
        self.n_legs_joints = 4
        self.duration = 30
        self.initial_phases = None                #vector of size 20
        self.initial_phases_rdn = None            #scalar
        self.phase_lag_body = ((2*np.pi)/8)
        self.contra_coupling_weight=10
        self.updown_coupling_weight=10
        self.amplitude_gradient = None
        self.amplitude_scaling = 1
        self.amplitude_gradient_scaling = False
        self.feedback_weight = 0
        # Feel free to add more parameters (ex: MLR drive)
        self.drive = 0
        self.turn = 0
        # ...

        # Disruptions
        self.set_seed = False
        self.randseed = 0
        self.n_disruption_couplings = 0
        self.n_disruption_oscillators = 0
        self.n_disruption_sensors = 0

        # Update object with provided keyword arguments
        # NOTE: This overrides the previous declarations
        self.__dict__.update(kwargs)
        self.drive_mlr = self.drive*np.ones(2*self.n_body_joints + self.n_legs_joints)

        if kwargs["amplitude_gradient"] is not None:
            self.amplitude_gradient = np.linspace(kwargs["amplitude_gradient"][0], kwargs["amplitude_gradient"][1], num=8)
            self.amplitude_gradient = np.concatenate([self.amplitude_gradient, self.amplitude_gradient])
        #print(self.phase_lag_body)


