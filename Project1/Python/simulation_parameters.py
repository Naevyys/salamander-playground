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
        self.initial_phases = None
        self.phase_lag_body = None
        self.amplitude_gradient = None
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


