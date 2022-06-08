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

        self.muted_sensors = np.ones(self.n_oscillators)
        self.muted_couplings = np.ones([self.n_oscillators, self.n_oscillators])
        self.muted_oscillators = np.ones(self.n_oscillators)

        # Set random seed
        if parameters.set_seed:
            np.random.seed(seed=parameters.randseed)
        else:
            pass

        affected_sensors_oneside = np.random.choice(8, size=parameters.n_disruption_sensors, replace=False)
        affected_sensors = np.concatenate((affected_sensors_oneside,affected_sensors_oneside+8))
        self.muted_sensors[affected_sensors] = 0

        affected_couplings = np.random.choice(7, size=parameters.n_disruption_couplings, replace=False)
        self.muted_couplings[affected_couplings, affected_couplings + 1] = 0
        self.muted_couplings[affected_couplings + 1,affected_couplings] = 0
        self.muted_couplings[affected_couplings + 8, affected_couplings + 9] = 0
        self.muted_couplings[affected_couplings + 9, affected_couplings + 8] = 0

        affected_oscillators_oneside = np.random.choice(8, size=parameters.n_disruption_oscillators, replace=False)
        affected_oscillators = np.concatenate((affected_oscillators_oneside,affected_oscillators_oneside+8))
        self.muted_oscillators[affected_oscillators] = 0

        self.feedback_weight = parameters.feedback_weight*self.muted_sensors

        #self.phase_lag_body = parameters.phase_lag_body
        #self.amplitude_scaling = parameters.amplitude_scaling
        #self.amplitude_gradient = parameters.amplitude_gradient
        #self.amplitude_gradient_scaling = parameters.amplitude_gradient_scaling
        self.update(parameters)


    def update(self, parameters):
        """Update network from parameters"""
        self.set_frequencies(parameters)  # f_i
        self.set_coupling_weights(parameters)  # w_ij
        self.set_phase_bias(parameters)  # theta_i
        self.set_amplitudes_rate(parameters)  # a_i
        self.set_nominal_amplitudes(parameters)  # R_i

    def step(self, iteration, salamandra_data):
        """Step function called at each iteration

        Parameters
        ----------

        salamanra_data: salamandra_simulation/data.py::SalamandraData
            Contains the robot data, including network and sensors.

        gps (within the method): Numpy array of shape [9x3]
            Numpy array of size 9x3 representing the GPS positions of each link
            of the robot along the body. The first index [0-8] coressponds to
            the link number from head to tail, and the second index [0,1,2]
            coressponds to the XYZ axis in world coordinate.

        """
        gps = np.array(
            salamandra_data.sensors.links.urdf_positions()[iteration, :9],
        )

    def set_frequencies(self, parameters):
        """Set frequencies"""


        # Induce turning
        parameters.drive_mlr += np.concatenate((np.ones(8) * parameters.turn, -np.ones(8) * parameters.turn, np.zeros(4)))

        for i in np.arange(self.n_oscillators_body):
            if (1.0 <= parameters.drive_mlr[i] <= 5.0):
               self.freqs[i] =  0.2*parameters.drive_mlr[i] + 0.3
            else: 
                self.freqs[i] = 0.0
        for i in np.arange(self.n_oscillators_body, self.n_oscillators): 
            if (1.0 <= parameters.drive_mlr[i] <= 3.0):
               self.freqs[i] =  0.2*parameters.drive_mlr[i] + 0.0
            else: 
                self.freqs[i] = 0.0

        self.freqs *= self.muted_oscillators

        return


    def set_coupling_weights(self, parameters):
        """Set coupling weights"""
        #pylog.warning('Coupling weights must be set')

        for i in np.arange(self.n_oscillators):

            #limb to limb links 
            if ((i == 16) or (i == 19)): 
                self.coupling_weights[i,17] = 10
                self.coupling_weights[i,18] = 10
            if ((i == 17) or (i == 18)): 
                self.coupling_weights[i,16] = 10
                self.coupling_weights[i,19] = 10

            #upward and downward links
            if ((not (i == 0)) and (not(i == 8)) and (i < 16)):  #downward link
                self.coupling_weights[i,i-1] = parameters.updown_coupling_weight
            if ((not (i == 7)) and (not(i == 15)) and (i < 16)):  #upward link
                self.coupling_weights[i,i+1] = parameters.updown_coupling_weight

            # colateral links
            if (i < 8):
                self.coupling_weights[i,8+i] = parameters.contra_coupling_weight
            if (8 <= i < 16):
                self.coupling_weights[i,i-8] = parameters.contra_coupling_weight

            #limb to body links
            if (0 <= i < 4):
                 self.coupling_weights[i,16] = 30
            if (4 <= i < 8):
                 self.coupling_weights[i,18] = 30
            if (8 <= i < 12):
                 self.coupling_weights[i,17] = 30
            if (12 <= i < 16):
                 self.coupling_weights[i,19] = 30

        self.coupling_weights *= self.muted_couplings
        return
        


    def set_phase_bias(self, parameters):
        """Set phase bias"""
        #pylog.warning('Phase bias must be set')

        for i in np.arange(self.n_oscillators):

            #limb to limb links 
            if ((i == 16) or (i == 19)): 
                self.phase_bias[i,17] = np.pi
                self.phase_bias[i,18] = np.pi
            if ((i == 17) or (i == 18)): 
                self.phase_bias[i,16] = np.pi
                self.phase_bias[i,19] = np.pi

            #upward and downward links
            if ((not (i == 0)) and (not (i == 8)) and (i < 16)):  #downward link
                self.phase_bias[i,i-1] = parameters.phase_lag_body
            if ((not (i == 7)) and (not(i == 15)) and (i < 16)):  #upward link
                self.phase_bias[i,i+1] = - parameters.phase_lag_body

            # colateral links
            if (i < 8):
                self.phase_bias[i,8+i] = np.pi
            if (8 <= i < 16): 
                self.phase_bias[i,i-8] = np.pi


            #limb to body links
            if (0 <= i < 4):
                    self.phase_bias[i,16] = np.pi
            if (4 <= i < 8):
                    self.phase_bias[i,18] = np.pi
            if (8 <= i < 12):
                    self.phase_bias[i,17] = np.pi
            if (12 <= i < 16):
                    self.phase_bias[i,19] = np.pi

        return

    def set_amplitudes_rate(self, parameters):
        """Set amplitude rates"""
        #pylog.warning('Convergence rates must be set')
        for i in np.arange(self.n_oscillators):
            self.rates[i] = 20.0
        #    if i >= 16:
        #        self.rates[i] = 20.0
        #    else:
        #        self.rates[i] = self.amplitude_gradient[i]

        #print(self.rates)
        return 

    def set_nominal_amplitudes(self, parameters):
        """Set nominal amplitudes"""

        #Induce turning
        parameters.drive_mlr += np.concatenate((np.ones(8)*parameters.turn, -np.ones(8)*parameters.turn, np.zeros(4)))

        for i in np.arange(self.n_oscillators_body): 
                if (1.0 <= parameters.drive_mlr[i] <= 5.0):
                    if parameters.amplitude_gradient is None : 
                        self.nominal_amplitudes[i] =  parameters.amplitude_scaling*(0.065*parameters.drive_mlr[i] + 0.196)

                    elif (parameters.amplitude_gradient_scaling == True):
                        self.nominal_amplitudes[i] =  parameters.amplitude_gradient[i]*(0.065*parameters.drive_mlr[i] + 0.196)

                    else: 
                        self.nominal_amplitudes[i] =  parameters.amplitude_gradient[i]
                else: 
                    self.nominal_amplitudes[i] = 0.0

        
        for i in np.arange(self.n_oscillators_body,self.n_oscillators):
            if (1.0 <= parameters.drive_mlr[i] <= 3.0):
                self.nominal_amplitudes[i] =  0.131*parameters.drive_mlr[i] + 0.131
            else: 
                self.nominal_amplitudes[i] = 0.0

        #print(self.nominal_amplitudes)
                
        return

    def set_feedback_gains(self, parameters):
        """Set feeback gains"""
        pylog.warning('Convergence rates must be set')


