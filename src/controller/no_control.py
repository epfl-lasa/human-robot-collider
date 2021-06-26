# -*- coding: utf-8 -*-

from . import Controller
import numpy as np
#Mass definition
H_mass = 75

timestep = 0.00005
acc_factor = 1
class NoControl(Controller):
    """No control implemenation"""

    def __init__(self,
    			 damping_gain=0.2,
                 robot_mass=133,
                 lambda_t=0.0,
                 lambda_n=0.5,
                 Fd=45,
                 activation_F=15,
                 **kwargs):
        super().__init__(**kwargs)
        self.damping_gain = damping_gain
        self.robot_mass = robot_mass
        self.activation_F = activation_F
        self.Lambda = np.diag([lambda_t, lambda_n])
        self.Fd = Fd
        self.D = self.Lambda

        # Internal State Variables
        self._Fmag = 0.0
        self._theta = 0.0
        self._h = 0.0
        self._Fx = 0.0
        self._Fy = 0.0
        self._Mz = 0.0

        self.V_contact = 0.0

    def update(self,nominal_robot_speed,Speed,F,v_prev, omega_prev, v_cmd, omega_cmd,deformation):
    	(self._Fx, self._Fy, self._Mz) = (F[0], F[1], F[5])
    	self.get_location_on_bumper(self._Fx, self._Fy, self._Mz)
    	if self._Fmag > self.activation_F:
    		return self.__control(v_prev, omega_prev, v_cmd, omega_cmd)
    	else:
    		self.V_contact = np.nan
    		self._theta = np.nan
    		return (v_cmd, omega_cmd), 0 
    def __control(self, v_prev, omega_prev, v_cmd, omega_cmd):
        """Get new velocity
        Parameters
        ----------
        v_prev : float
            Prev linear velocity
        omega_prev : float
            Prev rotational velocity
        v_cmd : float
            Demand linear velocity
        omega_cmd : float
            Demand rotational velocity
		
        Returns
        -------
        tuple(float, float), float
            Tuple containing linear and rotational velocity after control, acceleration values at every time step
        """
        friction_coef = 0.004

        acc =  (- 1.0*self._Fmag - friction_coef * self.robot_mass*9.81)/(self.robot_mass*H_mass/(self.robot_mass+H_mass))
        v = acc_factor*acc*timestep + v_prev 

       	#Placeholder values for omega and V_contact. Adapt for more elaborate model
        omega = omega_cmd
        self.V_contact  = v

        return (v, omega_cmd), acc

    def __differential_to_cartesian(self, v, omega):
        return (self.jacobian @ np.array([v, omega]))

    def __cartesian_to_differential(self, vx, vy):
        return (self.inv_jacobian @ np.array([vx, vy]))
