# -*- coding: utf-8 -*-

from . import Controller
import numpy as np

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
    		return self.__control(v_prev, omega_prev, v_cmd, omega_cmd, nominal_robot_speed, Speed, deformation)
    	else:
    		self.V_contact = np.nan
    		self._theta = np.nan
    		return (v_cmd, omega_cmd)
    def __control(self, v_prev, omega_prev, v_cmd, omega_cmd, nominal_robot_speed, speed, deformation):
        """Get new velocity
		F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x + f + s, f: friction, s:static force = 0
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
        tuple(float, float)
            Tuple containing linear and rotational velocity after compliant control
        """
        friction_coef = 0.004
        """
        if (nominal_robot_speed == 1):
	        if (v_prev>=0.0 and v_prev<=0.7):
	        	v_prev = 0.94*v_prev
	        	#acc = 0
	        	acc =  0.04*(- self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
	        #elif (v_prev>0.0 and v_prev < 0.0):
	        #	acc =  0.12*(- self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
	        else:
	        	acc =  (- 1.0*self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
        elif(nominal_robot_speed ==1.5):
        	if (v_prev>=0.34 and v_prev<=0.8):
	        	v_prev = 0.85*v_prev
	        	acc =  0.01*(- self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
	        elif (v_prev>0.1 and v_prev < 0.34):
	        	acc =  0.215*(- self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
	        else:
	        	acc =  (- 1.0*self._Fmag - friction_coef * self.robot_mass*9.81)/self.robot_mass
        else:
        """
        #For head change 75 to 5 and 208 to 138
        acc =  (-0.2*v_prev - 1.0*self._Fmag - friction_coef * self.robot_mass*9.81)/(self.robot_mass*75/208)
        v = acc*0.00005 + v_prev 
        omega = omega_cmd
        self.V_contact  = v
        return (v, omega_cmd)

    def __differential_to_cartesian(self, v, omega):
        return (self.jacobian @ np.array([v, omega]))

    def __cartesian_to_differential(self, vx, vy):
        return (self.inv_jacobian @ np.array([vx, vy]))
