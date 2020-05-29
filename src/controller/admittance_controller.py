# -*- coding: utf-8 -*-

import numpy as np

from . import Controller


class AdmittanceController(Controller):
    """Admittance ControllerS

    Attributes
    ----------
    damping_gain : float
        Damping gain to use
    robot_mass : float
        Effective mass of the robot.

    Notes
    -----
    Implements compliant controller based on following control equation with reference set to 0
    and discretised with ZOH.
    .. math:: F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x

    """

    def __init__(self,
                 damping_gain=1,
                 robot_mass=54.5,
                 **kwargs):
        super().__init__(**kwargs)
        self.damping_gain = damping_gain
        self.robot_mass = robot_mass

    def update(self, v, omega, F):
        (Fmag, h, theta) = self.get_location_on_bumper(Fx=F[0], Fy=F[1], Mz=F[5])
        return self.__control(v, omega, Fmag, theta)

    def __control(self, v_prev, omega_prev, Fmag, theta):
        """Get new velocity

        Parameters
        ----------
        v_prev : float
            Demand linear velocity
        omega_prev : float
            Demand rotational velocity
        Fmag : float
            Perpendicular force at point of contact
        theta : float
            Contact point location on the bumper

        Returns
        -------
        tuple(float, float)
            Tuple containing linear and rotational velocity after compliant control
        """
        stheta = np.sin(theta)    # Small optimization
        ctheta = np.cos(theta)    # Small optimization

        # Position wrt center of rotatiion
        R = np.sqrt((self.bumper_r*stheta)**2
                    + (self.bumper_l + self.bumper_r*ctheta)**2)
        beta = np.arctan2(self.bumper_r * stheta, self.bumper_l)

        sbeta = np.sin(beta)      # Small optimization
        cbeta = np.cos(beta)      # Small optimization

        a = ctheta
        b = R*(stheta*cbeta - ctheta*sbeta)

        # Admittance Control
        v_eff_prev = (a * v_prev) + (b * omega_prev)

        v_eff_dot = (-Fmag - self.damping_gain*v_eff_prev) / self.robot_mass
        v_eff = v_eff_dot * self.Ts + v_eff_prev

        # # Calculate new v and omega
        # c_prev = (-b * v_prev) + (a * omega_prev)
        # den = a**2 - b**2
        # v = (a*v_eff - b*c_prev) / den
        # omega = (-b*v_eff + a*c_prev) / den

        # Ensure non-zero 'a' and 'b'
        eps = 0.01
        if (abs(a) < eps):
            return (v_prev, v_eff/b)
        if (abs(b) < eps):
            return (v_eff/a, omega_prev)

        # Calculate new v and omega in parameterized form
        t = self.__get_t(theta)     # \in [0,1]
        v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
        omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

        return (v, omega)

    def __get_t(self, theta):
        """Returns tuning parameter based on angle
        """
        return self.__map(np.abs(theta), 0.0, np.pi, 1.0, 0.0)

    def __map(self, x, from_lower, from_upper, to_lower, to_upper):
        return to_lower + ((x - from_lower) * (to_upper - to_lower) / (from_upper - from_lower))
