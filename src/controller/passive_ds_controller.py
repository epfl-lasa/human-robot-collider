# -*- coding: utf-8 -*-

import numpy as np

from . import Controller


class PassiveDSController(Controller):
    """Passive DS Controller

    Attributes
    ----------
    damping_gain : float
        Damping gain to use
    robot_mass : float
        Effective mass of the robot.

    Notes
    -----
    Based on Passive DS
    """

    def __init__(self,
                 Mx=10,
                 My=10,
                 Cx=10,
                 Cy=10,
                 lambda_x=1,
                 lambda_y=1,
                 F_d=45,
                 **kwargs):
        super().__init__(**kwargs)
        self.M = np.diag([Mx, My])
        self.C = np.diag([Cx, Cy])
        self.Lambda = np.diag([lambda_x, lambda_y])
        self.F_d = F_d

        # Internal State Variables
        self._Fmag = 0.0
        self._theta = 0.0
        self._h = 0.0
        self._Fx = 0.0
        self._Fy = 0.0
        self._Mz = 0.0
        self._D = self.Lambda

        self.V_contact = 0.0

    def update(self, F, v_prev, omega_prev, v_cmd, omega_cmd):
        (self._Fx, self._Fy, self._Mz) = (F[0], F[1], F[5])
        self.get_location_on_bumper(self._Fx, self._Fy, self._Mz)
        if self._Fmag > self.activation_F:
            return self.__control(v_prev, omega_prev, v_cmd, omega_cmd)
        else:
            self.V_contact = np.nan
            self._theta = np.nan
            return (v_cmd, omega_cmd)

    def __control(self, v_prev, omega_prev, v_cmd, omega_cmd):
        """Get new velocity

        Parameters
        ----------
        v_prev : float
            Prev linear velocity
        omega_prev : float
            Prev rotational velocity
        v_prev : float
            Demand linear velocity
        omega_prev : float
            Demand rotational velocity

        Returns
        -------
        tuple(float, float)
            Tuple containing linear and rotational velocity after compliant control
        """
        v = v_cmd
        omega = omega_cmd

        return (v, omega)

    def __map(self, x, from_range, to_range):
        return (to_range[0]
                + ((x - from_range[0])
                   * (to_range[1] - to_range[0])
                   / (from_range[1] - from_range[0])))
