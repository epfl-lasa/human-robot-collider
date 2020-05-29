# -*- coding: utf-8 -*-

import numpy as np


class Controller:
    """Base Class for Controllers

    Parameters
    ----------
    timestep : float, optional
        timestep of controller, by default 0.01
    bumper_r : float, optional
        radius of bumper, by default 0.33
    bumper_l : float, optional
        location of bumper from COM, by default 0.2425
    """
    def __init__(self,
                 timestep=0.01,
                 bumper_r=0.33,
                 bumper_l=0.2425):
        self.Ts = timestep
        self.bumper_l = bumper_l
        self.bumper_r = bumper_r

    def update(self, v, omega, F):
        """Get updated desired velocity with compliance in mind.

        Parameters
        ----------
        v : float
            Original linear velocity
        omega : float
            Original angular velocity
        F : ndarray
            Array containing contact forces and moments. Order of force and moments is [Fx, Fy, Fz, Mx, My, Mz]

        Returns
        -------
        tuple(float, float)
            Tuple containing linear and rotational velocity after compliant control
        """
        raise NotImplementedError

    def get_location_on_bumper(self, Fx, Fy, Mz):
        """Get collision force and location on bumper.

        Parameters
        ----------
        Fx : float
            Force in X-direction (Towards left)
        Fy : float
            Force in Y-direction (Towards back)
        Mz : float
            Moment in Z-direction

        Returns
        -------
        tuple(float, float, float)
            Tuple containing
                - Fmag, Magnitude of collision force perpendicular to bumper
                - h, Height of the collision point
                - theta, Angle on bumper of the collision point
        """
        h = 0
        (a, b, c) = (Fx, Fy, Mz/self.bumper_r)
        theta = np.real(-1j * np.log(
            (c + 1j*np.sqrt(a**2 + b**2 - c**2))
            / (a + 1j*b)
        ))
        Fmag = Fx*np.sin(theta) + Fy*np.cos(theta)
        return (Fmag, h, theta)
