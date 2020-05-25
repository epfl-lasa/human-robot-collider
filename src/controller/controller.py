import numpy as np

class Controller:    
    def __init__(self,
                 timestep=0.01,
                 bumper_r=0.33,
                 bumper_l=0.2425):
        self.Ts = timestep
        self.bumper_l = bumper_l
        self.bumper_r = bumper_r
        
    def update(self, v, omega, F):
        """Update Control Velocity"""
        raise NotImplementedError

    def get_location_on_bumper(self, Fx, Fy, Mz):
        h = 0
        (a, b, c) = (Fx, Fy, Mz/self.bumper_r)
        theta = np.real(-1j * np.log(
            (c + 1j*np.sqrt(a**2 + b**2 - c**2)) /
            (a + 1j*b)
        ))
        Fmag = Fx*np.sin(theta) + Fy*np.cos(theta)
        return (Fmag, h, theta)
    