import numpy as np
import pybullet as p

from . import Controller

class NoControl(Controller):
    def __init__(self,
                 **kwargs):
        super().__init__(**kwargs)
    
    def update(self, v, omega, F):
        """Update Control Velocity"""
        return (v, omega)
