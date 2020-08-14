"""Robot Module.

This module implements model for different robots.
"""

__all__ = ["Robot", "Qolo", "Wheelchair"]
__version__ = '0.1'
__author__ = 'Vaibhav Gupta'


# Exports
from .robot import Robot
from .qolo.qolo import Qolo
from .wheelchair.wheelchair import Wheelchair
