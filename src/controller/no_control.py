# -*- coding: utf-8 -*-

from . import Controller


class NoControl(Controller):
    """No control implemenation"""

    def __init__(self,
                 **kwargs):
        super().__init__(**kwargs)

    def update(self, v, omega, F):
        return (v, omega)
