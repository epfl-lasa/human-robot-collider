import numpy as np
import pybullet as p

class AdmitanceController:
    # Parameters
    bumper_l = 0.2425   # (210+32.5) mm
    bumper_r = 0.33     # 330 mm
    Ts = 1.0/100        # 100 Hz
    damping_gain = 1    # 1 N-s/m 
    robot_mass = 120    # 120 kg
    
    def __init__(self):
        pass
    
    def update(self, v_prev, omega_prev, Fmag, h, theta):
        """Update Control Velocity
        
        $$ F = robot_mass \Delta \ddot{x} + Damping_gain \Delta \dot{x} + K \Delta x $$
        
        * Reference is set to 0 and discretised with ZOH
        """        
        stheta = np.sin(theta)    # Small optimization
        ctheta = np.cos(theta)    # Small optimization

        # Position wrt center of rotatiion
        R = np.sqrt(
            (self.bumper_r*stheta)**2 +
            (self.bumper_l + self.bumper_r*ctheta)**2
        )
        beta = np.atan2(self.bumper_r * stheta, self.bumper_l)

        sbeta = np.sin(beta)      # Small optimization
        cbeta = np.cos(beta)      # Small optimization
        
        a = ctheta
        b = R*(stheta*cbeta - ctheta*sbeta)

        # Admittance Control
        v_eff_prev = (a * v_prev) + (b * omega_prev)

        v_eff_dot = (-Fmag - self.Damping_gain*v_eff_prev) / self.robot_mass
        v_eff = v_eff_dot * Ts + v_eff_prev

        # # Calculate new v and omega
        # c_prev = (-b * v_prev) + (a * omega_prev)
        # den = a**2 - b**2
        # v = (a*v_eff - b*c_prev) / den
        # omega = (-b*v_eff + a*c_prev) / den

        # Ensure non-zero 'a' and 'b'
        eps = 0.01
        if (a < eps):
            return (v_prev, v_eff/b)
        if (b < eps):
            return (v_eff/a, omega_prev)

        # Calculate new v and omega in parameterized form
        t = 0.5     # \in [0,1]
        v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
        omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

        return (v, omega)

