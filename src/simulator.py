import time
import os
import logging

import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np

from collision import Collision


def pos_atan(y, x):
    a = np.arctan2(y, x)
    if a < 0.0:
        a += 2*np.pi
    return a


def reset_walker_case_4(walker, distance, robot_angle, human_angle, gait_phase):
    x = distance*np.cos(-np.pi/2-robot_angle)
    y = distance*np.sin(-np.pi/2-robot_angle)
    orientation = -np.pi/2-robot_angle + human_angle
    walker.resetGlobalTransformation(
        xyz=np.array([x, y, 0.94*walker.scaling]),
        rpy=np.array([0, 0, orientation-np.pi/2]),
        gait_phase_value=0
    )


class Ground:
    def __init__(self,
                 pybtPhysicsClient,
                 urdf_path=os.path.join(pybullet_data.getDataPath(), "plane.urdf")):
        self.id = p.loadURDF(
            urdf_path,
            physicsClientId=pybtPhysicsClient,
        )

    def advance(self, global_xyz, global_quaternion):
        p.resetBasePositionAndOrientation(
            self.id,
            global_xyz,
            global_quaternion
        )


class Simulator:
    DISTANCE = 2.0

    def __init__(
        self,
        Robot,
        Human,
        Controller,
        walker_scaling=1.0,
        show_GUI=True,
        timestep=0.01,
        collision_timestep=0.001,
        make_video=False,
        fast_forward=False
    ):
        self.timestep = timestep
        self.collision_timestep = collision_timestep
        self.t_max = 20.0 / walker_scaling

        # Objects
        self.Robot = Robot
        self.Human = Human
        self.Controller = Controller

        # define constants for the setup
        distance = self.DISTANCE
        robot_radius = 0.6
        human_radius = 0.6 * walker_scaling
        self.nominal_human_speed = 1.1124367713928223 * 0.95 * walker_scaling
        self.nominal_robot_speed = 1.0

        miss_angle_tmp = np.arccos(np.sqrt(1 - (robot_radius+human_radius)**2 / distance**2))
        self.miss_angle_lower_threshold = np.pi - miss_angle_tmp
        self.miss_angle_upper_threshold = np.pi + miss_angle_tmp
        self.miss_speed_threshold = (distance - human_radius - robot_radius) / self.t_max

        # set up Bullet with the robot and the walking man
        self.show_GUI = show_GUI
        self.__setup_world()

    def plot_collision_forces(self, collision_forces, robot_target_velocities):
        f, ax = plt.subplots(1, 2, sharex=True)

        time = np.arange(collision_forces.shape[0]) * self.collision_timestep
        ax[0].plot(time, np.linalg.norm(collision_forces[:, 0:2], axis=1))
        ax[0].set_xlabel("Time [s]")
        ax[0].set_ylabel("Force [N]")

        ax[1].plot(time, robot_target_velocities[:, 0], color=(0, 0.4470, 0.7410, 1))
        ax[1].set_xlabel("Time [s]")
        ax[1].set_ylabel("V [m/s]", color=(0, 0.4470, 0.7410, 1))
        ax[1].tick_params(axis='y', labelcolor=(0, 0.4470, 0.7410, 1))

        ax_ = ax[1].twinx()
        ax_.plot(time, robot_target_velocities[:, 1], color=(0.8500, 0.3250, 0.0980, 1))
        ax_.set_ylabel("Omega [rad/s]", color=(0.8500, 0.3250, 0.0980, 1))
        ax_.tick_params(axis='y', labelcolor=(0.8500, 0.3250, 0.0980, 1))

        plt.tight_layout()
        plt.show()

    def simulate(
        self,
        robot_angle=0,
        human_angle=0,
        gait_phase=0,
        human_speed_factor=1.0,
        robot_speed_factor=0.6
    ):
        human_speed = self.nominal_human_speed * human_speed_factor
        robot_speed = self.nominal_robot_speed * robot_speed_factor
        human_velocity = human_speed*np.array([np.cos(human_angle), np.sin(human_angle)])
        robot_velocity = robot_speed*np.array([np.cos(robot_angle), np.sin(robot_angle)])
        relative_velocity = human_velocity - robot_velocity
        relative_speed = np.sqrt(np.dot(relative_velocity, relative_velocity))
        angle_relative_v = pos_atan(relative_velocity[1], relative_velocity[0])

        if (
            self.miss_angle_lower_threshold < angle_relative_v < self.miss_angle_upper_threshold
            and relative_speed > self.miss_speed_threshold
        ):
            # Collision is possible
            self.collision_over = False
            self.robot.set_speed(robot_speed, 0)
            self.robot.reset()

            t = 0
            reset_walker_case_4(self.human, self.DISTANCE, robot_angle, human_angle, gait_phase)
            collision_forces = []
            robot_target_velocities = []
            sim_timestep = self.timestep
            self.robot.timestep = self.timestep
            self.human.timestep = self.timestep
            self.controller.timestep = self.timestep
            t_collision_over = None
            while t < self.t_max:
                sim_timestep = self.__step(collision_forces, robot_target_velocities)
                t += sim_timestep
                if self.collision_over:
                    if t_collision_over is None:
                        t_collision_over = t
                    if t - t_collision_over > 2:
                        # Exit simulation 2 sec after collision is over
                        break
                if self.show_GUI:
                    time.sleep(sim_timestep)

            collision_forces = np.array(collision_forces)
            robot_target_velocities = np.array(robot_target_velocities)

            if self.show_GUI:
                self.plot_collision_forces(collision_forces, robot_target_velocities)
            return collision_forces

    def __setup_world(self):
        if self.show_GUI:
            self.physics_client_id = p.connect(p.GUI)
        else:
            self.physics_client_id = p.connect(p.DIRECT)

        # Insert objects
        self.robot = self.Robot(self.physics_client_id, fixedBase=1, timestep=self.timestep)
        self.human = self.Human(self.physics_client_id, partitioned=True, timestep=self.timestep)
        self.ground = Ground(self.physics_client_id, os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

        if self.show_GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(1.7, -30, -5, [0, 0, 0.8], self.physics_client_id)

        # Attach Collision Detector and Controller
        self.collider = Collision(self.physics_client_id, robot=self.robot, human=self.human)
        self.controller = self.Controller(timestep=self.timestep)

    def __step(self, collision_forces, robot_target_velocities):
        self.robot.advance()
        xyz, quaternion = p.invertTransform(self.robot.global_xyz, self.robot.global_quaternion)
        self.human.advance(xyz, quaternion)
        self.ground.advance(xyz, quaternion)

        p.stepSimulation()

        F = self.collider.get_collision_force()
        if F is not None:
            # Collision Detected
            collision_forces.append(F)
            (v, omega) = self.controller.update(
                v=self.robot.v,
                omega=self.robot.omega,
                F=F
            )
            self.robot.set_speed(v, omega)
            robot_target_velocities.append([self.robot.v, self.robot.omega])
            self.human.fix()

            # Update timesteps
            self.robot.timestep = self.collision_timestep
            self.human.timestep = self.collision_timestep
            self.controller.timestep = self.collision_timestep
            return self.collision_timestep
        else:
            if len(collision_forces) > 0:
                # No collision after collision has occured
                self.robot.set_speed(0, 0)
                self.collision_over = True
            return self.timestep
