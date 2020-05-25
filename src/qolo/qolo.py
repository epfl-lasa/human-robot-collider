import os, math, time

import pybullet as p
import numpy as np


V_MAX = 1.5
OMEGA_MAX = 1.0


class Qolo:
	"""Class for QOLO Robot
	"""
	def __init__(self,
				 pybtPhysicsClient,
				 fixedBase=1,
				 self_collisions=False,
				 v=0,
				 omega=0,
				 timestep=0.01,
				 scaling=1.0):
		""" 
		"""
		self.body_id = p.loadURDF(
			os.path.join(os.path.dirname(__file__),"qolo.urdf"),
			flags=p.URDF_MAINTAIN_LINK_ORDER,
			physicsClientId=pybtPhysicsClient,
			globalScaling=scaling,
			useFixedBase=fixedBase,
			basePosition=[0, 0, 0.2],
			baseOrientation=p.getQuaternionFromEuler([0, 0, -math.pi/2])
		)
		self.scaling = scaling
		self.set_color()

		# Pose
		self.global_xyz = np.zeros(3)
		self.global_quaternion = np.zeros(4)
		self.axis_angle = 0
		self.wheel_phase = np.zeros(2)

		# Robot Speed
		self.set_speed(v, omega)

		# Time Step
		self.timestep = timestep

	def set_speed(self, v, omega):
		self.v = np.clip(v, -V_MAX, V_MAX)
		self.omega = np.clip(omega, -OMEGA_MAX, OMEGA_MAX)

		wheel_radius = self.scaling * 0.2
		half_width = self.scaling * 0.545/2
		self.wheel_speed = np.array([v+omega*half_width, v-omega*half_width]) / (2*math.pi*wheel_radius)

	def set_color(self):
		sdl = p.getVisualShapeData(self.body_id)
		colors = [
			[0.4, 0.4, 0.4, 1],	# Main Body
			[0.7, 0.7, 0.7, 1],	# Left Wheel
			[0.7, 0.7, 0.7, 1],	# Right Wheel
			[0.4, 0.4, 0.4, 1],	# Bumper
		]
		for i in range(len(sdl)):
			p.changeVisualShape(self.body_id, sdl[i][1], 
								rgbaColor=colors[i])

	def advance(self):
		self.global_xyz += self.timestep * np.array([self.v * np.sin(self.axis_angle),
													 -self.v * np.cos(self.axis_angle),
													 0])
		self.axis_angle += self.timestep * self.omega
		self.global_quaternion = p.getQuaternionFromAxisAngle((0,0,1), self.axis_angle)
		self.wheel_phase += self.timestep * self.wheel_speed
		# Left Wheel
		p.resetJointState(
			self.body_id,
			0,
			targetValue=self.wheel_phase[0]
		)
		# Right Wheel
		p.resetJointState(
			self.body_id,
			1,
			targetValue=self.wheel_phase[1]
		)

	def reset(self):
		self.global_xyz = np.zeros(3)
		self.global_rpy = np.zeros(3)
		self.wheel_phase = np.zeros(2)

