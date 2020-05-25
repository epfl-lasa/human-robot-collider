import argparse, time, os
import logging

import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import scipy.io as sio

from walker import *
from qolo import *
from collision import *
from controller import *


human_class = {
	"man": Man,
	"child": Child,
}
robot_class = {
	"qolo": Qolo,
}
controller_class = {
	"no_control": NoControl,
	"admittance": AdmittanceController,
}


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


def pos_atan(y,x):
	a = np.arctan2(y,x)
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


def case_both_moving_forward(
		Robot=Qolo,
		Human=Man,
		Controller=NoControl,
		robot_angle=0,
		human_angle=0,
		gait_phase=0,
		human_speed_factor=1.0,
		robot_speed_factor=0.6,
		walker_scaling=1.0,
		show_GUI=True,
		timestep=0.01,
		make_video=False,
		fast_forward=False,
	):
	# define constants for the setup
	distance = 2.0
	robot_radius = 0.6
	human_radius = 0.6*walker_scaling
	nominal_human_speed = 1.1124367713928223*0.95*walker_scaling
	nominal_robot_speed = 1.0
	t_max = 20.0/walker_scaling
	miss_angle_tmp = np.arccos(np.sqrt(1 - (robot_radius+human_radius)*(robot_radius+human_radius)/distance/distance))
	miss_angle_lower_threshold = np.pi - miss_angle_tmp
	miss_angle_upper_threshold = np.pi + miss_angle_tmp

	human_speed = nominal_human_speed*human_speed_factor
	robot_speed = nominal_robot_speed*robot_speed_factor
	human_velocity = human_speed*np.array([np.cos(human_angle), np.sin(human_angle)])
	robot_velocity = robot_speed*np.array([np.cos(robot_angle), np.sin(robot_angle)])
	relative_velocity = human_velocity - robot_velocity
	relative_speed = np.sqrt(np.dot(relative_velocity, relative_velocity))
	angle_relative_v = pos_atan(relative_velocity[1], relative_velocity[0])

	if (relative_speed > (distance -human_radius-robot_radius)/t_max) and (
		miss_angle_lower_threshold < angle_relative_v) and (
		miss_angle_upper_threshold > angle_relative_v):
		pass
	else:
		print("Skipping as no collision is possible")
		return

	# set up Bullet with the robot and the walking man
	if not show_GUI:
		physics_client_id = p.connect(p.DIRECT)
	else:
		physics_client_id = p.connect(p.GUI)
	
	p.setPhysicsEngineParameter(fixedTimeStep=timestep)
	robot = Robot(physics_client_id, fixedBase=1, timestep=timestep)
	human = Human(physics_client_id, partitioned=True, timestep=timestep)
	ground = Ground(physics_client_id, os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

	
	if show_GUI:
		p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
		p.resetDebugVisualizerCamera(1.7, -30, -5, [0, 0, 0.8], physics_client_id)

	# Attach Collision Detector and Controller
	collider = Collision(physics_client_id, robot=robot, human=human)
	controller = Controller(timestep=timestep)

	robot.set_speed(robot_speed, 0)
	robot.reset()

	t = 0
	reset_walker_case_4(human, distance, robot_angle, human_angle, gait_phase)
	collision_forces = []
	while t < t_max:
		robot.advance()
		xyz, quaternion = p.invertTransform(robot.global_xyz, robot.global_quaternion)								
		human.advance(xyz, quaternion)
		ground.advance(xyz, quaternion)

		p.stepSimulation()
		
		F = collider.get_collision_force()
		if F is not None:
			# Collision Detected
			collision_forces.append(F)
			(v, omega) = controller.update(
				v=robot.v,
				omega=robot.omega,
				F=F
			)
			robot.set_speed(v, omega)
			human.fix()
		else:
			if len(collision_forces) > 0:
				break
			robot.set_speed(robot_speed, 0)

		t += timestep

		if show_GUI and not fast_forward:
			time.sleep(timestep)

	p.disconnect()
	collision_forces = np.array(collision_forces)
	
	if show_GUI:
		plt.plot(np.linalg.norm(collision_forces[:,0:2],axis=1))
		plt.show()

	return collision_forces


if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		prog="HRC",
		description = """Simulation of robot and human collision"""
	)
	parser.add_argument("-b", "--human",
						choices=["man", "child"],
						default="man",
						help="Human to collide with the robot (default = man)")
	parser.add_argument("-r", "--robot",
						choices=["qolo"],
						default="qolo",
						help="Robot to collide with the human (default = qolo)")
	parser.add_argument("-c", "--controller",
						choices=["no_control","admittance"],
						default="no_control",
						help="Adaptive controller to use (default = no_control)")
	parser.add_argument("-g", "--gui",
						action="store_true",
						help="Set to show GUI")

	args = parser.parse_args()
	
	result = [
		case_both_moving_forward(
			Robot=robot_class[args.robot],
			Human=human_class[args.human],
			Controller=controller_class[args.controller],
			show_GUI=args.gui,
			robot_speed_factor=robot_speed_factor,
			human_speed_factor=human_speed_factor,
		) 
		for robot_angle in np.linspace(0, np.pi*2, 16, False)
		for human_angle in np.linspace(0, np.pi*2, 16, False)
		for robot_speed_factor in np.linspace(0.6, 1.4, 3, True)
		for human_speed_factor in [1.0]
		for gait_phase in np.linspace(0, 1, 4, False)
	]

	np.save("controlled_collision.npy", result)