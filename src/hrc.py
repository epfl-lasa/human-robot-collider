import argparse, time, os

import pybullet as p
import pybullet_data
import numpy as np
import scipy.io as sio

from walker import *
from qolo import *
from collision import *

class SphericalHighlight():
	def __init__(self, n_spheres, r_min, r_max, duration, physics_client_id):
		self.duration = duration
		colSphereId = p.createCollisionShape(p.GEOM_SPHERE,
			radius = 0.001, physicsClientId=physics_client_id)
		self.list_of_spheres = []
		for i in range(n_spheres):
			r = r_min + (r_max - r_min)*i/float(n_spheres-1)
			visSphereId = p.createVisualShape(p.GEOM_SPHERE, radius = r,
				physicsClientId=physics_client_id)
			sphere_id = p.createMultiBody(0, colSphereId, visSphereId, [0,0,100],
				physicsClientId=physics_client_id)
			sdl = p.getVisualShapeData(sphere_id, physicsClientId=physics_client_id)
			p.changeVisualShape(sphere_id, sdl[0][1], rgbaColor=[1, 1, 0.8, 0.5],
				physicsClientId=physics_client_id)
			self.list_of_spheres.append(sphere_id)
	
	def highlight(self, target_position):
		for i in range(2):
			actual_target_position = target_position*(1-i) + i*np.array([0,0,100])
			for j in range(len(self.list_of_spheres)):
				sphere_index = (len(self.list_of_spheres)-1-j)*i + j*(1-i)
				p.resetBasePositionAndOrientation(self.list_of_spheres[sphere_index],
					actual_target_position,[0,0,0,1])
				time.sleep(float(self.duration)/len(self.list_of_spheres)/2.0)


def rotate_by_inverse_of_quaternion(vec3, q):
	R = p.getMatrixFromQuaternion(q)
	result = [
		R[0]*vec3[0] + R[3]*vec3[1] + R[6]*vec3[2],
		R[1]*vec3[0] + R[4]*vec3[1] + R[7]*vec3[2],
		R[2]*vec3[0] + R[5]*vec3[1] + R[8]*vec3[2]]
	return result


def getLinkOrBaseState(body_id, link_index):
	if link_index > -1:
		link_state = p.getLinkState(body_id, link_index,
			computeForwardKinematics = 1)
	else:
		base_position, base_orientation = p.getBasePositionAndOrientation(body_id)
		link_state = [None, None, None, None, base_position, base_orientation]
	return link_state


def get_link_local_coordinates(body_id, link_index, point_global):
	link_state = getLinkOrBaseState(body_id, link_index)
	point_local = rotate_by_inverse_of_quaternion(np.array(point_global)-np.array(link_state[4]), 
											link_state[5])
	return point_local


def pos_atan(y,x):
	a = np.arctan2(y,x)
	if a < 0.0:
		a += 2*np.pi
	return a


class Ground:
	def __init__(self,
				 pybtPhysicsClient,
				 urdf_path=os.path.join(pybullet_data.getDataPath(), "plane.urdf")):
		self.id = p.loadURDF(
			urdf_path,
            physicsClientId=pybtPhysicsClient,
		)

	def advance(self, global_xyz, global_rpy):
		p.resetBasePositionAndOrientation(
			self.id,
			global_xyz,
			p.getQuaternionFromEuler(global_rpy)
		)


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
		robot_urdf_path,
		robot_angle_list,
		human_angle_list,
		gait_phase_list,
		human_speed_factor_list,
		robot_speed_factor_list,
		walker_scaling,
		Human=Man,
		shuffle=False,
		with_GUI=True,
		timestep=0.01,
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


	# set up Bullet with the robot and the walking man
	#make_video = True
	show_GUI = with_GUI
	fast_forward = False
	if not show_GUI:
		physics_client_id = p.connect(p.DIRECT)
	else:
		physics_client_id = p.connect(p.GUI)
	# robot_body_id = p.loadURDF(robot_urdf_path, useFixedBase = 1)
	# shape_data = p.getVisualShapeData(robot_body_id)
	# p.changeVisualShape(robot_body_id, shape_data[0][1], 
	# 				rgbaColor=[0.4,0.4,0.4, 1])
	robot = Qolo(physics_client_id, fixedBase=1, timestep=timestep)
	human = Human(physics_client_id, partitioned=True, timestep=timestep)
	collider = Collision(physics_client_id, robot_body_id=robot.body_id, human_body_id=human.body_id)
	
	if show_GUI:
		human.setColor()
		p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
		p.resetDebugVisualizerCamera(1.7, -30, -5, [0, 0, 0.8], physics_client_id)
		hl = SphericalHighlight(30, 0.003, 0.3, 2.5, physics_client_id)
		ground = Ground(physics_client_id, os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
		#if make_video:
		#	p.startStateLogging( p.STATE_LOGGING_VIDEO_MP4, 'video_hrc.mp4' )

	# initialize the container for the results of all the iterations
	# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, velocity-point1-x,-y,-z, contact_normal_2_to_1-x,-y,-z, robot_speed, robot_angle, human_angle, initial_gait_phase, walker_scaling, point_A_link_local-x,-y,-z]
	result = np.zeros([0, 23])

	iteration_number = 0
	number_of_collision_free_iterations = 0

	if shuffle:
		for para_list in [robot_angle_list, human_angle_list, gait_phase_list, human_speed_factor_list, robot_speed_factor_list]:
			para_list[:] = list(np.random.permutation(para_list))
	# let them collide
	for robot_angle in robot_angle_list:
		for human_angle in human_angle_list:
			for gait_phase in gait_phase_list:
				for human_speed_factor in human_speed_factor_list:
					for robot_speed_factor in robot_speed_factor_list:
						human_speed = nominal_human_speed*human_speed_factor
						robot_speed = nominal_robot_speed*robot_speed_factor
						human_velocity = human_speed*np.array([np.cos(human_angle), np.sin(human_angle)])
						robot_velocity = robot_speed*np.array([np.cos(robot_angle), np.sin(robot_angle)])
						relative_velocity = human_velocity - robot_velocity
						relative_speed = np.sqrt(np.dot(relative_velocity, relative_velocity))
						angle_relative_v = pos_atan(relative_velocity[1], relative_velocity[0])

						robot.set_speed(robot_speed, 0)
						robot.reset()
						if (relative_speed > (distance -human_radius-robot_radius)/t_max) and (
							miss_angle_lower_threshold < angle_relative_v) and (
							miss_angle_upper_threshold > angle_relative_v):

							ti = 0
							collision_free = True
							collision_free_ = True
							time_out = False
							reset_walker_case_4(human, distance, robot_angle, human_angle, gait_phase)
							ground_loc = np.zeros(2)
							while collision_free and not time_out:
								robot.advance()
								if collision_free_:
									human.advance(-robot.global_xyz, -robot.global_rpy)
								else:
									human.fix()
								ground.advance(-robot.global_xyz, -robot.global_rpy)

								p.stepSimulation()
								
								(F, h, theta) = collider.get_collision_stat()
								if F is not None:
									# last_collision_point = theta
									# collision_free = False
									print((F, h, theta))
									# robot.set_speed(0, 0)
									collision_free_= False
	
								ti += 1
								if ti > int(t_max/0.01):
									time_out = True

								if show_GUI and not fast_forward:
									time.sleep(0.01)
							if time_out and collision_free:
								number_of_collision_free_iterations += 1
							elif show_GUI:
								time.sleep(0.5)
								hl.highlight(np.array(last_collision_point))
								time.sleep(0.5)
			
						else:
							number_of_collision_free_iterations += 1
						iteration_number += 1

	print ("Number of collision-free iterations: ", number_of_collision_free_iterations)
	return result


if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description = 'Computes contact points between a walking human and the standing mobility device Qolo driving forward.')

	parser.add_argument(
		'--human',
		choices=['adult','child'],
		default='adult',
		help='Human to collide with Qolo.'
	)

	parser.add_argument(
		'--with_gui',
		choices=['gui','no_gui'],
		default='gui',
		help="""The option allows to display the collisions in a GUI or 
			  to quickly simulate all the collisions without showing them."""
	)

	args = parser.parse_args()

	if args.human == 'child':
		human = Child
	else:
		human = Man

	walker_scaling = 1.0

	if args.with_gui == 'gui':
		option_gui = True
	else:
		option_gui = False

	urdf_path = 'data/qolo_with_user.urdf'
	result_name = 'result_phase_1'
	robot_angle_list = list(np.linspace(0,np.pi*2,16,False))
	human_angle_list = list(np.linspace(0,np.pi*2,16,False))
	gait_phase_list = list(np.linspace(0, 1, 4, False))
	human_speed_factor_list = [1.0]
	robot_speed_factor_list = list(np.linspace(0.6, 1.4, 3, True))

	result = case_both_moving_forward(urdf_path,
		robot_angle_list,
		human_angle_list,
		gait_phase_list,
		human_speed_factor_list,
		robot_speed_factor_list,
		walker_scaling,
		Human = human,
		shuffle = False,
		with_GUI = option_gui)

	# write the result to a npy-file and a mat-file
	np.save(result_name + '.npy', result)
	sio.savemat(result_name + '.mat', {'result': result})