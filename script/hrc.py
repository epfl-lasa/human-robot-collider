import pybullet as p
from walk.walker import Man
import time

import numpy as np
import scipy.io as sio

import argparse

def rotate_by_quaternion(vec3, q):
	R = p.getMatrixFromQuaternion(q)
	result = [
		R[0]*vec3[0] + R[1]*vec3[1] + R[2]*vec3[2],
		R[3]*vec3[0] + R[4]*vec3[1] + R[5]*vec3[2],
		R[6]*vec3[0] + R[7]*vec3[1] + R[8]*vec3[2]]
	return result

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

def case_walking_human_static_robot(robot_urdf_path,
	orientation_list,
	excentricity_list,
	gait_phase_list):
	# set up Bullet with the robot and the walking man
	physics_client_id = p.connect(p.DIRECT)
	#physics_client_id = p.connect(p.GUI)
	robot_body_id = p.loadURDF(robot_urdf_path, useFixedBase = 1)
	walking_man = Man(physics_client_id)

	# initialize the container for the results of all the iterations
	# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, velocity-point1-x,-y,-z, orientation, excentricity, initial_gait_phase]
	result = np.zeros([0, 15])

	iteration_number = 0
	number_of_collision_free_iterations = 0

	# let them collide
	for orientation in orientation_list:
		for excentricity in excentricity_list:
			for gait_phase in gait_phase_list:
				reset_walker(walking_man, orientation, excentricity, gait_phase)

				t = 0
				collision_free = True
				while (t < 300) and collision_free:
					walking_man.advance()
					p.stepSimulation()
					contact_points = p.getContactPoints(walking_man.body_id,
						robot_body_id)
					for cp in contact_points:
						if cp[8] <= 0.0:
							collision_free = False
							# compute the velocity of the contact point on the human via finite differences
							# 1) Get the point in link-local coordinates
							link_state = getLinkOrBaseState(walking_man.body_id, cp[3])

							p1_local = rotate_by_inverse_of_quaternion(np.array(cp[5])-np.array(link_state[4]), 
								link_state[5])
							# 2) Keep the local coordinates constant while advancing and regressing
							#    and compute the resulting world coordinates
							walking_man.advance()

							link_state = getLinkOrBaseState(walking_man.body_id, cp[3])
							p1_global_future = np.array(link_state[4]) + np.array(rotate_by_quaternion(p1_local, link_state[5]))

							walking_man.regress()
							walking_man.regress()

							link_state = getLinkOrBaseState(walking_man.body_id, cp[3])
							p1_global_past = np.array(link_state[4]) + np.array(rotate_by_quaternion(p1_local, link_state[5]))
							
							walking_man.advance()

							# 3) compute the velocity via finite differences assuming a timestep of 0.01
							velocity = (p1_global_future - p1_global_past)/0.02

							result = np.append(result, np.array([[
								iteration_number,
								cp[3],
								cp[4],
								cp[5][0],
								cp[5][1],
								cp[5][2],
								cp[6][0],
								cp[6][1],
								cp[6][2],
								velocity[0],
								velocity[1],
								velocity[2],
								orientation,
								excentricity,
								gait_phase]]), 0)
					t += 1
					#time.sleep(0.01)

				if t == 300:
					number_of_collision_free_iterations += 1
				iteration_number += 1

	print ("Number of collision-free iterations: ", number_of_collision_free_iterations)
	return result

def reset_walker(walker, orientation, excentricity, gait_phase):
	radius = 1.5
	x = -radius*np.cos(orientation) - excentricity*np.sin(orientation)
	y = -radius*np.sin(orientation) + excentricity*np.cos(orientation)
	walker.resetGlobalTransformation([x,y,0.94],[0,0, orientation-np.pi/2])
	walker.setGaitPhase(gait_phase)


def pos_atan(y,x):
	a = np.arctan2(y,x)
	if a < 0.0:
		a += 2*np.pi
	return a

def reset_walker_case_4(walker, distance, robot_angle, human_angle, gait_phase):
	x = distance*np.cos(-np.pi/2-robot_angle)
	y = distance*np.sin(-np.pi/2-robot_angle)
	orientation = -np.pi/2-robot_angle + human_angle
	walker.resetGlobalTransformation([x,y,0.94],[0,0, orientation-np.pi/2])
	walker.setGaitPhase(gait_phase)

def advance_case_4(walker, robot_speed):
	walker.global_xyz += 0.01*np.array([0.0, robot_speed, 0.0])
	walker.advance()

def regress_case_4(walker, robot_speed):
	walker.global_xyz += 0.01*np.array([0.0, -robot_speed, 0.0])
	walker.regress()

def case_both_moving_forward(robot_urdf_path, robot_angle_list, human_angle_list, gait_phase_list):
	# define constants for the setup
	distance = 2.0
	robot_radius = 0.6
	human_radius = 0.6
	t_max = 8.0
	human_speed = 1.1124367713928223
	robot_speed = 1.0
	miss_angle_tmp = np.arccos(np.sqrt(1 - (robot_radius+human_radius)*(robot_radius+human_radius)/distance/distance))
	miss_angle_lower_threshold = np.pi - miss_angle_tmp
	miss_angle_upper_threshold = np.pi + miss_angle_tmp


	# set up Bullet with the robot and the walking man
	physics_client_id = p.connect(p.DIRECT)
	#physics_client_id = p.connect(p.GUI)
	robot_body_id = p.loadURDF(robot_urdf_path, useFixedBase = 1)
	walking_man = Man(physics_client_id)

	# initialize the container for the results of all the iterations
	# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, velocity-point1-x,-y,-z, robot_angle, human_angle, initial_gait_phase]
	result = np.zeros([0, 15])

	iteration_number = 0
	number_of_collision_free_iterations = 0

	# let them collide
	for robot_angle in robot_angle_list:
		for human_angle in human_angle_list:
			for gait_phase in gait_phase_list:
				human_velocity = human_speed*np.array([np.cos(human_angle), np.sin(human_angle)])
				robot_velocity = robot_speed*np.array([np.cos(robot_angle), np.sin(robot_angle)])
				relative_velocity = human_velocity - robot_velocity
				relative_speed = np.sqrt(np.dot(relative_velocity, relative_velocity))
				angle_relative_v = pos_atan(relative_velocity[1], relative_velocity[0])
				if (relative_speed > (distance -human_radius-robot_radius)/t_max) and (
					miss_angle_lower_threshold < angle_relative_v) and (
					miss_angle_upper_threshold > angle_relative_v):

					ti = 0
					collision_free = True
					time_out = False
					reset_walker_case_4(walking_man, distance, robot_angle, human_angle, gait_phase)
					while collision_free and not time_out:
						advance_case_4(walking_man, robot_speed)

						p.stepSimulation()
						contact_points = p.getContactPoints(walking_man.body_id,
							robot_body_id)

						for cp in contact_points:
							if cp[8] <= 0.0:
								collision_free = False
								# compute the velocity of the contact point on the human via finite differences
								# 1) Get the point in link-local coordinates
								link_state = getLinkOrBaseState(walking_man.body_id, cp[3])

								p1_local = rotate_by_inverse_of_quaternion(np.array(cp[5])-np.array(link_state[4]), 
									link_state[5])
								# 2) Keep the local coordinates constant while advancing and regressing
								#    and compute the resulting world coordinates
								advance_case_4(walking_man, robot_speed)

								link_state = getLinkOrBaseState(walking_man.body_id, cp[3])
								p1_global_future = np.array(link_state[4]) + np.array(rotate_by_quaternion(p1_local, link_state[5]))

								regress_case_4(walking_man, robot_speed)
								regress_case_4(walking_man, robot_speed)

								link_state = getLinkOrBaseState(walking_man.body_id, cp[3])
								p1_global_past = np.array(link_state[4]) + np.array(rotate_by_quaternion(p1_local, link_state[5]))
								
								advance_case_4(walking_man, robot_speed)

								# 3) compute the velocity via finite differences assuming a timestep of 0.01
								velocity = (p1_global_future - p1_global_past)/0.02

								result = np.append(result, np.array([[
									iteration_number,
									cp[3],
									cp[4],
									cp[5][0],
									cp[5][1],
									cp[5][2],
									cp[6][0],
									cp[6][1],
									cp[6][2],
									velocity[0],
									velocity[1],
									velocity[2],
									robot_angle,
									human_angle,
									gait_phase]]), 0)
						ti += 1
						if ti > int(t_max/0.01):
							time_out = True
						#time.sleep(0.01)
					if time_out and collision_free:
						number_of_collision_free_iterations += 1
				else:
					number_of_collision_free_iterations += 1
				iteration_number += 1

	print ("Number of collision-free iterations: ", number_of_collision_free_iterations)
	return result

if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description = 'Computes contact points between a human and a robot.')
	#parser.add_argument('robot', help='your name, enter it')

	parser.add_argument('robot',
		choices=['qolo','cuybot'],#,'pepper'],
		help='Robot to collide with')

	parser.add_argument('case',
		choices=['3','4'],
		help="""Experiment case.
			Case 3: human walks forward into the standing robot.
			Case 4: human walks forward and collides with robot moving forward.""")
		#"""Experiment case.
			#Case 1: robot moves forward into the standing human.
			#Case 2: robot moves backward into the standing human.
			#Case 3: human walks forward into the standing robot.""")

	args = parser.parse_args()

	if args.case == '3':
		if args.robot == 'qolo':
			urdf_path = '../data/qolo_and_user.urdf'
			result_name = 'qolo_contact_points_case_3_with_velocities'
			orientation_list = list(np.linspace(0, np.pi*2, 16, False))
			excentricity_list = list(np.linspace(-0.8, 0.8, 10, False))
			gait_phase_list = list(np.linspace(0, 1, 10, False))
		elif args.robot == 'cuybot':
			urdf_path = '../data/cuybot.urdf'
			result_name = 'cuybot_contact_points_case_3_with_velocities'
			orientation_list = list(np.linspace(0, np.pi*2, 16, False))
			excentricity_list = list(np.linspace(-0.6, 0.6, 11, True))
			gait_phase_list = list(np.linspace(0, 1, 10, False))

		#elif args.robot is 'pepper':
		#	urdf_path = '../data/pepper.urdf'
		#	result_name = 'pepper_contact_points_case_3'

		result = case_walking_human_static_robot(urdf_path,
			orientation_list,
			excentricity_list,
			gait_phase_list)
	elif args.case == '4':
		if args.robot == 'qolo':
			urdf_path = '../data/qolo_and_user_rotated.urdf'
			result_name = 'qolo_contact_points_case_4_with_velocities'
			robot_angle_list = list(np.linspace(0,np.pi*2,16,False))
			human_angle_list = list(np.linspace(0,np.pi*2,16,False))
			gait_phase_list = list(np.linspace(0, 1, 10, False))
		elif args.robot == 'cuybot':
			urdf_path = '../data/cuybot.urdf'
			result_name = 'cuybot_contact_points_case_4_with_velocities'
			robot_angle_list = [0]
			human_angle_list = [0]
			gait_phase_list = [0]
		result = case_both_moving_forward(urdf_path,
			robot_angle_list,
			human_angle_list,
			gait_phase_list)

	# write the result to a npy-file and a mat-file
	np.save(result_name + '.npy', result)
	sio.savemat(result_name + '.mat', {'result': result})