import pybullet as p
from walk.walker import Man
#import time

import numpy as np
import scipy.io as sio

import argparse

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
	# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, orientation, excentricity, initial_gait_phase]
	result = np.zeros([0, 12])

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


if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description = 'Computes contact points between a human and a robot.')
	#parser.add_argument('robot', help='your name, enter it')

	parser.add_argument('robot',
		choices=['qolo','cuybot'],#,'pepper'],
		help='Robot to collide with')

	parser.add_argument('case',
		choices=['1','2','3'],
		help="""Experiment case.
			Case 1: robot moves forward into the standing human.
			Case 2: robot moves backward into the standing human.
			Case 3: human walks forward into the standing robot.""")

	args = parser.parse_args()

	if args.case == '3':
		if args.robot == 'qolo':
			urdf_path = '../data/qolo_and_user.urdf'
			result_name = 'qolo_contact_points_case_3'
			orientation_list = list(np.linspace(0, np.pi*2, 16, False))
			excentricity_list = list(np.linspace(-0.8, 0.8, 10, False))
			gait_phase_list = list(np.linspace(0, 1, 10, False))
		elif args.robot == 'cuybot':
			urdf_path = '../data/cuybot.urdf'
			result_name = 'cuybot_contact_points_case_3'
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

	# write the result to a npy-file and a mat-file
	np.save(result_name + '.npy', result)
	sio.savemat(result_name + '.mat', {'result': result})