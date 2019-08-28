import pybullet as p
from walk.walker import Man
#import time

import numpy as np
#import scipy.io as sio

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
	result = case_walking_human_static_robot('../data/qolo_and_user.urdf',
		list(np.linspace(0, np.pi*2, 16, False)),
		list(np.linspace(-0.8, 0.8, 10, False)),
		list(np.linspace(0, 1, 10, False)))

	# write the result to a npy-file
	np.save('qolo_contact_points_case_3.npy', result)