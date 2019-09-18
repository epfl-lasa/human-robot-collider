import pybullet as p
from walk.walker import Man
import time

import numpy as np
import scipy.io as sio

import argparse

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
	walker.resetGlobalTransformation([x,y,0.94*walker.scaling],[0,0, orientation-np.pi/2])
	walker.setGaitPhase(gait_phase)

def advance_case_4(walker, robot_speed):
	walker.global_xyz += 0.01*np.array([0.0, robot_speed, 0.0])
	walker.advance()

def regress_case_4(walker, robot_speed):
	walker.global_xyz += 0.01*np.array([0.0, -robot_speed, 0.0])
	walker.regress()

def case_both_moving_forward(robot_urdf_path, robot_angle_list, human_angle_list, gait_phase_list,
	human_speed_factor_list, robot_speed_factor_list, walker_scaling, shuffle = False):
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
	show_GUI = True
	fast_forward = False
	if not show_GUI:
		physics_client_id = p.connect(p.DIRECT)
	else:
		physics_client_id = p.connect(p.GUI)
	robot_body_id = p.loadURDF(robot_urdf_path, useFixedBase = 1)
	shape_data = p.getVisualShapeData(robot_body_id)
	p.changeVisualShape(robot_body_id, shape_data[0][1], 
					rgbaColor=[0.4,0.4,0.4, 1])
	walking_man = Man(physics_client_id, partitioned = True, scaling = walker_scaling)
	
	if show_GUI:
		walking_man.setColorForPartitionedCase4()
		p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
		p.resetDebugVisualizerCamera(1.7,-30,-5,[0,0,0.8])
		hl = SphericalHighlight(30, 0.003, 0.3, 2.5, physics_client_id)

	show_one_box = False
	if show_one_box:
		colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [50,50,50])
		box_id = p.createMultiBody(0, colBoxId, -1, [0, 0, -50], [0,0,0,1])
		shape_data = p.getVisualShapeData(box_id)
		p.changeVisualShape(box_id, shape_data[0][1], 
					rgbaColor=[0.7,0.7,0.7, 1])

	show_boxes = True
	if (show_boxes and show_GUI):
		colBoxIds = []
		thickness = 4.0
		shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents = [10,thickness/2,5])
		n_boxes = 100
		start_y = -100.0
		for i in range(n_boxes):
			box_id = p.createMultiBody(0, shape_id, -1,
				[0, start_y + i*thickness, -5], [0, 0, 0, 1])
			shape_data = p.getVisualShapeData(box_id)
			p.changeVisualShape(box_id, shape_data[0][1], 
					rgbaColor=[i % 2,i % 2,i % 2, 1])

			colBoxIds.append(box_id)

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
										last_collision_point = cp[5] 
										point_A_link_local = get_link_local_coordinates(walking_man.body_id, cp[3], cp[5])
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
											cp[7][0],
											cp[7][1],
											cp[7][2],
											robot_speed,
											robot_angle,
											human_angle,
											gait_phase,
											walker_scaling,
											point_A_link_local[0],
											point_A_link_local[1],
											point_A_link_local[2]]]), 0)
								ti += 1
								if ti > int(t_max/0.01):
									time_out = True

								if (show_boxes and show_GUI):
									for i in range(n_boxes):
										pos, ori = p.getBasePositionAndOrientation(colBoxIds[i])
										p.resetBasePositionAndOrientation(colBoxIds[i],
											np.array(pos) + np.array([0,robot_speed*0.01,0]), ori)
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

def prepare_fixed_man_on_qolo(body_id, angle):
	if p.getNumJoints(body_id) == 0:
		return False # for Cuybot

	belly_x_rot = 0.09

	trans, rot = p.multiplyTransforms([0,0,0],p.getQuaternionFromEuler([0,0,angle+np.pi/2]),
		[0,0,1.345],p.getQuaternionFromEuler([np.pi/2+belly_x_rot,0,0]))
	p.resetBasePositionAndOrientation(body_id, trans, rot)

	p.resetJointStateMultiDof(body_id, 0, p.getQuaternionFromEuler([-belly_x_rot,0,0]))

	neck_x_rot = 0.45
	p.resetJointStateMultiDof(body_id, 14, p.getQuaternionFromEuler([neck_x_rot,0,0]))
	head_x_rot = -0.45
	p.resetJointStateMultiDof(body_id, 15, p.getQuaternionFromEuler([head_x_rot,0,0]))

	arm_x_rot = 0.02
	arm_z_rot = 0.25
	arm_y_rot = 0.0
	p.resetJointStateMultiDof(body_id, 8, p.getQuaternionFromEuler([arm_x_rot,arm_y_rot,-arm_z_rot]))
	p.resetJointStateMultiDof(body_id, 9, p.getQuaternionFromEuler([arm_x_rot,-arm_y_rot,arm_z_rot]))

	forearm_rot = -0.35
	p.resetJointState(body_id, 10, forearm_rot)
	p.resetJointState(body_id, 11, forearm_rot)

	hand_x_rot = -0.09
	hand_z_rot = -0.09
	hand_y_rot = 0.0
	p.resetJointStateMultiDof(body_id, 12, p.getQuaternionFromEuler([hand_x_rot,hand_y_rot,-hand_z_rot]))
	p.resetJointStateMultiDof(body_id, 13, p.getQuaternionFromEuler([hand_x_rot,-hand_y_rot,hand_z_rot]))
	return True

def reset_walker_case_5(walker, distance, human_angle, gait_phase):
	walker.resetGlobalTransformation([distance,0,0.94],[0,0, human_angle-np.pi/2])
	walker.setGaitPhase(gait_phase)

def reset_robot_case_5(robot_body_id, robot_angle):
	is_qolo = prepare_fixed_man_on_qolo(robot_body_id,robot_angle)
	if not is_qolo:
		print ("Verify that cuybot is oriented towards +x by default. Aborting...")
		quit()
		p.resetBasePositionAndOrientation(robot_body_id, [0,0,0], [0,0,robot_angle])


def advance_robot_case_5(robot_body_id, robot_angle, robot_speed):
	pos, ori = p.getBasePositionAndOrientation(robot_body_id)
	v = robot_speed*np.array([np.cos(robot_angle),np.sin(robot_angle),0])
	p.resetBasePositionAndOrientation(robot_body_id, np.array(pos)+v*0.01, ori)

def regress_robot_case_5(robot_body_id, robot_angle, robot_speed):
	pos, ori = p.getBasePositionAndOrientation(robot_body_id)
	v = -robot_speed*np.array([np.cos(robot_angle),np.sin(robot_angle),0])
	p.resetBasePositionAndOrientation(robot_body_id, np.array(pos)+v*0.01, ori)

def set_robot_velocity_case_5(robot_body_id, robot_angle, robot_speed):
	p.resetBaseVelocity(robot_body_id,
		[robot_speed*np.cos(robot_angle), robot_speed*np.sin(robot_angle),0],
		[0,0,0])
	for j in range(p.getNumJoints(robot_body_id)):
		ji = p.getJointInfo(robot_body_id,j)
		jointType = ji[2]
		if (jointType   == p.JOINT_SPHERICAL):
			js = p.getJointStateMultiDof(robot_body_id, j)
			p.resetJointStateMultiDof(robot_body_id, j, js[0], [0,0,0])
		if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
			js = p.getJointState(robot_body_id, j)
			p.resetJointState(robot_body_id, j, js[0], 0.0)

def extract_impulse_in_global_coordinates(contact_point_struct):
	return (np.array(contact_point_struct[7])*contact_point_struct[9] +
		np.array(contact_point_struct[11])*contact_point_struct[10] +
		np.array(contact_point_struct[13])*contact_point_struct[12])

def get_all_link_velocities(body_id):
	all_velocities = []
	all_velocities.append(p.getBaseVelocity(body_id))
	for i in range(p.getNumJoints(body_id)):
		state_l = p.getLinkState(body_id, i, computeLinkVelocity=1)
		all_velocities.append([state_l[6], state_l[7]])
	return all_velocities

def get_link_local_coordinates(body_id, link_index, point_global):
	link_state = getLinkOrBaseState(body_id, link_index)
	point_local = rotate_by_inverse_of_quaternion(np.array(point_global)-np.array(link_state[4]), 
											link_state[5])
	return point_local

def get_pre_impact_velocity(body_id, all_link_velocities, link_index, contact_point):
	link_state = getLinkOrBaseState(body_id, link_index)
	link_position = np.array(link_state[4])
	link_origin_velocity = np.array(all_link_velocities[link_index+1][0]) # +1 due to base
	link_angular_velocity = np.array(all_link_velocities[link_index+1][1]) # +1 due to base
	contact_point_vel = link_origin_velocity + np.cross(link_angular_velocity,
		np.array(contact_point) - link_position)
	return contact_point_vel

def collide_case_5(result, iteration_number, robot_angle, human_angle, gait_phase,
	robot_speed, t_max_to_collision, distance, walking_man, robot_body_id, show_GUI):
	reset_walker_case_5(walking_man, distance, human_angle, gait_phase)
	reset_robot_case_5(robot_body_id, robot_angle)
	# detect if and when a collision happens
	ti = 0
	collision_free = True
	time_out = False
	while collision_free and not time_out:
		walking_man.advance()
		advance_robot_case_5(robot_body_id, robot_angle, robot_speed)
		p.stepSimulation()
		contact_points = p.getContactPoints(walking_man.body_id, robot_body_id)
		for cp in contact_points:
			if cp[8] <= 0.0:
				collision_free = False
		ti += 1
		if ti > int(t_max_to_collision/0.01):
			time_out = True
		if show_GUI:
			time.sleep(0.01)
	if time_out and collision_free:
		return (result, False)
	# simulate the first 2 seconds of the collision
	time_horizon = 2.0
	walking_man.regress() 
	regress_robot_case_5(robot_body_id, robot_angle, robot_speed) # back up
	walking_man.set_body_velocities_from_gait()
	set_robot_velocity_case_5(robot_body_id, robot_angle, robot_speed) # set velocities
	for i in range(int(time_horizon*240.0)):
		all_walker_link_velocities = get_all_link_velocities(walking_man.body_id)
		all_robot_link_velocities = get_all_link_velocities(robot_body_id)
		p.stepSimulation()
		contact_points = p.getContactPoints(walking_man.body_id, robot_body_id)
		for cp in contact_points:
			impulse = extract_impulse_in_global_coordinates(cp)
			point_A_link_local = get_link_local_coordinates(walking_man.body_id, cp[3], cp[5])
			point_B_link_local = get_link_local_coordinates(robot_body_id, cp[4], cp[6])
			pre_impact_velocity_point_A = get_pre_impact_velocity(walking_man.body_id,
				all_walker_link_velocities, cp[3], cp[5])
			pre_impact_velocity_point_B = get_pre_impact_velocity(robot_body_id,
				all_robot_link_velocities, cp[4], cp[6])
			result = np.append(result, np.array([[
				iteration_number, cp[3], cp[4], # both link's indices
				cp[5][0], cp[5][1], cp[5][2], cp[6][0], cp[6][1], cp[6][2], #contact points
				impulse[0], impulse[1], impulse[2],
				cp[7][0], cp[7][1], cp[7][2], # normal direction B to A
				point_A_link_local[0], point_A_link_local[1], point_A_link_local[2],
				point_B_link_local[0], point_B_link_local[1], point_B_link_local[2],# link-local coordinates of contact points A and B, respectively
				pre_impact_velocity_point_A[0], pre_impact_velocity_point_A[1], pre_impact_velocity_point_A[2],
				pre_impact_velocity_point_B[0], pre_impact_velocity_point_B[1], pre_impact_velocity_point_B[2],# pre-impact velocities of contact points A and B, respectively
				i/240.0, robot_angle, human_angle, gait_phase, robot_speed]]), 0)
			#if show_GUI:
			#	p.addUserDebugLine(cp[6], np.array(cp[6])+np.array(cp[7]))
			#	time.sleep(1.0)
		if show_GUI:
			time.sleep(1/240.0)
	return (result, True)

def disable_body_motors(body_id):
	for j in range (p.getNumJoints(body_id)):
		ji = p.getJointInfo(body_id,j)
		targetPosition=[0]
		jointType = ji[2]
		if (jointType   == p.JOINT_SPHERICAL):
			targetPosition=[0,0,0,1]
			p.setJointMotorControlMultiDof(body_id,j,p.POSITION_CONTROL,targetPosition, targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0,0,0])
		if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
			p.setJointMotorControl2(body_id,j,p.VELOCITY_CONTROL,targetVelocity=0, force=0)

def set_robot_colors(body_id):
	sdl = p.getVisualShapeData(body_id)
	for i in range(len(sdl)):
		if i < 26: 
			p.changeVisualShape(body_id, sdl[i][1], rgbaColor=[0, 0, 0.45, 1])
		else:
			p.changeVisualShape(body_id, sdl[i][1], rgbaColor=[0.45, 0.45, 0.45, 1])

def case_both_moving_forward_impulse(robot_urdf_path,
	robot_angle_list, human_angle_list, gait_phase_list, robot_speed_factor_list):

	t_max_to_collision = 8.0
	human_speed = 1.1124367713928223*0.95
	nominal_robot_speed = 1.0
	distance = 2.0
	robot_radius = 0.6
	human_radius = 0.6
	miss_angle_tmp = np.arccos(np.sqrt(1 - (robot_radius+human_radius)*(robot_radius+human_radius)/distance/distance))
	miss_angle_lower_threshold = np.pi - miss_angle_tmp
	miss_angle_upper_threshold = np.pi + miss_angle_tmp

	# set up Bullet with the robot and the walking man
	show_GUI = False
	if not show_GUI:
		physics_client_id = p.connect(p.DIRECT)
	else:
		physics_client_id = p.connect(p.GUI)
	robot_body_id = p.loadURDF(robot_urdf_path, flags=p.URDF_MAINTAIN_LINK_ORDER)
	walking_man = Man(physics_client_id, partitioned = True)
	ground_box = p.createCollisionShape(p.GEOM_BOX, halfExtents = [100,100,5])
	ground_ID = p.createMultiBody(0, ground_box, -1, [0, 0, -5], [0, 0, 0, 1])
	if show_GUI:
		walking_man.setColorForPartitionedCase4()
		set_robot_colors(robot_body_id)
		p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

	disable_body_motors(walking_man.body_id)
	disable_body_motors(robot_body_id)

	bodies = [ground_ID, walking_man.body_id, robot_body_id]
	for body in bodies:
		for i in range(-1, p.getNumJoints(body)):
			p.changeDynamics(body, i, restitution = 0.0,
				lateralFriction = 0.0,
				rollingFriction = 0.0,
				spinningFriction = 0.0
				)
	#p.setGravity(0,0,-9.81)

	# initialize the container for the results of all the iterations
	# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, impulse-x,-y,-z, 
	# contact_normal_2_to_1-x,-y,-z, point1-local-x,-y,-z, point2-local-x,-y,-z,
	# point1_pre_impact_velocity-x,-y,-z, point2_pre_impact_velocity-x,-y,-z,
	# time_after_impact, robot_angle, human_angle, initial_gait_phase, robot_speed]
	result = np.zeros([0, 32])

	iteration_number = 0
	number_of_collision_free_iterations = 0

	# let them collide
	for robot_angle in robot_angle_list:
		for human_angle in human_angle_list:
			for gait_phase in gait_phase_list:
				for robot_speed_factor in robot_speed_factor_list:
					robot_speed = nominal_robot_speed*robot_speed_factor
					human_velocity = human_speed*np.array([np.cos(human_angle), np.sin(human_angle)])
					robot_velocity = robot_speed*np.array([np.cos(robot_angle), np.sin(robot_angle)])
					relative_velocity = human_velocity - robot_velocity
					relative_speed = np.sqrt(np.dot(relative_velocity, relative_velocity))
					angle_relative_v = pos_atan(relative_velocity[1], relative_velocity[0])
					if (relative_speed > (distance -human_radius-robot_radius)/t_max_to_collision) and (
						miss_angle_lower_threshold < angle_relative_v) and (
						miss_angle_upper_threshold > angle_relative_v):
						result, has_collision = collide_case_5(result, iteration_number,
							robot_angle, human_angle, gait_phase, robot_speed,
							t_max_to_collision, distance, walking_man, robot_body_id, show_GUI)
					else:
						has_collision = False
					if not has_collision:
						number_of_collision_free_iterations += 1
					iteration_number += 1
					print(np.size(result,0))

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
		choices=['3','4','5'],
		help="""Experiment case.
			Case 3: human walks forward into the standing robot.
			Case 4: human walks forward and collides with robot moving forward.
			Case 5: dynamic simulation of human and robot moving forward and colliding.""")
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
			result_name = 'qolo_contact_points_case_4_with_velocities_adult'
			robot_angle_list = list(np.linspace(0,np.pi*2,16,False))
			human_angle_list = list(np.linspace(0,np.pi*2,16,False))
			gait_phase_list = list(np.linspace(0, 1, 4, False))
			human_speed_factor_list = [1.0]#list(np.linspace(0.6, 1.4, 3, True))
			robot_speed_factor_list = list(np.linspace(0.6, 1.4, 3, True))
		elif args.robot == 'cuybot':
			urdf_path = '../data/cuybot.urdf'
			result_name = 'cuybot_contact_points_case_4_with_velocities'
			robot_angle_list = [0]
			human_angle_list = [0]
			gait_phase_list = [0]
			human_speed_factor_list = [1]
			robot_speed_factor_list = [1]
		walker_scaling = 1.0
		result = case_both_moving_forward(urdf_path,
			robot_angle_list,
			human_angle_list,
			gait_phase_list,
			human_speed_factor_list,
			robot_speed_factor_list,
			walker_scaling,
			shuffle = True)
		quit()
	elif args.case == '5':
		if args.robot == 'qolo':
			urdf_path = '../data/man_on_qolo/man_x_partitioned_on_qolo_fixed.urdf'
			result_name = 'qolo_contact_points_case_5'
			robot_angle_list = list(np.linspace(0,np.pi*2,16,False))
			human_angle_list = list(np.linspace(0,np.pi*2,16,False))
			gait_phase_list = list(np.linspace(0, 1, 4, False))
			robot_speed_factor_list = list(np.linspace(0.7, 1.3, 4, True))
		else:
			print ("Case 5 for Cuybot is not implemented yet.")
			quit()
		result = case_both_moving_forward_impulse(urdf_path, robot_angle_list,
			human_angle_list, gait_phase_list, robot_speed_factor_list)
	# write the result to a npy-file and a mat-file
	np.save(result_name + '.npy', result)
	sio.savemat(result_name + '.mat', {'result': result})