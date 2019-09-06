import pybullet as p
from walk.walker import Man
import time

import numpy as np

def quaternion_multiplication(q1_, q2_):
	q1 = [q1_[3],q1_[0],q1_[1],q1_[2]]
	q2 = [q2_[3],q2_[0],q2_[1],q2_[2]]
	q1_times_q2 = np.array([
		q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
		q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
		q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
		q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]])
	return np.roll(q1_times_q2, -1)

def get_joint_positions_as_list(walker):
	joint_position_list = []
	for i in range(p.getNumJoints(walker.body_id)):
		joint_info = p.getJointInfo(walker.body_id, i)
		if joint_info[2] == p.JOINT_SPHERICAL:
			joint_state = p.getJointStateMultiDof(walker.body_id, i)
		elif joint_info[2] == p.JOINT_REVOLUTE:
			joint_state = p.getJointState(walker.body_id, i)
		joint_position_list.append(joint_state[0])
	return joint_position_list

def quaternion_and_its_derivative_to_angular_velocity(quaternion, derivative):
	dummy, inverse_quaternion = p.invertTransform([0,0,0], quaternion)
	omega4vec = quaternion_multiplication(2*derivative, inverse_quaternion)
	return np.array([omega4vec[0], omega4vec[1], omega4vec[2]])

def get_body_velocities(walker):
	walker.advance()

	pos_2, ori_2 = p.getBasePositionAndOrientation(walker.body_id)
	joint_position_list_2 = get_joint_positions_as_list(walker)

	walker.regress()
	walker.regress()

	pos_1, ori_1 = p.getBasePositionAndOrientation(walker.body_id)
	joint_position_list_1 = get_joint_positions_as_list(walker)

	walker.advance()

	# compute base velocities via central differences assuming a timestep of 0.01 [s]
	baseLinearVelocity = (np.array(pos_2) - np.array(pos_1))/0.02
	quaternion_derivative = (np.array(ori_2) - np.array(ori_1))/0.02

	dummy, ori_now = p.getBasePositionAndOrientation(walker.body_id)
	dummy, inverse_ori_now = p.invertTransform([0,0,0], ori_now)
	omega4vec = quaternion_multiplication(2*quaternion_derivative, inverse_ori_now)
	baseAngularVelocity = [omega4vec[0], omega4vec[1], omega4vec[2]] 

	# compute joint velocities via central differences assuming a timestep of 0.01 [s]
	joint_velocities = []
	joint_position_list_now = get_joint_positions_as_list(walker)
	for i in range(len(joint_position_list_now)):
		joint_info = p.getJointInfo(walker.body_id, i)
		if joint_info[2] == p.JOINT_SPHERICAL:
			omega = quaternion_and_its_derivative_to_angular_velocity(joint_position_list_now[i],
				(np.array(joint_position_list_2[i]) - np.array(joint_position_list_1[i]))/0.02)
			joint_velocities.append(omega)
		elif joint_info[2] == p.JOINT_REVOLUTE:
			omega_scalar = (joint_position_list_2[i] - joint_position_list_1[i])/0.02
			joint_velocities.append(omega_scalar)

	base_velocities = [baseLinearVelocity, baseAngularVelocity]
	return (base_velocities, joint_velocities)

def set_body_velocities(walker):
	walker.advance()

	pos_2, ori_2 = p.getBasePositionAndOrientation(walker.body_id)
	joint_position_list_2 = get_joint_positions_as_list(walker)

	walker.regress()
	walker.regress()

	pos_1, ori_1 = p.getBasePositionAndOrientation(walker.body_id)
	joint_position_list_1 = get_joint_positions_as_list(walker)

	walker.advance()

	# compute base velocities via central differences assuming a timestep of 0.01 [s]
	baseLinearVelocity = (np.array(pos_2) - np.array(pos_1))/0.02
	quaternion_derivative = (np.array(ori_2) - np.array(ori_1))/0.02

	dummy, ori_now = p.getBasePositionAndOrientation(walker.body_id)
	dummy, inverse_ori_now = p.invertTransform([0,0,0], ori_now)
	omega4vec = quaternion_multiplication(2*quaternion_derivative, inverse_ori_now)
	baseAngularVelocity = [omega4vec[0], omega4vec[1], omega4vec[2]] 

	p.resetBaseVelocity(walker.body_id,
		linearVelocity = baseLinearVelocity, angularVelocity = baseAngularVelocity)

	# compute joint velocities via central differences assuming a timestep of 0.01 [s]
	joint_position_list_now = get_joint_positions_as_list(walker)
	for i in range(len(joint_position_list_now)):
		joint_info = p.getJointInfo(walker.body_id, i)
		if joint_info[2] == p.JOINT_SPHERICAL:
			omega = quaternion_and_its_derivative_to_angular_velocity(joint_position_list_now[i],
				(np.array(joint_position_list_2[i]) - np.array(joint_position_list_1[i]))/0.02)
			p.resetJointStateMultiDof(walker.body_id, i,
				targetValue = joint_position_list_now[i], targetVelocity = omega)
		elif joint_info[2] == p.JOINT_REVOLUTE:
			omega_scalar = (joint_position_list_2[i] - joint_position_list_1[i])/0.02
			p.resetJointState(walker.body_id, i,
				targetValue = joint_position_list_now[i], targetVelocity = omega_scalar)

def disable_motors(walker):
	for j in range (p.getNumJoints(walker.body_id)):
		ji = p.getJointInfo(walker.body_id,j)
		targetPosition=[0]
		jointType = ji[2]
		if (jointType   == p.JOINT_SPHERICAL):
			targetPosition=[0,0,0,1]
			p.setJointMotorControlMultiDof(walker.body_id,j,p.POSITION_CONTROL,targetPosition, targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0,0,0])
		if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
			p.setJointMotorControl2(walker.body_id,j,p.VELOCITY_CONTROL,targetVelocity=0, force=0)

def prepare_man_on_qolo(body_id):
	sdl = p.getVisualShapeData(body_id)
	for i in range(len(sdl)):
		if i < 26: 
			p.changeVisualShape(body_id, sdl[i][1], rgbaColor=[0, 0, 0.45, 1])
		else:
			p.changeVisualShape(body_id, sdl[i][1], rgbaColor=[0.45, 0.45, 0.45, 1])

	belly_x_rot = 0.09
	p.resetBasePositionAndOrientation(body_id,
		[0,0,1.345],p.getQuaternionFromEuler([np.pi/2+belly_x_rot,0,0]))

	p.resetJointStateMultiDof(body_id, 0, p.getQuaternionFromEuler([-belly_x_rot,0,0]))

	leg_z_rot = 0.04
	leg_x_rot = -0.23
	leg_y_rot = -0.35
	p.resetJointStateMultiDof(body_id, 2, p.getQuaternionFromEuler([leg_x_rot,leg_y_rot,-leg_z_rot]))
	p.resetJointStateMultiDof(body_id, 3, p.getQuaternionFromEuler([leg_x_rot,-leg_y_rot,leg_z_rot]))

	shin_rot = 0.4
	p.resetJointState(body_id, 4, shin_rot)
	p.resetJointState(body_id, 5, shin_rot)

	foot_y_rot = 0.45
	foot_x_rot = -0.2
	p.resetJointStateMultiDof(body_id, 6, p.getQuaternionFromEuler([foot_x_rot,foot_y_rot,0]))
	p.resetJointStateMultiDof(body_id, 7, p.getQuaternionFromEuler([foot_x_rot,-foot_y_rot,0]))

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


physicsClient = p.connect(p.GUI)
body_id = p.loadURDF("../data/man_on_qolo/man_x_partitioned_on_qolo_fixed.urdf",
	flags=p.URDF_MAINTAIN_LINK_ORDER)

prepare_man_on_qolo(body_id)
disable_body_motors(body_id)

#p.resetBasePositionAndOrientation(body_id,)

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [5,5,5])
ground_ID = p.createMultiBody(0, colBoxId, -1, [0, 0, -5], [0, 0, 0, 1])

#p.setGravity(0.0,0.0,-9.81)

m = Man(physicsClient, self_collisions = True)
m.resetGlobalTransformation([0,-3,0.94],[0,0,0])

bodies = [ground_ID, m.body_id, body_id]
for body in bodies:
	for i in range(-1, p.getNumJoints(body)):
		p.changeDynamics(body, i, restitution = 0.0,
			lateralFriction = 0.0,
			rollingFriction = 0.0,
			spinningFriction = 0.0
			)

#m_ref = Man(physicsClient, self_collisions = True)
#m_ref.resetGlobalTransformation([2,0,0.94],[0,0,0])

#p.resetBaseVelocity(body_id, [0,-1.0*2.4,0],[0,0,0])
p.resetBaseVelocity(body_id, [0,-1.0,0],[0,0,0])

time.sleep(3)

hit = False
while not hit:
	m.advance()
	p.stepSimulation()
	cps = p.getContactPoints(m.body_id, body_id)
	for cp in cps:
		if cp[8] <= 0.0:
			hit = True
	time.sleep(0.01)

time.sleep(5.01)
m.regress()
disable_motors(m)
set_body_velocities(m)

#p.resetBaseVelocity(body_id, [0,-1.0,0],[0,0,0])

#sum_of_absolute_contact_impulses = 0.0
#sum_of_absolute_contact_forces = 0.0
#maximum_absolute_contact_force = 0.0

timestep = 1/240.0
p.setTimeStep(timestep)
#p.setGravity(0,0,-9.81)

time_horizon = 2

hit_count = 0
step_count = 0
radial_bound = 10.1
#while hit_count < 1:
for i in range(int(time_horizon/timestep)):#(240*20):
	step_count += 1
	p.stepSimulation()
	cps = p.getContactPoints(m.body_id, body_id)
	for cp in cps:
		force_magnitude = np.sqrt(
			cp[9]*cp[9] + cp[10]*cp[10] + cp[12]*cp[12])
		if force_magnitude > 0.0:
			if hit_count == 0:
				first_contact_point = np.array(cp[6])
				sum_of_absolute_contact_forces = force_magnitude
				sum_of_absolute_contact_impulses = timestep*force_magnitude
				maximum_absolute_contact_force = force_magnitude
			else:
				if np.linalg.norm(np.array(cp[6])-first_contact_point) < radial_bound:
					sum_of_absolute_contact_forces += force_magnitude
					sum_of_absolute_contact_impulses += timestep*force_magnitude
					if force_magnitude > maximum_absolute_contact_force:
						maximum_absolute_contact_force = force_magnitude
			hit_count += 1

	time.sleep(timestep)#1/240.0)

time.sleep(3)

p.disconnect()

print ("Step count", step_count)

print ("sum_of_absolute_contact_impulses: ", sum_of_absolute_contact_impulses)
print ("sum_of_absolute_contact_forces: ", sum_of_absolute_contact_forces)

maximum_absolute_contact_impulse = timestep*maximum_absolute_contact_force

print ("maximum_absolute_contact_impulse: ", maximum_absolute_contact_impulse)
print ("maximum_absolute_contact_force: ", maximum_absolute_contact_force)