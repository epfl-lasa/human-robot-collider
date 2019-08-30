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

physicsClient = p.connect(p.GUI)

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [5,5,5])
box_ID = p.createMultiBody(0, colBoxId, -1, [0, 0, -5], [0, 0, 0, 1])

p.setGravity(0.0,0.0,-9.81)

m = Man(physicsClient, self_collisions = True)
m.resetGlobalTransformation([0,0,0.94],[0,0,0])

time.sleep(7)
for i in range(100):
	m.advance()
	time.sleep(0.01)

disable_motors(m)
set_body_velocities(m)

for i in range(240*20):
	p.stepSimulation()
	time.sleep(1/240.0)

p.disconnect()