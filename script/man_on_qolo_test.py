import pybullet as p
from walk.walker import Man
import time
import numpy as np

#setColorForPartitionedCase4(body_id)
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
body_id = p.loadURDF("../data/man_on_qolo/man_x_partitioned_on_qolo.urdf",
	flags=p.URDF_MAINTAIN_LINK_ORDER)

prepare_man_on_qolo(body_id)
disable_body_motors(body_id)

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [5,5,5])
box_ID = p.createMultiBody(0, colBoxId, -1, [0, 0, -5], [0, 0, 0, 1])

#time.sleep(1)

for i in range(5000):
	p.stepSimulation()
	time.sleep(1/240.0)

p.disconnect()