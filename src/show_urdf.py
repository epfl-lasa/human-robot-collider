#PYHTON 3!

import pybullet as p
import time
import math

def generateQuaternionFromMMMRxRyRz(rx, ry, rz):
	q_intermediate = p.getQuaternionFromEuler([math.pi/2, 0, math.pi/2])
	t, q_intermediate_inv = p.invertTransform([0,0,0], q_intermediate)
	q_tmp = p.getQuaternionFromEuler([ry, rz, rx])
	t, q_tmp = p.multiplyTransforms([0,0,0], q_intermediate, [0,0,0], q_tmp)
	t, q = p.multiplyTransforms([0,0,0], q_tmp, [0,0,0], q_intermediate_inv)
	return q

# def generateQuaternionFromMMMGlobalRxRyRz(rx, ry, rz):
# 	q_intermediate = p.getQuaternionFromEuler([0, -math.pi/2, math.pi])
# 	t, q_intermediate_inv = p.invertTransform([0,0,0], q_intermediate)
# 	q_tmp = p.getQuaternionFromEuler([rz, -ry, rx])
# 	t, q_tmp = p.multiplyTransforms([0,0,0], q_intermediate, [0,0,0], q_tmp)
# 	t, q = p.multiplyTransforms([0,0,0], q_tmp, [0,0,0], q_intermediate_inv)
# 	return q

def applyMMMRotationToURDFJoint(urdf_body_id, joint_index, rx, ry, rz, inverse=False):
	q = generateQuaternionFromMMMRxRyRz(rx, ry, rz)
	quat_tf_urdf = p.getQuaternionFromEuler([-math.pi/2, math.pi, 0])
	translation, quat_tf_urdf_inv = p.invertTransform([0,0,0], quat_tf_urdf)
	t, q = p.multiplyTransforms([0,0,0], quat_tf_urdf, [0,0,0], q)
	t, q = p.multiplyTransforms([0,0,0], q, [0,0,0], quat_tf_urdf_inv)

	if inverse:
		t, q = p.invertTransform([0,0,0], q)

	p.resetJointStateMultiDof(urdf_body_id, joint_index, q)

physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadURDF("../data/human_adult_shaped.urdf",
	flags=p.URDF_MAINTAIN_LINK_ORDER)

p.resetDebugVisualizerCamera(cameraDistance=3, 
	cameraYaw=-0, cameraPitch=-88, cameraTargetPosition=[-0,0,0])

p.changeVisualShape(human_adult_ID, -1,
	rgbaColor=[0, 1, 1, 0.5])

applyMMMRotationToURDFJoint(human_adult_ID,
	12, 0.1, -0.27, -0.2)
applyMMMRotationToURDFJoint(human_adult_ID,
	13, 0.1, 0.27, 0.2)
# right arm to right forearm
p.resetJointState(human_adult_ID, 14, -0.31)
# left arm to left forearm
p.resetJointState(human_adult_ID, 15, -0.4)


time.sleep(100)
p.disconnect()
quit()


#p.setGravity(0.0,0.0,-10.0)

sphereRadius = 0.5
colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [0.11,0.11,0.2])
mass = 0
#visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius)
baseOrientation = [0, 0, 0, 1]
basePosition = [0, 0, -3.0]

planeID = p.createMultiBody(mass, colSphereId, -1, basePosition,
                                baseOrientation)

#time.sleep(1.0)
#raw_input("Press Enter to continue...")

for i in range(20000):
	for j in range(3):
		p.resetJointStateMultiDof(human_adult_ID, j, p.getQuaternionFromEuler([0,0,0]))
	p.resetJointStateMultiDof(human_adult_ID, 3, p.getQuaternionFromEuler([i/100.0,i/150.0,i/350.0]))
	#for j in range(p.getNumJoints(human_adult_ID)):
	#	p.resetJointStateMultiDof(human_adult_ID,
	#		j,
	#		targetValue=[i/10.0,i/8.0,i/10.0],
	#		targetVelocity=[0.01,0.01,0.01])
	p.resetBasePositionAndOrientation(human_adult_ID, [0.1, 0.1, 0.1], p.getQuaternionFromEuler([0,0,0]))
	p.stepSimulation()
	time.sleep(1./240.)

#p.getNumJoints(humanoid)
#p.resetJointStateMultiDof(humanoid, j, targetValue=targetPosition, targetVelocity=targetVel)


#for i in range(2000):
#	pos, ori = p.getBasePositionAndOrientation(human_adult_ID)
#	p.applyExternalForce(human_adult_ID, -1,
#		[50*math.cos(i/100.0*math.pi), 50*math.sin(i/88.0*math.pi), 50*math.sin(i/130.0*math.pi)],
#		pos, p.WORLD_FRAME)
#	p.stepSimulation()
#	time.sleep(1./240.)

p.disconnect()