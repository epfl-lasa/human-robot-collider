#PYHTON 3!

import pybullet as p
import time
import math

physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadURDF("../data/human_adult.urdf", flags=p.URDF_MAINTAIN_LINK_ORDER)

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