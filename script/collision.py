import pybullet as p
import math
import time

# load the human-adult model
physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadURDF("../data/human_adult_scan.urdf",
	#useFixedBase = 1,
	flags=p.URDF_MAINTAIN_LINK_ORDER | p.URDF_USE_SELF_COLLISION)#| means and here

#disable motors
for j in range (p.getNumJoints(human_adult_ID)):
	ji = p.getJointInfo(human_adult_ID,j)
	targetPosition=[0]
	jointType = ji[2]
	if (jointType   == p.JOINT_SPHERICAL):
		targetPosition=[0,0,0,1]
		p.setJointMotorControlMultiDof(human_adult_ID,j,p.POSITION_CONTROL,targetPosition, targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0,0,0])
	if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
		p.setJointMotorControl2(human_adult_ID,j,p.VELOCITY_CONTROL,targetVelocity=0, force=0)

p.resetBasePositionAndOrientation(human_adult_ID, [0,0,1.3],
	p.getQuaternionFromEuler([math.pi/2,0,0]))

#for j in range (p.getNumJoints(human_adult_ID)):
#	info = p.getDynamicsInfo(human_adult_ID, j)
#	print (info[0]) #Just checking if masses are being used

# create another body to collide with
#colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius = 0.5)
#sphere_ID = p.createMultiBody(40.0, colSphereId, -1, [0, 0, -3.0], [0, 0, 0, 1])
colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [5,5,5])
box_ID = p.createMultiBody(0, colBoxId, -1, [0, 0, -5], [0, 0, 0, 1])

p.setGravity(0.0,0.0,-10.0)

time.sleep(5.0)

for i in range(40*240):
	p.stepSimulation()
	time.sleep(1./240.)

p.disconnect()