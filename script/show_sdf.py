import pybullet as p
import time

physicsClient = p.connect(p.GUI)
human_adult_ID = p.loadSDF("../data/human_adult.sdf")
time.sleep(1.0)
#raw_input("Press Enter to continue...")

for i in range(2000):
	for j in range(p.getNumJoints(human_adult_ID[0])):
		p.resetJointStateMultiDof(human_adult_ID[0],
			j,
			targetValue=[i/10.0,i/8.0,i/10.0],
			targetVelocity=[0,0,0])
	p.resetBaseVelocity(human_adult_ID[0], [0.1, 0.1, 0.1])
	p.stepSimulation()
	time.sleep(1./240.)

p.disconnect()