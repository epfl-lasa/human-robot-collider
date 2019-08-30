import pybullet as p
from walk.walker import Man
import time

physicsClient = p.connect(p.GUI)

m1 = Man(physicsClient, partitioned = True)
m2 = Man(physicsClient)

for j in range(15):
	m1.resetGlobalTransformation([0,0,1],[0,0, -0.4*j])
	m2.resetGlobalTransformation([-3.0 + 0.5*j,-2,1],[0,0, 0])
	m2.setGaitPhase(0.4)
	for i in range(400):
		m1.advance()
		m2.advance()
		time.sleep(0.01)

p.disconnect()