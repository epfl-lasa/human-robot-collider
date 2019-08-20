import pybullet as p
from walk.walker import Man

physicsClient = p.connect(p.GUI)

m = Man(physicsClient)
m.resetGlobalTransformation([0,20,0.875],[0,0, 0])

p.resetDebugVisualizerCamera(cameraDistance=5, 
	cameraYaw=90, cameraPitch=0, cameraTargetPosition=[0,22,0.875])

#p.changeVisualShape(m.body_id, -1,
#	rgbaColor=[0, 1, 1, 0.5])
#shape_data_list = p.getVisualShapeData(m.body_id)
#for sd in shape_data_list:
#	p.changeVisualShape(m.body_id, sd[1], rgbaColor=[0, 1, 1, 1])

for i in range(20):
	text = input("Press enter to continue")
	for j in range(15):
		m.advance()

p.disconnect()