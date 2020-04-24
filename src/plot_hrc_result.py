import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from stl import mesh

def plot_hrc_result(result, robot_mesh_path):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	# plot the contact points
	X = (result[:,3]+result[:,6])/2
	Y = (result[:,4]+result[:,7])/2
	Z = (result[:,5]+result[:,8])/2
	C = color_code(result[:,1])
	#ax.scatter(X,Y,Z, marker = 'o', color = C, s = 60)
	#add_spheres(ax,X,Y,Z,C)

	# plot a sphere for each point
	sphere_mesh = mesh.Mesh.from_file('../data/sphere.stl')
	add_sphere_collections(ax, X, Y, Z, C, sphere_mesh)

	# plot the robot mesh
	robot_mesh = mesh.Mesh.from_file(robot_mesh_path)
	collection = mplot3d.art3d.Poly3DCollection(
		robot_mesh.vectors, alpha=1)
	face_color = [0.5, 0.5, 1] # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
	collection.set_facecolor(face_color)
	ax.add_collection3d(collection)
	#scale = robot_mesh.points.flatten(-1)
	#ax.auto_scale_xyz(scale, scale, scale)

	# Fix aspect ratio as follows
	ax.set_aspect('equal')

	# Create cubic bounding box to simulate equal aspect ratio
	max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
	Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(X.max()+X.min())
	Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(Y.max()+Y.min())
	Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(Z.max()+Z.min())
	# Comment or uncomment following both lines to test the fake bounding box:
	for xb, yb, zb in zip(Xb, Yb, Zb):
   		ax.plot([xb], [yb], [zb], 'w')

	plt.show()

def color_code(human_link_indices):
	red = np.array([1, 0, 0])
	blue = np.array([0, 1, 0])
	green = np.array([0, 0, 1])
	yellow = np.array([1, 1, 0])
	pink = np.array([1, 0, 1])
	cyan = np.array([0, 1, 1])
	black = np.array([0, 0, 0])
	grey = np.array([0.5, 0.5, 0.5])
	orange = np.array([255/255, 110/255, 10/255])
	purple = np.array([110/255, 0, 1])

	C = np.zeros([np.size(human_link_indices,0), 3])

	true_for_foot_indices = (
		(human_link_indices == 6) |
		(human_link_indices == 7) |
		(human_link_indices == 16) |
		(human_link_indices == 17) |
		(human_link_indices == 18) |
		(human_link_indices == 19))
	true_for_shin_indices = (
		(human_link_indices == 4) |
		(human_link_indices == 5))
	true_for_leg_indices = (
		(human_link_indices == 2) |
		(human_link_indices == 3))
	true_for_arm_indices = (
		(human_link_indices == 8) |
		(human_link_indices == 9))
	true_for_forearm_indices = (
		(human_link_indices == 10) |
		(human_link_indices == 11))
	true_for_hand_indices = (
		(human_link_indices == 12) |
		(human_link_indices == 13))
	true_for_head_indices = (human_link_indices == 15)
	true_for_neck_indices = (human_link_indices == 14)
	true_for_torso_indices = ((human_link_indices == -1) |
		(human_link_indices == 0))
	true_for_pelvis_indices = (human_link_indices == 1)

	C[true_for_foot_indices, :] = red
	C[true_for_shin_indices, :] = orange
	C[true_for_leg_indices, :] = yellow
	C[true_for_hand_indices, :] = blue
	C[true_for_forearm_indices, :] = cyan
	C[true_for_arm_indices, :] = grey
	C[true_for_pelvis_indices, :] = pink
	C[true_for_torso_indices, :] = green
	C[true_for_neck_indices, :] = black
	C[true_for_head_indices, :] = purple

	return C

### Link-index correspondence
# -1 Chest
# 0 Belly
# 1 Pelvis
# 2 right leg
# 3 left leg
# 4 right shin
# 5 left shin
# 6 right foot
# 7 left foot
# 8 right arm
# 9 left arm
# 10 right forearm
# 11 left forearm
# 12 right hand
# 13 left hand
# 14 neck
# 15 head
# 16 right sole
# 17 left sole
# 18 right toes
# 19 left toes

def add_spheres(ax, X, Y, Z, C):
	radius = 0.025

	for i in range(np.size(X,0)):
		# draw sphere
		# u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
		# x = radius*np.cos(u)*np.sin(v) + X[i]
		# y = radius*np.sin(u)*np.sin(v) + Y[i]
		# z = radius*np.cos(v) + Z[i]
		# ax.plot_wireframe(x, y, z, color=C[i,:])

		# Make data
		u = np.linspace(0, 2 * np.pi, 7)
		v = np.linspace(0, np.pi, 7)
		x = radius * np.outer(np.cos(u), np.sin(v))
		y = radius * np.outer(np.sin(u), np.sin(v))
		z = radius * np.outer(np.ones(np.size(u)), np.cos(v))

		# Plot the surface
		ax.plot_surface(x+X[i], y+Y[i], z+Z[i], color=C[i,:])

def add_sphere_collections(ax, X, Y, Z, C, sphere_mesh):
	radius = 0.025
	for i in range(np.size(X,0)):
		collection = mplot3d.art3d.Poly3DCollection(
			radius*sphere_mesh.vectors + np.array([[[X[i],Y[i],Z[i]]]]))
		collection.set_facecolor(C[i])
		ax.add_collection3d(collection)

if __name__ == '__main__':
	result = np.load("qolo_contact_points_case_3.npy")
	plot_hrc_result(result, "../data/Qolo_w_user_01.stl")
