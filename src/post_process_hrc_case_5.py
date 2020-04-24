import numpy as np
from matplotlib import pyplot as plt

def spring_constant_by_link_indices(human_link_idx, qolo_link_idx):
	human_link_spring_constant_map = [
		35000.0,35000.0,35000.0, # chest belly pelvis (front)
		50000.0,50000.0, # upper legs
		60000.0,60000.0, # shins
		75000.0,75000.0, # ankles/feet
		75000.0,75000.0, # upper arms
		75000.0,75000.0, # forearms
		75000.0,75000.0, # hands
		50000.0, # neck (front)
		75000.0, # head (front/face)
		75000.0,75000.0, # soles/feet
		75000.0,75000.0, # toes/feet
		35000.0,35000.0,35000.0, # chest belly pelvis (back)
		50000.0, # neck (back)
		150000.0 # head (back/skull)
		]
	return human_link_spring_constant_map[int(human_link_idx) + 1] #+1 for base

def reference_mass_by_link_indices(human_link_idx):
	human_link_mass_map = [
		40.0,40.0,40.0, # chest belly pelvis (front)
		15.0,15.0, # upper legs
		5.0,5.0, # shins
		1.0,1.0, # ankles/feet
		3.0,3.0, # upper arms
		2.0,2.0, # forearms
		0.6,0.6, # hands
		1.2, # neck (front)
		4.4, # head (front/face)
		1.0,1.0, # soles/feet
		1.0,1.0, # toes/feet
		40.0,40.0,40.0, # chest belly pelvis (back)
		1.2, # neck (back)
		4.4 # head (back/skull)
		]
	return human_link_mass_map[int(human_link_idx) + 1] #+1 for base

result = np.load("qolo_contact_points_case_5.npy")

# [iteration_number, link_1_index, link_2_index, point1-x,-y,-z, point2-x,-y,-z, impulse-x,-y,-z, 
# contact_normal_2_to_1-x,-y,-z, point1-local-x,-y,-z, point2-local-x,-y,-z,
# point1_pre_impact_velocity-x,-y,-z, point2_pre_impact_velocity-x,-y,-z,
# time_after_impact, robot_angle, human_angle, initial_gait_phase, robot_speed]

# find the "first impact" cluster for each iteration
time_horizon = 0.1
radial_bound = 0.05

processed_result_per_iteration = []
reference_model_force_result_per_iteration = []
skip_count = 0
default_spring_constant = 70000.0 # [N/m]

for iteration_number in range(int(result[-1,0]+2)):
	result_this_iteration = result[result[:,0]==float(iteration_number),:]
	n_impacts = np.size(result_this_iteration, 0)
	if n_impacts == 0:
		continue
	reference_contact_point = 0.5*(result_this_iteration[0,3:6] + result_this_iteration[0,6:9])
	normal_weighted_average = np.zeros([3])
	impulse_sum = np.zeros([3])
	unnormalized_weight_sum = 0.0
	spring_constant_weigthed_average = 0.0
	for i in range(n_impacts):
		contact_point = 0.5*(result_this_iteration[i,3:6] + result_this_iteration[i,6:9])
		if np.linalg.norm(contact_point - reference_contact_point) > radial_bound:
			continue
		#else:
			#reference_contact_point = contact_point
		time = result_this_iteration[i, 27]
		if time > time_horizon:
			break
		impulse = result_this_iteration[i, 9:12]/240.0 # it stores actually the force
		impulse_sum += impulse
		normal = result_this_iteration[i, 12:15]
		unnormalized_weight = np.linalg.norm(impulse)
		unnormalized_weight_sum += unnormalized_weight
		normal_weighted_average += unnormalized_weight*normal
		human_link_idx = result_this_iteration[i,1]
		qolo_link_idx = result_this_iteration[i,2]
		spring_constant = spring_constant_by_link_indices(human_link_idx, qolo_link_idx)
		spring_constant_weigthed_average += unnormalized_weight*spring_constant
	if unnormalized_weight_sum > 0.0:
		normal_weighted_average /= unnormalized_weight_sum
		spring_constant_weigthed_average /= unnormalized_weight_sum
	else:
		print ("Zero impulses, skipping ... N_impacts=", n_impacts)
		skip_count += 1
		continue
	normalization = np.linalg.norm(normal_weighted_average)
	if normalization > 0.000001:
		normal_weighted_average /= normalization
	else:
		print ("Normalization problem, skipping ... N_impacts=", n_impacts)
		skip_count += 1
		continue
	first_contact_relative_velocity = -(-result_this_iteration[0,21:24]
		+ result_this_iteration[0,24:27]) # the direction seems reversed to what is documented
	projected_impulse_sum = np.dot(normal_weighted_average, impulse_sum)
	projected_relative_velocity_first_contact = np.dot(normal_weighted_average, first_contact_relative_velocity)
	#reduced_mass = projected_impulse_sum/projected_relative_velocity_first_contact
	if (projected_relative_velocity_first_contact < 0.0):
		print ("velocity sign problem, skipping ... N_impacts=", n_impacts)
		skip_count += 1
		continue 
	if (projected_impulse_sum < 0.0):
		print ("impulse sum sign problem, skipping ... N_impacts=", n_impacts)
		skip_count += 1
		continue
	Fmax = np.sqrt(projected_relative_velocity_first_contact*projected_impulse_sum*spring_constant_weigthed_average)
	processed_result_per_iteration.append(Fmax)
	ref_mass = reference_mass_by_link_indices(result_this_iteration[0,1])
	ref_spring_constant = spring_constant_by_link_indices(result_this_iteration[0,1], 0)
	robot_mass = 36.0+68.0
	Fmax_ref_model = projected_relative_velocity_first_contact*np.sqrt(
		robot_mass*ref_mass/(robot_mass + ref_mass)*ref_spring_constant)
	reference_model_force_result_per_iteration.append(Fmax_ref_model)

#plt.hist(processed_result_per_iteration)
print ("Number of used composite impacts (iterations): ", len(processed_result_per_iteration))
print ("Number of skipped composite impacts (iterations): ", skip_count)
#plt.show()

# Creates two subplots and unpacks the output array immediately
f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ax1.hist(processed_result_per_iteration)
ax1.set_title('Force Distribution from Impulse Simulation')
ax1.set_xlabel('Force [N]')
ax1.set_ylabel('Occurrence count')
ax2.hist(reference_model_force_result_per_iteration)
ax2.set_title('Force Distribution from ISO Mass Simplification')
ax2.set_xlabel('Force [N]')
plt.show()



