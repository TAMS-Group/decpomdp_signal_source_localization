import numpy as np
from particle_filter import *
from policy import *
from tools import *
import matplotlib.pyplot as plt

def draw_graph(ax, locations, neighbours):
	ax.plot(locations[:,0], locations[:,1], 'ro', markersize=12,
		markeredgecolor='k')
	# Draw neighbour edges
	for v, v_neighs in neighbours.items():
		for n in v_neighs:
			ax.plot((locations[v,0], locations[n,0]), (locations[v,1], locations[n,1]), 'k')
	pass

def run_policy(x, weights, problem, num_steps, source_xy, policy0, policy1, visualize=False):
	average_distances = []
	for step in range(num_steps):
		# Take the actions according to policy
		l0 = policy0.next_action()
		l1 = policy1.next_action()

		z0, z1 = problem.sample_joint_observation(source_xy, l0, l1)
		l = problem.likelihood(x, l0, l1, z0, z1)

		if np.allclose(l, 0):
			print('Likelihood of observation 0 for all particles, this could be a problem; quitting now...')
			print('Agent 0 recorded RSS: {:f} dBm and agent 1 recorded RSS {:f} dBm'.format(np.asscalar(z0), np.asscalar(z1)))
			print('Agent 0 at location {:d} on graph and agent 1 at location {:d} on graph'.format(l0, l1))
			print('Source at (x,y) = ({:f}, {:f})'.format(source_xy[0], source_xy[1]))
			quit()
		print("weights before: {}".format(max(weights)))
		x, weights = SIR_step(x, weights, l)
		print("weights after step: {}".format(max(weights)))
		# update nodes in the policy
		policy0.next_node(z0)
		policy1.next_node(z1)

		# Calculate the weighted distance of particles to true source location
		distance_to_true = weightedL2(x, source_xy, weights)
		average_distances.append(distance_to_true)

		if visualize:
			print('*** Step {:d} ***'.format(step))
			print('Current average distance of particles to true source location: {:f} meters'.format(distance_to_true))
			print('Agent 0 recorded RSS: {:f} dBm and agent 1 recorded RSS {:f} dBm'.format(np.asscalar(z0), np.asscalar(z1)))
			print('Agent 0 at location {:d} on graph and agent 1 at location {:d} on graph'.format(l0, l1))
			print('Source at (x,y) = ({:f}, {:f})'.format(source_xy[0], source_xy[1]))

			fig, ax = plt.subplots()
			draw_graph(ax, problem.locations, problem.neighbours)
			ax.scatter(x[:,0], x[:,1], c=weights, s=36)
			ax.plot(source_xy[0], source_xy[1], 'bo', markersize=12, label='Source')
			ax.grid()

			# indiacte active locations
			color0, color1 = 'g', 'c'
			for agentidx, (l, c) in enumerate(zip((l0, l1), (color0, color1))):
				ax.plot(problem.locations[l,0], problem.locations[l,1], color=c, linestyle='none', marker='o', markersize=12,
					markeredgecolor='k', label='Agent {:d}'.format(agentidx))

			ax.scatter(x[:,0], x[:,1], c=weights)
			ax.set_title('Particle filter: Step {:d}'.format(step))
			ax.set_xlabel('X [meters]')
			ax.set_ylabel('Y [meters]')
			ax.legend(loc='center right')
			plt.show()


	return average_distances
