import numpy as np
import os
from rss_rician import SourceSeekingRSS
from execution import run_policy
from policy import RandomPolicy

# https://stackoverflow.com/a/43357954/5471520
def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def sample_source_locations(num_experiments):
	source_locations = -20.0 + 40.0 * np.random.uniform(size=(2,num_experiments))
	return source_locations

def run_source_localization(source_location, num_particles, num_steps, visualize):
	# Define graph for agent motion
	locations = -20.0 + 40.0 * np.array([(0.1, 0.9),
				 (0.9, 0.9),
				 (0.9, 0.1),
				 (0.1, 0.1),
				 (0.3, 0.5),
				 (0.5, 0.7),
				 (0.7, 0.5),
				 (0.5, 0.3),
				 (0.5, 0.5)])
	neighbours = {0: [0,1,3,4,5], 1: [0,1,2,5,6], 2: [1,2,3,6,7], 3: [0,2,3,4,7], 4: [0,3,4,5,7,8], 5: [0,1,4,5,6,8], 
					6: [1,2,5,6,7,8], 7: [2,3,4,7,6,8], 8: [4,5,6,7,8]}

	start_loc0, start_loc1 = 0, 0

	# The problem
	problem = SourceSeekingRSS(locations, neighbours)

	# avg_dists = []
	# for experiment in range(num_experiments):
	p0 = RandomPolicy(start_loc0, neighbours)
	p1 = RandomPolicy(start_loc1, neighbours)
	# reset filter state
	x = -20.0 + 40.0 * np.random.uniform(size=(num_particles,2))
	weights = np.repeat(1.0/float(num_particles), num_particles)

	average_distance = run_policy(x, weights, problem, num_steps, source_location, p0, p1, visualize=visualize)
	print('Task completed, average distance to true source location: {:f} meters'.format(average_distance[-1]))

def save_source_locations():
	num_experiments = 1000
	np.random.seed(1234567890)
	source_locations = sample_source_locations(num_experiments)
	np.save(os.path.join(RESULTS_PATH, 'source_locations.npy'), source_locations)

if __name__ == '__main__':
	import argparse
	parser = argparse.ArgumentParser(description='Simulate Dec-POMDP policy and save results')
	parser.add_argument('--numparticles', type=int, help='number of particles to use in filter', default=2000)
	parser.add_argument('--horizon', type=int, required=True, help='problem horizon (how many steps)')
	parser.add_argument('--seed', type=int, help='rng seed', default=1234567890)
	parser.add_argument("--visualize", type=str2bool, nargs='?',
	                        const=True, default=False,
	                        help='visualize particle filter for tracking')
	args = parser.parse_args()

	# Sample random source location and run source localization with random movements
	np.random.seed(args.seed)
	source_location = -20.0 + 40.0 * np.random.uniform(size=(2,))

	run_source_localization(source_location, num_particles=args.numparticles, num_steps=args.horizon, visualize=args.visualize)