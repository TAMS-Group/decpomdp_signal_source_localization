import numpy as np
def SIR_step(x, weights, observation_likelihoods):
	weights = weights * observation_likelihoods 
	weights = weights / weights.sum()
	return x, weights

def initialize(num_of_particles, min_x, min_y, max_x, max_y):
	step_size_x = (max_x - min_x)/np.sqrt(num_of_particles)
	step_size_y = (max_y - min_y)/np.sqrt(num_of_particles)
	particles = np.mgrid[min_x:max_x:step_size_x, min_y:max_y:step_size_y].reshape(2,-1).T
	idx = np.arange(0, 1000, 1)
	particles = particles[idx,:]
	weights = np.repeat(1.0/float(num_of_particles), num_of_particles)
	
	return particles, weights


