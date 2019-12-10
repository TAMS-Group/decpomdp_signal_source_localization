import numpy as np

def resample(x, weights):
	N = weights.size
	idx = np.random.choice(N, size=N, p=weights)
	x = x[idx,:]
	weights = np.repeat(1.0/float(N), N)
	return x, weights

# Add small Gaussian noise to particles to reinvigorate them
def particle_reinvigoration(x):
	sd = 0.25
	return x + np.random.normal(scale=sd, size=x.shape)


def SIR_step(x, weights, observation_likelihoods, ess_threshold_fraction=0.1):
	num_particles = weights.size
	weights = weights * observation_likelihoods
	weights = weights / weights.sum()
	ess = 1.0 / weights.dot(weights)
	if ess < ess_threshold_fraction * float(num_particles):
		x, weights = resample(x, weights)
		x = particle_reinvigoration(x)
	return x, weights

