import numpy as np

def weightedL2(a,b,w):
	q = a-b
	return np.sqrt((w.reshape(-1,1)*q*q).sum())

def compute_distance(a,b):
	assert a.shape[1] == 2, "a shape must be [n, 2]"
	assert b.shape[1] == 2, "b shape must be [n, 2]"
	if a.shape[0] != b.shape[0]:
		assert a.shape[0] == 1 or b.shape[0] == 1, "either a or b must be [1,2] if both are not [n,2]"
	D = np.linalg.norm(a-b, ord=2, axis=1)
	return D