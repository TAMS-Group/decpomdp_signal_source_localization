# A Dec-POMDP source seeking problem on a graph
class Problem(object):
	def __init__(self, locations, neighbours):
		self.locations = locations
		self.neighbours = neighbours
	def sample_joint_observation(self, source_xy, l0, l1):
		pass
	def likelihood(self, x, l0, l1, z0, z1):
		pass