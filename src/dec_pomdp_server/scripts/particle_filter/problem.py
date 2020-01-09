# A Dec-POMDP source seeking problem on a graph
class Problem(object):
	def __init__(self, locations, neighbours):
		self.locations = locations
		self.neighbours = neighbours
	def likelihood(self, x, l0, z0):
		pass
