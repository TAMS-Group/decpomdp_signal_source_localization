import re
import random

# Reads a policy graph as .dot file produced by NPGI
def read_graph(filename):
	import pydot
	graph = pydot.graph_from_dot_file(filename)[0]

	node_actions = {}
	max_horizon = 0
	for n in graph.get_nodes():
		idx = int(n.get_name())
		d = n.get('label')
		m = re.search(': (.+?)\[', d)
		if m:
		    action = int(m.group(1))
		else:
			action = -1 # end node

		m = re.search('H(.+?)\)', d)
		if m:
			horizon = int(m.group(1))
		else:
			horizon = 0
		if horizon > max_horizon:
			max_horizon = horizon
			start = idx

		node_actions[idx] = action

	edges = []
	for e in graph.get_edges():
		d = e.get('label')
		m = re.search('"(.+?)', d)
		if m:
			observation = int(m.group(1))
		else:
			observation = -1

		src = int(e.get_source())
		trg = int(e.get_destination())

		edges.append((src, trg, observation))

	return node_actions, edges, start, max_horizon

class Policy(object):
	def next_action(self):
		pass

	def next_node(self, observation):
		pass

class RandomPolicy(Policy):
	def __init__(self, start_location, neighbours):
		self.location = start_location
		self.neighbours = neighbours

	def next_action(self):		
		next_location = random.choice(self.neighbours[self.location])
		self.location = next_location
		return next_location

# An open loop policy for given sequence of actions
class SequencePolicy(Policy):
	def __init__(self, sequence):
		self.sequence = sequence
		self.pos = 0

	def next_action(self):
		return self.sequence[self.pos]

	def next_node(self, observation):
		self.pos += 1

# Local policy graph
class DecPOMDPPolicy(Policy):
	def __init__(self, start_node, edges, nodes):
		self.node = start_node
		self.edges = edges
		self.actions = nodes

	def next_action(self):
		return self.actions[self.node]

	def next_node(self, observation):
		for s, t, o in self.edges:
			if s == self.node and o == observation:
				self.node = t
				return
		raise RuntimeError("could not set next node")


class DecPOMDPPolicyWithContinuous(DecPOMDPPolicy):
	def __init__(self, start_node, edges, nodes, high_threshold, low_threshold):
		super().__init__(start_node, edges, nodes)
		self.high_threshold = high_threshold
		self.low_threshold = low_threshold

	def next_node(self, observation):
		if observation >= self.high_threshold:
			# print('{:f} classified as {:d}'.format(observation[0], 0))
			super().next_node(0)
		elif observation <= self.low_threshold:
			# print('{:f} classified as {:d}'.format(observation[0], 2))
			super().next_node(2)
		else:
			# print('{:f} classified as {:d}'.format(observation[0], 1))
			super().next_node(1)