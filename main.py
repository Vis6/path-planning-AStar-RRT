"""
Implementation of two path finding algorithms using A* and RRT.
"""

# library
import numpy as np
import math
from matplotlib import pyplot as plt
import os
import random
import sys


class GridWorld:
	def __init__(self, show_animation=False, save_fig=False):
		self.show_animation = show_animation
		self.save_fig = save_fig

		# position of source and destination
		self.x_source = 10
		self.y_source = 10
		self.x_dest = 50
		self.y_dest = 50

		# boundary and obstacle of the grid world
		self.x_min, self.y_min = 0, 0
		self.x_max, self.y_max = 60, 60
		self.x_len = self.x_max - self.x_min
		self.y_len = self.y_max - self.y_min
		self.map = np.zeros((self.x_len + 1, self.y_len + 1), dtype=bool)
		self.map[self.x_min, :] = True
		self.map[self.x_max, :] = True
		self.map[:, self.y_min] = True
		self.map[:, self.y_max] = True
		self.map[20, :40] = True
		self.map[40, 21:] = True

	def show_grid_world(self, fig_num, name=None):
		plt.figure()

		# plot boundary
		m, n = self.map.shape
		for i in range(m):
			for j in range(n):
				if self.map[i, j]:
					plt.plot(i, j, '.k')

		# plot source and destination
		plt.plot(self.x_source, self.y_source, '+g')
		plt.plot(self.x_dest, self.y_dest, 'xb')
		plt.plot()
		plt.grid(True)
		plt.axis('equal')

		if self.save_fig:
			if name is None:
				plt.savefig('./gif/grid_world.jpg')
			else:
				if not os.path.exists('./gif/%s' % name):
					os.makedirs('./gif/%s' % name)
				plt.savefig('./gif/%s/%d.jpg' % (name, fig_num))

		if not self.show_animation:
			plt.show()


class AStarPathPlanner:
	def __init__(self, world, show_animation=False, save_fig=False):
		self.show_animation = show_animation
		self.save_fig = save_fig
		self.env = world
		self.env.show_animation = self.show_animation
		self.env.save_fig = self.save_fig
		self.motion = self.get_motion()
		self.path_x = None
		self.path_y = None
		self.fig_num = 0

	class Node:
		def __init__(self, x, y, g_cost, h_cost, parent_index):
			self.x = x  # index of grid
			self.y = y  # index of grid
			self.g_cost = g_cost
			self.h_cost = h_cost
			self.f_cost = self.g_cost + self.h_cost
			self.parent_index = parent_index

		def __str__(self):
			return str(self.x) + "," + str(self.y) + "," + str(self.g_cost) + "," + str(self.h_cost) + "," + str(
				self.f_cost) + "," + str(self.parent_index)

	@staticmethod
	def get_motion():
		# dx, dy, cost
		motion = [[1, 0, 1],
		          [0, 1, 1],
		          [-1, 0, 1],
		          [0, -1, 1],
		          [-1, -1, math.sqrt(2)],
		          [-1, 1, math.sqrt(2)],
		          [1, -1, math.sqrt(2)],
		          [1, 1, math.sqrt(2)]]
		return motion

	def path_planning(self):
		# source and destination positions
		x_s = self.env.x_source
		y_s = self.env.y_source
		x_d = self.env.x_dest
		y_d = self.env.y_dest
		s_node = self.Node(x_s, y_s, 0.0, 0.0, -1)
		d_node = self.Node(x_d, y_d, 0.0, 0.0, -1)
		del x_s, y_s, x_d, y_d

		# create open set and closed set
		open_set, closed_set = dict(), dict()
		open_set['%d,%d' % (s_node.x, s_node.y)] = s_node

		# plot grid world
		if self.show_animation:
			self.env.show_grid_world(self.fig_num, 'a_star')
			self.fig_num += 1

		# loop
		while True:
			# if open set is empty, jump out
			if len(open_set) == 0:
				break

			# return the node in the open set with the lowest f_cost
			current_id = min(open_set, key=lambda o: open_set[o].f_cost)
			current = open_set[current_id]

			# remove current node from open set and add it to closed set
			del open_set[current_id]
			closed_set[current_id] = current

			# plot current node in the grid world figure
			if self.show_animation:
				plt.plot(current.x, current.y, 'xc')

				if self.save_fig:
					plt.savefig('./gif/a_star/%d.jpg' % self.fig_num)
					self.fig_num += 1

				if len(closed_set.keys()) % 10 == 0:
					plt.pause(0.001)

			# if current node is the destination, break
			if current.x == d_node.x and current.y == d_node.y:
				d_node.parent_index = current.parent_index
				d_node.f_cost = current.f_cost
				break

			# for each neighbor of the current node
			for i, _ in enumerate(self.motion):
				new_x = current.x + self.motion[i][0]
				new_y = current.y + self.motion[i][1]
				node = self.Node(new_x,
				                 new_y,
				                 current.g_cost + self.motion[i][2],
				                 self.calc_heuristic(d_node, new_x, new_y),
				                 current_id)
				node_id = '%d,%d' % (node.x, node.y)

				# if the node is not traversable or is in closed set, skip
				# else add new node or compare with the existing one
				if not self.is_traversable(node) or node_id in closed_set:
					continue
				else:
					if node_id not in open_set:
						open_set[node_id] = node
					else:
						if open_set[node_id].f_cost > node.f_cost:
							open_set[node_id] = node

		# generate the path
		self.generate_path(d_node, closed_set)

		# plot path
		plt.plot(self.path_x, self.path_y, '-r')
		if self.save_fig:
			plt.savefig('./gif/a_star/%d.jpg' % self.fig_num)
		plt.pause(0.001)
		plt.show()

	def calc_heuristic(self, dest, x_neig, y_neig):
		x_dest = dest.x
		y_dest = dest.y
		x_delta = abs(x_neig - x_dest)
		y_delta = abs(y_neig - y_dest)

		if x_delta == y_delta:
			d = math.hypot(x_delta, y_delta)
		else:
			delta_min = min(x_delta, y_delta)
			d = abs(x_delta - y_delta) + math.hypot(delta_min, delta_min)
		return d

	def is_traversable(self, node):
		if node.x < self.env.x_min or node.y < self.env.y_min or node.x > self.env.x_max or node.y > self.env.y_max:
			# out of the grid world
			return False
		elif self.env.map[node.x, node.y]:  # obstacle detect
			return False
		else:
			return True

	def generate_path(self, dest, closed_set):
		x_path = [dest.x]
		y_path = [dest.y]
		parent_index = dest.parent_index
		while parent_index != -1:
			node = closed_set[parent_index]
			x_path.append(node.x)
			y_path.append(node.y)
			parent_index = node.parent_index
		self.path_x = x_path
		self.path_y = y_path


class RapidlyExploringRandomTree:
	def __init__(self, world, dist_to_dest=2, max_iter=200000, delta_dist=2, prob=0.5, show_animation=False,
	             save_fig=False):
		self.show_animation = show_animation
		self.save_fig = save_fig
		self.env = world
		self.env.show_animation = self.show_animation
		self.env.save_fig = self.save_fig
		self.dist_to_dest = dist_to_dest  # distance to the destination
		self.max_iter = max_iter
		self.delta_dist = delta_dist  # incremental distance
		self.prob = prob  # used in random node generation
		self.node_list = {}
		self.s_node = None  # source node
		self.d_node = None  # destination node
		self.path_x = None
		self.path_y = None
		self.fig_num = 0

	class Node:
		def __init__(self, x, y, parent_id=None):
			self.id = '%d,%d' % (x, y)
			self.x = x  # index of grid
			self.y = y  # index of grid
			self.parent_id = parent_id

		def __str__(self):
			return self.id + ',' + str(self.x) + ',' + str(self.y) + ',' + str(self.parent_id)

	def path_planning(self):
		is_path_found = False
		counter = 0  # initial counter
		x_s = self.env.x_source
		y_s = self.env.y_source
		x_d = self.env.x_dest
		y_d = self.env.y_dest
		s_node = self.Node(x_s, y_s)
		d_node = self.Node(x_d, y_d)
		self.s_node = s_node
		self.d_node = d_node
		self.node_list['%d,%d' % (x_s, y_s)] = s_node

		# plot grid world
		if self.show_animation:
			self.env.show_grid_world(self.fig_num, 'rrt')
			self.fig_num += 1

		while counter < self.max_iter:
			counter += 1
			rnd_node = self.generate_random_node()  # generate random node
			nearest = self.find_nearest_node(self.node_list, rnd_node)  # find the nearest node in the tree
			if self.calc_dist_to_dest(nearest) < self.dist_to_dest:
				self.d_node.parent_id = nearest.id
				is_path_found = True
				break
			new_node = self.extend_tree(nearest, rnd_node)  # extend tree
			if new_node is not None:
				self.node_list[new_node.id] = new_node

				# plot current node in the grid world figure
				if self.show_animation:
					plt.plot(new_node.x, new_node.y, 'xc')

					if self.save_fig:
						plt.savefig('./gif/rrt/%d.jpg' % self.fig_num)
						self.fig_num += 1

					if len(self.node_list) % 10 == 0:
						plt.pause(0.001)

		if not is_path_found:  # check if path is found
			print("RRT: Path is not found!")
			sys.exit(-1)

		# generate path
		self.generate_path()

		# plot path
		plt.plot(self.path_x, self.path_y, '-r')
		if self.save_fig:
			plt.savefig('./gif/rrt/%d.jpg' % self.fig_num)
		plt.pause(0.001)
		plt.show()

	def generate_random_node(self):
		p = random.random()
		if p < self.prob:
			# randomly generate node position in the boundary
			x = random.randint(self.env.x_min + 1, self.env.x_max - 1)
			y = random.randint(self.env.y_min + 1, self.env.y_max - 1)

			# if node exists, regenerate
			while self.node_exist(x, y):
				x = random.randint(self.env.x_min + 1, self.env.x_max - 1)
				y = random.randint(self.env.y_min + 1, self.env.y_max - 1)

			return self.Node(x, y)
		else:
			return self.d_node

	@staticmethod
	def find_nearest_node(rrt_tree, node):
		nearest_node_id = None
		min_dist = -1
		for node_id in rrt_tree:
			node_in_tree = rrt_tree[node_id]
			if min_dist == -1:  # init
				nearest_node_id = node_in_tree.id
				min_dist = math.hypot(node_in_tree.x - node.x, node_in_tree.y - node.y)
			else:
				temp_dist = math.hypot(node_in_tree.x - node.x, node_in_tree.y - node.y)
				if temp_dist < min_dist:
					min_dist = temp_dist
					nearest_node_id = node_in_tree.id
		return rrt_tree[nearest_node_id]

	def calc_dist_to_dest(self, node):
		return math.hypot(node.x - self.d_node.x, node.y - self.d_node.y)

	def extend_tree(self, nearest_node, rnd_node):
		new_node = None

		# calculate new node's position
		gain = self.delta_dist / math.hypot(nearest_node.x - rnd_node.x, nearest_node.y - rnd_node.y)
		x_new = round(gain * (nearest_node.x + (rnd_node.x - nearest_node.x)))
		y_new = round(gain * (nearest_node.y + (rnd_node.y - nearest_node.y)))

		if self.is_out_of_boundary(x_new, y_new) or self.node_exist(x_new, y_new):  # in the grid world or node exists
			return new_node
		if not self.detect_obstacle(x_new, y_new, nearest_node):
			new_node = self.Node(x_new, y_new, parent_id=nearest_node.id)
		return new_node

	def generate_path(self):
		x_path = [self.d_node.x]
		y_path = [self.d_node.y]
		parent_id = self.d_node.parent_id
		while parent_id is not None:
			node = self.node_list[parent_id]
			x_path.append(node.x)
			y_path.append(node.y)
			parent_id = node.parent_id
		self.path_x = x_path
		self.path_y = y_path

	def detect_obstacle(self, x_pos, y_pos, node):
		if self.env.map[x_pos, y_pos]:  # collision with obstacle
			return True
		else:
			if (20 - node.x) * (20 - x_pos) < 0:
				y = node.y + (y_pos - node.y) / (x_pos - node.x) * (20 - node.x)
				if y < 40:
					return True
			if (40 - node.x) * (40 - x_pos) < 0:
				y = node.y + (y_pos - node.y) / (x_pos - node.x) * (40 - node.x)
				if y > 20:
					return True
		return False

	def node_exist(self, x_pos, y_pos):
		if '%d,%d' % (x_pos, y_pos) in self.node_list.keys():
			return True
		else:
			return False

	def is_out_of_boundary(self, x_pos, y_pos):
		if x_pos < self.env.x_min or y_pos < self.env.y_min or x_pos > self.env.x_max or y_pos > self.env.y_max:
			return True
		else:
			return False


if __name__ == '__main__':
	# anim = True  # show animation
	grid_world = GridWorld()  # create grid world

	# A star
	a_star = AStarPathPlanner(grid_world, show_animation=True, save_fig=False)
	a_star.path_planning()

	# RRT
	rrt = RapidlyExploringRandomTree(grid_world, show_animation=True, save_fig=False)
	rrt.path_planning()
