"""
Implementation of two path finding algorithms using A* and RRT.
"""

# library
import numpy as np
import math
from matplotlib import pyplot as plt
import os


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
		self.map[0, :] = True
		self.map[60, :] = True
		self.map[:, 0] = True
		self.map[:, 60] = True
		self.map[20, :49] = True
		self.map[40, 21:] = True

		if self.show_animation:
			self.show_grid_world()

	def show_grid_world(self):
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
			if not os.path.exists('./gif/a_star'):
				os.makedirs('./gif/a_star')
			if not os.path.exists('./gif/rrt'):
				os.makedirs('./gif/rrt')
			plt.savefig('./gif/a_star/0.jpg')
			plt.savefig('./gif/rrt/0.jpg')

		if not self.show_animation:
			plt.show()


class AStarPathPlanner:
	def __init__(self, world, show_animation=False, save_fig=False):
		self.show_animation = show_animation
		self.save_fig = save_fig
		self.env = world
		self.motion = self.get_motion()
		self.path_x = None
		self.path_y = None
		self.fig_num = 1

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
	def __init__(self, world, show_animation=False, save_fig=False):
		self.show_animation = show_animation
		self.save_fig = save_fig
		self.fig_num = 1
		self.env = world

	def path_planning(self):
		pass


if __name__ == '__main__':
	# anim = True  # show animation
	grid_world = GridWorld(show_animation=True, save_fig=False)  # create grid world

	# A star
	a_star = AStarPathPlanner(grid_world, show_animation=True, save_fig=False)
	a_star.path_planning()

	# RRT
	rrt = RapidlyExploringRandomTree(grid_world, show_animation=False, save_fig=False)
	rrt.path_planning()
