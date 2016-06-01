#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math, random, sys
from collections import deque
from util import *

from robot_config import RobotConfig

#from decidors import frontier_util as decision
from decidors import srt as decision
#from decidors import scan as decision
#from decidors import concentric as decision
#from decidors import concentric_util as decision
#from decidors import sampling_based as decision
#from decidors import random_flight as decision

config = {
	# +++ WORLD SETUP +++
	'world_width' : 40,		# meters
	'world_height' : 30,
	'cell_size' : .1,

	# +++ SCENARIO +++
	'start_position' : [5.1, 2.1, 1.33939], #[5., 7., 1.33939],
	'start_rotation' : 3.2959, #math.pi / 4,
	'object1' : (0, 0),
	'object2' : (0, 1),
	'object3' : (0, 2),
}

# ENUMS
CELL_UNKNOWN = -1	# no information yet
CELL_OPEN = 0		# open space
CELL_OBSTACLE = 1	# blocked space


class Stats(object):
	'''class to hold and manage ALL teh stats!!!'''
	PROGRESS_TICK = .5		# interval in s in which to propagate progress info
	def __init__(self):
		self.rotation = 0.
		self.translation = 0.
		self.time = 0.
		self.visited_voxels = 0
		self.progress = []
		self.__last_progress_tick__ = -Stats.PROGRESS_TICK
		self.object_found_times = []

	def progress_snapshot(self):
		if self.__last_progress_tick__ + Stats.PROGRESS_TICK <= self.time:
			self.__last_progress_tick__ += Stats.PROGRESS_TICK
			self.progress.append(self.visited_voxels)
			print >> sys.stderr, 'cells visited', self.visited_voxels

	def __repr__(self):
		return (
			'Time needed: %.2f s\nDistance travelled: %.2f m\nTotal rotation: %.1f°\n'
			% (self.time, self.translation, self.rotation / (math.pi * 2.) * 360.)
			+ 'Objects found after: %s\n\n' % ', '.join(map(lambda t: '%.2f s' % t, self.object_found_times))
			+ '+++ following progress ticks +++\n' + '\n'.join(map(str, self.progress))
		)

class Grid(object):
	'''class to hold the cell plane'''
	def __init__(self, width, height, cell_size):
		self.cell_size = cell_size
		self.width = width
		self.height = height
		self.matrix = [[CELL_UNKNOWN] * width for _ in xrange(height)]

	@staticmethod
	def neighbors_iter(cell, yield_self=False):
		'''yields all 8 neighbor cells, optionally also itself'''
		x, y = cell
		for nx, ny in [(x+1, y), (x+1, y+1), (x, y+1), (x-1, y+1), (x-1, y), (x-1, y-1), (x, y-1), (x+1, y-1)]:
			yield nx, ny
		if yield_self:
			yield cell

	@staticmethod
	def environment_iter(cell, distance):
		'''iterates over the cells in local environment within a given
		   distance using BFS.'''
		# starting cell coords
		vx, vy = cell[0]*self.cell_size, cell[1]*self.cell_size
		# BFS
		q = deque([cell])
		visited = set([cell])
		while len(q) > 0:
			ccell = q.popleft()
			# current cell's coords
			cx, cy = ccell[0]*self.cell_size, ccell[1]*self.cell_size
			# distance from current cell to starting cell
			cdist = math.sqrt((cx-vx)**2 + (cy-vy)**2)
			if cdist <= distance:
				yield ccell
				# add neighbors to queue
				for ncell in Grid.neighbors_iter(ccell):
					if ncell not in visited:
						q.append(ncell)
						visited.add(ncell)

	def set_visited(self, cell):
		'''marks given cell as visited or obstacle respectively'''
		if not self.is_out_of_bounds(cell):
			x, y = cell
			self.matrix[y][x] = CELL_OBSTACLE if self.is_obstacle_cell(cell) else CELL_OPEN

	def get_cell_status(self, cell):
		'''returns if a cells was visited before and wheather it is an obstacle'''
		x, y = cell
		w, h = self.get_size()
		return CELL_UNKNOWN if self.is_out_of_bounds(cell) else self.matrix[y][x]

	def is_obstacle_cell(self, cell):
		'''returns if a cell is a border cell'''
		x, y = cell
		w, h = self.get_size()
		return (x == 0 or y == 0 or x == w-1 or y == h-1)

	def is_out_of_bounds(self, cell):
		'''returns if a cell is an external cell'''
		x, y = cell
		w, h = self.get_size()
		return (x < 0 or y < 0 or x >= w or y >= h)

	def get_size(self):
		'''returns grid width and height in cells'''
		return self.width, self.height

	def get_size_metric(self):
		'''returns grid width and height in meters'''
		return self.width * self.cell_size, self.height * self.cell_size

	def get_cell_index(self, position):
		'''metric position -> cell index'''
		x, y = position
		return int(x / self.cell_size), int(y / self.cell_size)

	def get_cell_center(self, voxel):
		'''cell index -> metric position'''
		x, y = voxel
		return x*self.cell_size + self.cell_size/2, y*self.cell_size + self.cell_size/2

class Robot(RobotConfig):
	'''robot class with position, orientation and goals.
	   see robot_config.py for static values like camera setup.'''
	def __init__(self, position, rotation, stats_object):
		super(Robot, self).__init__()
		self.position = list(position)
		self.rotation = rotation
		self.position_history = [(self.position[:2], self.rotation)]
		self.stats = stats_object
		self.target_position = None
		self.target_rotation = None
		self.busy = True

	def is_moving(self):
		return (self.position != self.target_position
			or self.rotation != self.target_rotation)

	def move_order(self, position=None, rotation=None):
		'''sets a new target for the robot to travel to'''
		if position == None:
			self.target_position = self.position
		else:
			self.target_position = list(position)
		if rotation == None:
			self.target_rotation = self.rotation
		else:
			self.target_rotation = rotation % (math.pi*2)

	def move(self, timespan):
		'''moves the robot in the world w.r.t the time which passed since the
		   last move() call.'''
		# translation
		if not self.target_position is None:
			rx, ry, rz = self.position
			tx, ty, tz = self.target_position
			vector = [tx-rx, ty-ry, tz-rz]
			length = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)

			if length > self.TRANSLATION_SPEED*timespan:
				# regular step (further than one translation tick)
				vector[0] *= self.TRANSLATION_SPEED*timespan/length
				vector[1] *= self.TRANSLATION_SPEED*timespan/length
				vector[2] *= self.TRANSLATION_SPEED*timespan/length
				self.position[0] += vector[0]
				self.position[1] += vector[1]
				self.position[2] += vector[2]
				self.stats.translation += self.TRANSLATION_SPEED*timespan
			else:
				# final step (smaller than one translation tick)
				self.position = self.target_position
				self.target_position = None
				self.stats.translation += length
				# add to history
				self.position_history.append((list(self.position[:2]), self.rotation))

		# rotation
		if not self.target_rotation is None:
			rotation_diff = abs(get_rotation_diff(self.target_rotation, self.rotation))

			if rotation_diff > self.ROTATION_SPEED*timespan:
				# regular step (larger than one rotation tick)
				if get_rotation_diff(self.target_rotation, self.rotation) < 0:
					self.rotation -= self.ROTATION_SPEED*timespan
				else:
					self.rotation += self.ROTATION_SPEED*timespan
				self.stats.rotation += abs(self.ROTATION_SPEED*timespan)
			else:
				# final step (smaller than one rotation tick)
				self.rotation = self.target_rotation
				self.target_rotation = None
				self.stats.rotation += rotation_diff

				if self.target_position is None:
					# rotation took longer than translation. store in history
					self.position_history.append((list(self.position[:2]), self.rotation))

	def rays_iter(self):
		'''yields one ray (vector) per camera pixel'''
		for x in xrange(self.CAMRAYS_H):
			# horizontal ray angle
			h = self.rotation - self.FIELD_OF_VIEW_H / 2. + self.FIELD_OF_VIEW_H * x / (self.CAMRAYS_H-1)
			for y in xrange(self.CAMRAYS_V):
				# vertical ray angle
				v = self.CAMERA_ROTATION - self.FIELD_OF_VIEW_V / 2. + self.FIELD_OF_VIEW_V * y / (self.CAMRAYS_V-1)
				# initial vector facing "straight"
				rx, ry, rz = self.CAMRAY_LENGTH, 0., 0.
				# rotate vertically (around y axis)
				rx, ry, rz = math.cos(v) * rx, 0., math.sin(v) * rx
				# rotate horizontally (around z axis)
				rx, ry, rz = math.cos(h) * rx, math.sin(h) * rx, rz
				yield rx, ry, rz
		# note from the past: this is correct!

class World(object):
	def __init__(self, stats_object, drawables_object):
		self.stats = stats_object
		self.drawables = drawables_object

		# metric world values
		self.WORLD_WIDTH = config['world_width']
		self.WORLD_HEIGHT = config['world_height']
		cell_size = config['cell_size']

		# create a grid depending on metric dimensions and cell size
		self.grid = Grid(int(self.WORLD_WIDTH / cell_size), int(self.WORLD_HEIGHT / cell_size), cell_size)

		# randomize robot position
		rx, ry, rz = 7 + random.random()*(self.WORLD_WIDTH - 14), 7 + random.random()*(self.WORLD_HEIGHT - 14), 1.33939
		rot = random.random() * math.pi * 2
		if 'start_position' in config:
			rx, ry, rz = config['start_position']
		if 'start_rotation' in config:
			rot = config['start_rotation']
		self.robot = Robot((rx, ry, rz), rot, stats_object)

		# "hide" three objects
		if 'object1' in config and 'object2' in config and 'object3' in config:
			self.objects = [config['object1'], config['object2'], config['object3']]
		else:
			w, h = self.grid.get_size()
			self.objects = []
			for x in xrange(3):
				while True:
					voxel = random.randrange(w), random.randrange(h)
					if voxel not in self.objects:	# distinct positions
						self.objects.append(voxel)
						break

	def get_ray_impact(self, ray):
		'''returns (x, y) where the ray hits z=0'''
		rx, ry, rz = ray
		x, y, z = self.robot.position

		if z + rz > 0:
			return None

		f = -z / rz
		return x+rx*f, y+ry*f

	def perceive(self):
		'''sends out camera rays and discovers cells'''
		cells = set()
		# cast rays
		for ray in self.robot.rays_iter():
			# get impact coords
			impact = self.get_ray_impact(ray)
			if impact: # close enough!
				# get corresponding cell
				cell = self.grid.get_cell_index(impact)
				if cell not in cells:
					cells.add(cell)
					# TODO: on an unknown map, this won't work. cast perception rays cell-wise instead.
					if self.grid.get_cell_status(cell) == CELL_UNKNOWN and not self.grid.is_out_of_bounds(cell):
						# found new cell
						self.grid.set_visited(cell)
						self.stats.visited_voxels += 1
						if cell in self.objects:
							# found hidden object
							self.stats.object_found_times.append(self.stats.time)
		self.drawables.visible_cells = cells


class Simulation:
	def __init__(self, drawables_object=None):
		if drawables_object is None:
			import display
			self.drawables = display.Drawables()
		else:
			self.drawables = drawables_object
		self.stats = Stats()
		self.world = World(self.stats, self.drawables)
		self.decidor = decision.Decidor(self.drawables)

	def tick(self):
		'''simulates everything for the duration of one camera frame'''
		self.world.perceive()
		robot = self.world.robot
		if not robot.target_position and not robot.target_rotation:
			waypoint = self.decidor.decide(self.world.grid, robot.position, robot.rotation)
			if waypoint is None:
				robot.busy = False
			else:
				(x, y), r = waypoint
				robot.move_order((x, y, robot.position[2]), r)
		robot.move(1./robot.CAM_FPS)
		self.stats.time += 1./robot.CAM_FPS
		self.stats.progress_snapshot()


if __name__ == '__main__':
	print '+++ SRT SIMULATION +++'
	if len(sys.argv) == 10:
		rx, ry, rot = map(float, sys.argv[1:4])
		o1x, o1y, o2x, o2y, o3x, o3y = map(int, sys.argv[4:10])
		_, _, rz = config['start_position']
		config['start_position'] = [rx, ry, rz]
		config['start_rotation'] = rot
		config['object1'] = (o1x, o1y)
		config['object2'] = (o2x, o2y)
		config['object3'] = (o3x, o3y)
		# TODO: scenario configs in Konstruktor verschieben
		#       Modul als Klasse. Output wahlweise als Attribute.
	elif len(sys.argv) == 1:
		pass
	else:
		print 'wrong number of arguments'
		exit(1)
	simulation = Simulation()
	while simulation.world.robot.busy:
		simulation.tick()
	sys.stderr.close()
	print '+++ position history +++\n' + str(simulation.world.robot.position_history)
	print '\n+++ STATS +++\n' + str(simulation.stats)
