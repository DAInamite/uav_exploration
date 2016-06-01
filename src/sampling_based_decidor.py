# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg


GENERATION_SIZE = 20
GENERATIONS = 6

# ENUMS
UNKNOWN = -1	# no information yet
OPEN = 0		# open space
OBSTACLE = 1	# blocked space

import math, random, threading, time
import rospy

# TODO:
# - check world coordinates scaling
#   -> altitude
#   -> visibility range

def rotation_diff(a, b):
	'''returns relative angle difference from b to a'''
	return math.atan2(math.sin(a-b), math.cos(a-b))

def frange(start, end, step):
	'''float version of xrange'''
	f = start
	while f < end:
		yield f
		f += step


class OneTimePublisher(threading.Thread):
	'''class to publish data of a given type under a ROS topic once.'''

	def __init__(self, topic, type, data):
		super(OneTimePublisher, self).__init__()
		self.publisher = rospy.Publisher(topic, type, queue_size=1)
		self.data = data

	def run(self):
		# wait a bit to have the publisher register itself
		time.sleep(1)
		self.publisher.publish(self.data)


class Decidor(object):
	'''class to find out the best waypoint based on the current copter
	   configuration. one instance per decision.'''

	def __init__(self, grid, position, rotation):
		self.grid = grid
		self.grid_offset = self.grid.info.origin.position.x, self.grid.info.origin.position.y
		self.position = position
		self.rotation = rotation
		# outer config
		self.field_of_view_h = float(rospy.get_param('field_of_view_h', 58)) / 360. * math.pi * 2
		self.field_of_view_v = float(rospy.get_param('field_of_view_v', 45)) / 360. * math.pi * 2
		self.camera_rotation = float(rospy.get_param('camera_rotation', -45)) / 360. * math.pi * 2
		self.altitude = float(rospy.get_param('altitude', 1.5))
		self.translation_speed = float(rospy.get_param('translation_speed', 1.))
		self.rotation_speed = float(rospy.get_param('rotation_speed', 40)) / 360. * math.pi * 2
		# compute frontiers
		self.frontiers = list(self._iter_frontiers_())
		self.publish_frontiers()
		# planar visibility range
		self.visrange_min = math.tan(-self.field_of_view_v/2. - self.camera_rotation) * self.altitude
		self.visrange_max = math.tan(self.field_of_view_v/2. - self.camera_rotation) * self.altitude

	def publish_frontiers(self):
		point_cloud = PointCloud()
		point_cloud.header = std_msgs.msg.Header()
		point_cloud.header.stamp = rospy.Time.now()
		point_cloud.header.frame_id = '/orb_slam/map'
		off_x, off_y = self.grid.info.origin.position.x, self.grid.info.origin.position.y
		point_cloud.points = [Point32(
				(x+.5) * self.grid.info.resolution + off_x,
				(y+.5) * self.grid.info.resolution + off_y, 0.0
			) for (y, x) in self.frontiers
		]
		publisher = OneTimePublisher("exploration_frontiers", PointCloud, point_cloud)
		publisher.start()

	def get_grid_string(self):
		'''returns a string representation of the grid'''
		s = ''
		w, h = self.grid.info.width, self.grid.info.height
		for x in xrange(0, w):
			for y in xrange(0, h):
				c = self.get_cell_status((x, y))
				if c == OPEN:
					s += '□'
				elif c == OBSTACLE:
					s += '■'
				elif c == UNKNOWN:
					s += ' '
				else:
					raise ValueError
			s += '\n'
		return s

	def get_cell_status(self, index):
		'''get cell status from index'''
		x, y = index
		w, h = self.grid.info.width, self.grid.info.height
		index = x*w+y
		if index < 0 or index >= w*h:
			# out of bounds? unknown.
#			print 'OUT OF BOUNDS!'
			return UNKNOWN
		# return cell value
		value = self.grid.data[index]
		if value == -1:
#			print 'YET UNKNOWN!'
			return UNKNOWN
		elif value >= 0 and value < 50:
#			print 'OPEN!'
			return OPEN
		else:
#			print 'OBSTACLE!'
			return OBSTACLE

	def get_cell_at(self, position):
		'''get cell index from position'''
		# get offset
		ox, oy, = self.grid_offset
		# compute cell index
		cell_size = self.grid.info.resolution
		x, y = position
		x = int((x-ox) / cell_size)
		y = int((y-oy) / cell_size)
		return x, y

	def get_cell_position(self, index):
		'''get cell center position from index'''
		# get offset
		ox, oy, = self.grid_offset
		# compute cell position
		cell_size = self.grid.info.resolution
		x, y = index
		x = x*cell_size + ox
		y = y*cell_size + oy
		return x, y

	def _iter_frontiers_(self):
		w, h = self.grid.info.width, self.grid.info.height
		for x in xrange(0, w):
			for y in xrange(0, h):
				if self.get_cell_status((x, y)) == OPEN:
					if (self.get_cell_status((x-1, y)) == UNKNOWN or 
							self.get_cell_status((x+1, y)) == UNKNOWN or 
							self.get_cell_status((x, y+1)) == UNKNOWN or 
							self.get_cell_status((x, y-1)) == UNKNOWN):
						#print 'frontier:', (x, y)
						yield (x, y)

	def iter_visible_cells(self, position, rotation):
		'''yields possibly visible cells given a planar robot configuration assuming
		   flat ground.'''
		# TODO: quadrants
		rx, ry = position
		# iterate over bounding box of cells
		for x in frange(rx - self.visrange_max, rx + self.visrange_max, self.grid.info.resolution):
			for y in frange(ry - self.visrange_max, ry + self.visrange_max, self.grid.info.resolution):
				dist = math.sqrt((x-rx)**2 + (y-ry)**2)
				if (dist > self.visrange_min and dist < self.visrange_max and
						abs(rotation_diff(math.atan2(y-ry, x-rx), rotation)) < self.field_of_view_h/2.):
					# cell is within visible range and distance
					yield self.get_cell_at((x, y))

	def get_utility(self, position):
		'''returns utility value of a potential robot configuration.'''
		# rotation to position
		px, py = position
		rx, ry, _ = self.position
		rotation = math.atan2(py-ry, px-rx)
		# count possible newly discovered cells
		new_cells = 0
		for cell in self.iter_visible_cells(position, rotation):
			if self.get_cell_status(cell) == OPEN:
				new_cells += 1
		# transfer cost
		distance = math.sqrt((px-rx)**2 + (py-ry)**2)
		alpha = abs(rotation_diff(rotation, self.rotation))
		# combine
		seconds = (distance / self.translation_speed +
			alpha / self.rotation_speed)
		return (math.e**(-seconds/2.) *	# penalize time needed to travel to
			math.e**(-alpha) *			# also penalize rotation generally
			new_cells)					# fresh cells... feelsgoodman.jpg

	def get_initial_samples(self):
		'''creates a first generation of robot configuration samples.'''
		PADDING = 1.2
		# TODO: basic "fly forward" sample
		samples = []
		shots = 0
		hits = 0
		# random frontier based samples
		while len(samples) < GENERATION_SIZE*6:
			# frontier coords
			frontier = random.choice(self.frontiers)
			fx, fy = self.get_cell_position(frontier)
			rot = random.random() * math.pi * 2
			# sample coords
			sx, sy = fx + math.cos(rot)*PADDING, fy + math.sin(rot)*PADDING
			cell = self.get_cell_at((sx, sy))
			shots += 1
			if self.get_cell_status(cell) == OPEN:
				hits += 1
				# only add sample position when cell is open
				samples.append(((sx, sy), self.get_utility((sx, sy))))
		print '%i%% open cell hits' % (100. * hits / shots)
		return sorted(samples, key=lambda (sample, util): -util)[:GENERATION_SIZE]

	def next_generation(self, samples):
		'''adds mutated samples and returns next generation pool'''
		new_samples = []
		for (x, y), _ in samples:
			# variate position
			while True:
				alpha = random.random() * math.pi * 2
				delta = random.gauss(0, 0.5)
				new_x = x + math.cos(alpha)*delta
				new_y = y + math.sin(alpha)*delta
				new_cell = self.get_cell_at((new_x, new_y))
				if self.get_cell_status(new_cell) == OPEN:
					break
			new_samples.append(((new_x, new_y), self.get_utility((new_x, new_y))))
		samples = sorted(samples + new_samples, key=lambda (sample, util): -util)
		return samples[:GENERATION_SIZE]

	def decide_waypoint(self):
		'''get best waypoint to move to'''
		if len(self.frontiers) == 0:
			# no frontiers left? kthxbye.
			return
		# get some initial samples
		print 'creating initial samples...'
		samples = self.get_initial_samples()
		print 'mutating...'
		for x in xrange(GENERATIONS):
			#print 'generation', x
			samples = self.next_generation(samples)
		return sorted(samples, key=lambda (sample, util): -util)[0]
