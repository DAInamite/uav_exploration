import math, random
from util import *
import robot_config

# ENUMS
CELL_UNKNOWN = -1	# no information yet
CELL_OPEN = 0		# open space
CELL_OBSTACLE = 1	# blocked space

class AbstractDecidor(object):
	def __init__(self, drawables_object):
		self.drawables = drawables_object
		self.grid = None
		self.position = None
		self.rotation = None
		self.robot_config = robot_config.RobotConfig()

	def decide(self, grid, position, rotation):
		self.grid = grid
		self.position = position
		self.rotation = rotation

	def get_visible_cells(self, position, rotation):
		'''returns possibly visible cells given a planar robot configuration assuming
		   flat ground. (maximum: 411.61)'''
		# TODO: quadrants
		# compute min and max planar cell distance from robot
		rx, ry, rz = position
		alpha = -self.robot_config.FIELD_OF_VIEW_V/2. - self.robot_config.CAMERA_ROTATION
		beta = self.robot_config.FIELD_OF_VIEW_V/2. - self.robot_config.CAMERA_ROTATION
		min_dist = math.tan(alpha) * rz
		max_dist = math.tan(beta) * rz
		# iterate over bounding box of cells
		for x in frange(rx - max_dist, rx + max_dist, self.grid.cell_size):
			for y in frange(ry - max_dist, ry + max_dist, self.grid.cell_size):
				dist = math.sqrt((x-rx)**2 + (y-ry)**2)
				if (dist > min_dist and dist < max_dist and
					abs(get_rotation_diff(math.atan2(y-ry, x-rx), rotation)) < self.robot_config.FIELD_OF_VIEW_H/2.):
					# cell is within visible range and distance
					cell = self.grid.get_cell_index((x, y))
					# TODO: on an unknown map, this won't work. cast perception rays cell-wise instead.
					if not self.grid.is_out_of_bounds(cell):
						# cell can be seen
						yield cell

	def iter_frontiers(self):
		'''yields all frontier cells. these are discovered, open cells with
		   undiscovered neighbors'''
		w, h = self.grid.get_size()
		for x in xrange(0, w):
			for y in xrange(0, h):
				if self.grid.get_cell_status((x, y)) == CELL_OPEN:
					if (self.grid.get_cell_status((x-1, y)) == CELL_UNKNOWN or
							self.grid.get_cell_status((x+1, y)) == CELL_UNKNOWN or
							self.grid.get_cell_status((x, y+1)) == CELL_UNKNOWN or
							self.grid.get_cell_status((x, y-1)) == CELL_UNKNOWN):
						yield (x, y)

	def get_vision_range(self):
		'''returns the planar distance of the furthest visible voxel to the
		   robot'''
		return self.position[2] * math.tan(self.robot_config.FIELD_OF_VIEW_V/2. - self.robot_config.CAMERA_ROTATION)

	def get_vision_width(self):
		'''returns width of top (far) horizontal vision edge depending on
		   current flight level assuming flat ground'''
		c = self.get_vision_range()
		return c * math.sin(self.robot_config.FIELD_OF_VIEW_H/2.) * 2.

	def get_vision_center_distance(self):
		'''returns the planar distance between robot position and vision center
		   assuming flat ground'''
		return math.tan(-self.robot_config.CAMERA_ROTATION) * self.position[2]

	def get_vision_center(self):
		'''returns the robot-relative vision center coordinates'''
		length = self.get_vision_center_distance()
		return math.cos(self.rotation) * length, math.sin(self.rotation) * length

	def point_vision_at(self, position):
		'''returns how the robot should be moved so that the vision center
		   points at a given position'''
		# TODO: this is not optimal. minimize max(rotation_time, translation_time) instead.
		tx, ty = position			# target
		rx, ry, _ = self.position
		# vector from target position to current position
		dx, dy = rx-tx, ry-ty
		# normalize to length of vision offset
		length = math.sqrt(dx**2 + dy**2)
		off_length = self.get_vision_center_distance()
		dx *= off_length / length
		dy *= off_length / length
		# rotate from current position facing to target
		rotation = math.atan2(ty - ry, tx - rx)
		return (tx+dx, ty+dy), rotation
