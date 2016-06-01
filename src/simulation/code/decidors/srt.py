import math, random
from util import *
from abstract_decidor import AbstractDecidor

MIN_DISTANCE = 1.
MAX_CANDIDATES = 250

# enums
EXPLORING = 1
BACKTRACKING = 2
SCANNING = 3

CELL_UNKNOWN = -1	# no information yet
CELL_OPEN = 0		# open space
CELL_OBSTACLE = 1	# blocked space

class Config(object):
	def __init__(self, parent, position, safe_cells=None):
		self.position = position
		self.safe_cells = set(safe_cells)
		self.parent = parent
		self.nexts = []
		if parent:
			parent.nexts.append(self)
		
class Decidor(AbstractDecidor):
	def __init__(self, drawables_object):
		super(Decidor, self).__init__(drawables_object)
		self.current_config = None
		self.configs = []
		self.status = SCANNING
		self.reset_scan_rotation()

	def reset_scan_rotation(self):
		self.scan_rotation = math.pi * 2 - (self.robot_config.FIELD_OF_VIEW_H / 3. * 2.)

	def is_cell_in_configs(self, cell, except_position=None):
		for config in self.configs:
			if config.position != except_position and cell in config.safe_cells:
				return True
		return False

	def iter_safe_area(self):
		'''yields all cells around the robot which are within visibility range
		   and are already discovered as open area'''
		rx, ry, rz = self.position
		alpha = self.robot_config.FIELD_OF_VIEW_V/2. - self.robot_config.CAMERA_ROTATION
		max_dist = math.tan(alpha) * rz
		for x in frange(rx - max_dist, rx + max_dist, self.grid.cell_size):
			for y in frange(ry - max_dist, ry + max_dist, self.grid.cell_size):
				dist = math.sqrt((x-rx)**2 + (y-ry)**2)
				if dist < max_dist:
					# cell is within visible range and distance
					cell = self.grid.get_cell_index((x, y))
					if self.grid.get_cell_status(cell) == CELL_OPEN:
						yield cell

	def decide(self, grid, position, rotation):
		super(Decidor, self).decide(grid, position, rotation)
		safe_cells = None
		rx, ry, rz = position
		if self.status == SCANNING:
			# first step: rotate to see enclosing area
			r = min(math.pi * 2 / 3, self.scan_rotation) # rotate in thirds
			self.scan_rotation -= r
			if self.scan_rotation == 0:
				# last call, prepare for next phase
				self.reset_scan_rotation()
				self.status = EXPLORING
			return (rx, ry), rotation + r
		if self.status == EXPLORING:
			# second step: store current config
			safe_cells = list(self.iter_safe_area()) # list() for random.choice
			self.current_config = Config(self.current_config, position, safe_cells)
			self.configs.append(self.current_config)
			self.drawables.safe_area = set(safe_cells)
		# plan next step
		if safe_cells is None:
			safe_cells = list(self.current_config.safe_cells)
		for i in xrange(MAX_CANDIDATES):
			cand_cell = random.choice(safe_cells)
			# no overlapping
			if self.is_cell_in_configs(cand_cell, position):
				continue
			# not too close
			cx, cy = self.grid.get_cell_center(cand_cell)
			#(candx, candy, candz), candrot = self.world.robot.point_vision_at((cx, cy))
			if math.sqrt((rx-cx)**2 + (ry-cy)**2) < MIN_DISTANCE:
				continue
			# got one!
			self.status = SCANNING
			return (cx, cy), None
		# local safe area exhausted. need to backtrack
		self.current_config = self.current_config.parent
		if self.current_config == None:
			return
		self.status = BACKTRACKING
		px, py, _ = self.current_config.position
		return (px, py), None
