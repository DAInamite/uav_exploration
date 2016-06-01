import math
from util import *
from abstract_decidor import AbstractDecidor

class Decidor(AbstractDecidor):
	def __init__(self, drawables_object):
		super(Decidor, self).__init__(drawables_object)

	def utility(self, cell):
		'''returns utility to visit given cell based on distance to vision
		   center and rotation delta'''
		# distance and rotation difference
		rx, ry, _ = self.position
		cell_pos = self.grid.get_cell_center(cell)
		(tx, ty), tr = self.point_vision_at(cell_pos)	# target
		distance = math.sqrt((rx-tx)**2 + (ry-ty)**2)
		rotation = get_rotation_diff(self.rotation, tr)
		# times
		translation_time = distance / self.robot_config.TRANSLATION_SPEED
		rotation_time = abs(rotation) / self.robot_config.ROTATION_SPEED #* 10
		travel_time = max(translation_time, rotation_time)
		return -travel_time

	def get_best_cell(self):
		best_cell = max(self.frontiers, key=lambda v: self.utility(v))
		return best_cell

	def decide(self, grid, position, rotation):
		super(Decidor, self).decide(grid, position, rotation)
		self.frontiers = list(self.iter_frontiers())
		self.drawables.frontiers = set(self.frontiers)
		if len(self.frontiers) == 0:
			return
		cell = self.get_best_cell()
		return self.point_vision_at(self.grid.get_cell_center(cell))
