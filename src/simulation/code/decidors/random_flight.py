import random, math
from abstract_decidor import AbstractDecidor

# ENUMS
CELL_UNKNOWN = -1	# no information yet
CELL_OPEN = 0		# open space
CELL_OBSTACLE = 1	# blocked space

class Decidor(AbstractDecidor):
	def __init__(self, drawables_object):
		super(Decidor, self).__init__(drawables_object)

	def decide(self, grid, position, rotation):
		super(Decidor, self).decide(grid, position, rotation)
		if not any(self.iter_frontiers()):
			return
		w, h = self.grid.get_size_metric()
		rx, ry, _ = self.position
		r = random.random() * math.pi * 2
		if ((r > math.pi/2 and r < math.pi/2*3 and rx <= 0) or
				((r < math.pi/2 or r > math.pi/2*3) and rx >= w) or
				(r > math.pi and ry <= 0) or
				(r < math.pi and ry >= h)):
			r += math.pi
		dx, dy = math.cos(r), math.sin(r)
		# how to scale to get rx+dx = 0, rx+dx = w, ry+dy = 0, ry+dy = h
		scales = [-rx/dx, -rx/dx + w/dx, -ry/dy, -ry/dy + h/dy]
		# filter out non-positive values
		scales = [a for a in scales if a > 0]
		# get scale that yields the nearest edge
		scale = min(scales)

		x, y = rx + scale*dx, ry + scale*dy
		return (x, y), math.atan2(y-ry, x-rx)
