'''Decidor based on heuristically sampling potential robot configurations
   (views) to test them with a utility function.'''

import math, random, time
from util import *
from abstract_decidor import AbstractDecidor

GENERATION_SIZE = 20
GENERATIONS = 15
FRONTIER_STEP = 1.	# distance to randomly jump from a frontier cell for sampling
CHUNK_SIZE = 1.		# block size from which to pick one frontier each

# ENUMS
CELL_UNKNOWN = -1	# no information yet
CELL_OPEN = 0		# open space
CELL_OBSTACLE = 1	# blocked space

class Decidor(AbstractDecidor):
	def __init__(self, drawables_object):
		super(Decidor, self).__init__(drawables_object)

	def get_config_similarity(self, config1, config2):
		cx1, cy1, cr1 = config1
		cx2, cy2, cr2 = config2
		d = math.sqrt((cx1-cx2)**2 + (cy1-cy2)**2)
		r = abs(get_rotation_diff(cr1, cr2))
		return math.e**(-d/10. - r/math.pi)

	def get_utility(self, config):
		'''returns utility value of a potential robot position and rotation.'''
		px, py, rotation = config
		rx, ry, rz = self.position
		# count possible newly discovered cells
		new_cells = 0
		for cell in self.get_visible_cells((px, py, rz), rotation):
			if self.grid.get_cell_status(cell) == CELL_UNKNOWN:
				new_cells += 1
		# distance and rotation difference
		distance = math.sqrt((px-rx)**2 + (py-ry)**2)
		rotation = abs(get_rotation_diff(rotation, self.rotation))
		# times
		translation_time = distance / self.robot_config.TRANSLATION_SPEED
		rotation_time = abs(rotation) / self.robot_config.ROTATION_SPEED
		travel_time = max(translation_time, rotation_time)
		if new_cells == 0 or travel_time == 0:
			return 0
		else:
			return new_cells / travel_time

	def get_initial_samples(self):
		'''creates a first generation of robot configuration samples.'''
		# basic "current config" sample
		rx, ry, rz = self.position
		sample = (rx, ry, self.rotation)
		samples = [(sample, self.get_utility(sample))]
		# basic "fly forward" sample
		max_dist = self.get_vision_range()
		d = max_dist * .9
		sx, sy = rx + math.cos(self.rotation)*d, ry + math.sin(self.rotation)*d
		scell = self.grid.get_cell_index((sx, sy))
		if self.grid.get_cell_status(scell) == CELL_OPEN:
			sample = (sx, sy, self.rotation)
			samples.append((sample, self.get_utility(sample)))
		# random frontier based samples
		block_covered = set()
		for frontier in self.frontiers:
			fx, fy = self.grid.get_cell_center(frontier)
			bx, by = int(fx / CHUNK_SIZE), int(fy / CHUNK_SIZE)
			block = bx, by
			if block not in block_covered:
				block_covered.add(block)
				for _ in xrange(10):		# try
					# jump distance
					d = FRONTIER_STEP * random.random() + random.gauss(0, FRONTIER_STEP / 2.)
					# jump direction
					r = random.random() * math.pi * 2
					sx, sy = fx + math.cos(r)*d, fy + math.sin(r)*d
					scell = self.grid.get_cell_index((sx, sy))
					if self.grid.get_cell_status(scell) == CELL_OPEN:
						sample = (sx, sy, -r)	# look at frontier
						samples.append((sample, self.get_utility(sample)))
						break
		print 'samples:', len(samples)
		return samples

	def next_generation(self, samples):
		new_samples = []
		for (x, y, r), _ in samples:
			# mutate position
			while True:
				alpha = random.random() * math.pi * 2
				delta = random.gauss(0, 1)
				new_x = x + math.cos(alpha)*delta
				new_y = y + math.sin(alpha)*delta
				ncell = self.grid.get_cell_index((new_x, new_y))
				if self.grid.get_cell_status(ncell) == CELL_OPEN:
					break
			# mutate rotation
			new_r = r + random.gauss(0, math.pi/3)
			# store sample
			sample = (new_x, new_y, new_r)
			new_samples.append((sample, self.get_utility(sample)))

			# cross over
			(_, _, other_r), _ = random.choice(samples)
			# store sample
			sample = (x, y, other_r)
			new_samples.append((sample, self.get_utility(sample)))
			self.add_samples_to_drawables(new_samples)
		samples += new_samples

		# filter out similar samples
		# remove_indexes = set()
		# for i in xrange(len(samples)):
		# 	if i in remove_indexes:
		# 		continue
		# 	config1, util1 = samples[i]
		# 	for j in xrange(i+1, len(samples)):
		# 		if j in remove_indexes:
		# 			continue
		# 		config2, util2 = samples[j]
		# 		if random.random() < self.get_config_similarity(config1, config2):
		# 			#print 'remove! sim:', self.get_config_similarity(config1, config2), 'samples:', samples[i], samples[j]
		# 			# remove worse one
		# 			if util1 < util2:
		# 				remove_indexes.add(i)
		# 			else:
		# 				remove_indexes.add(j)
		# samples = [sample for i, sample in enumerate(samples) if i not in remove_indexes]

		samples = sorted(samples, key=lambda (config, utility): -utility)
		return samples[:GENERATION_SIZE]

	def decide(self, grid, position, rotation):
		'''depending on the current grid setup, position and rotation,
		   makes the robot navigate to the next best view.'''
		super(Decidor, self).decide(grid, position, rotation)

		self.frontiers = list(self.iter_frontiers())
		self.drawables.frontiers = set(self.frontiers)
		if len(self.frontiers) == 0:
			# no frontiers? no exploration
			return
		start_time = time.time()
		samples = self.get_initial_samples()
		del self.drawables.samples[:]		# clear
		self.add_samples_to_drawables(samples)
		for x in xrange(GENERATIONS):
			samples = self.next_generation(samples)
		print 'time needed: %.2f' % (time.time() - start_time), 'configs tested:', len(self.drawables.samples)
		best_config = samples[0]
		(x, y, r), _ = best_config
		return (x, y), r

	def add_samples_to_drawables(self, samples):
		self.drawables.samples += [(x, y) for (x, y, r), util in samples]
