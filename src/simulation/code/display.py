#!/usr/bin/env python

import sys, math
import pygame
from pygame.locals import *

import simulation

SCREEN_WIDTH = 4 * 240  # 170
SCREEN_HEIGHT = 3 * 240


class Drawables(object):
	'''class to hold drawable data to be provided by decision/simulation code'''
	def __init__(self):
		self.visible_cells = [] # current perception
		self.safe_area = [] # current SRT scan radius
		self.frontiers = [] # open cells with unknown neighbors
		self.samples = [] # recent NBV candidates
		self.debug_coords = None

class Display(object):

	def __init__(self):
		'''create window, required objects and surfaces'''
		self.drawables = Drawables()
		self.simulation = simulation.Simulation(self.drawables)
		self.grid_w, self.grid_h = self.simulation.world.grid.get_size()
		self.hscale = SCREEN_WIDTH / self.simulation.world.WORLD_WIDTH		# horizontal scaling factor
		self.vscale = SCREEN_HEIGHT / self.simulation.world.WORLD_HEIGHT	# vertical scaling factor

		pygame.init()
		self.clock = pygame.time.Clock()

		self.surf_window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
		self.surf_window.set_alpha(None)
		pygame.display.set_caption('Autonomous exloration simulation')

		self.font_sans14 = pygame.font.Font('freesansbold.ttf', 14)
		self.col_black = pygame.Color(0, 0, 0)
		self.col_white = pygame.Color(248, 248, 242)
		self.col_grid = pygame.Color(33, 34, 28)
		self.col_unknown = pygame.Color(39, 40, 34)
		self.col_visited = pygame.Color(157, 116, 230) #(174, 129, 255)
		self.col_safe = pygame.Color(193, 143, 255)
		self.col_robot = pygame.Color(166, 226, 46)
		self.col_frontier = pygame.Color(230, 219, 116)
		self.col_object = pygame.Color(255, 50, 50)
		self.col_debug = pygame.Color(249, 38, 114)
		self.col_samples = pygame.Color(3, 18, 31)
		#self.col_something = pygame.Color(253, 151, 31)
		self.surf_rawgrid = pygame.Surface((self.grid_w, self.grid_h))

	def run(self):
		printed_stats = False
		show_fps = True
		frame = 0
		while True:
			if self.simulation.world.robot.busy:
				self.simulation.tick()
			elif not printed_stats:
				print self.simulation.stats
				printed_stats = True

			self.surf_window.fill(self.col_black)

			self.draw_grid(self.simulation.world)
			self.draw_path()
			self.draw_samples()
			self.draw_robot()
			self.draw_debug_coords()

			#frame += 1
			#pygame.image.save(self.surf_window, 'imgout/frame_%i.jpg' % frame)

			if show_fps:
				surf_info = self.font_sans14.render('FPS: %.1f' % self.clock.get_fps(), True, self.col_debug)
				self.surf_window.blit(surf_info, (1, -1))
				surf_info = self.font_sans14.render('Time: %.1f s' % self.simulation.stats.time, True, self.col_debug)
				self.surf_window.blit(surf_info, (1, 12))

			# all teh infos
			#surf_frontiers = self.font_sans14.render('frontier voxels: %i' % len(self.simulation.world.grid.frontiers), True, self.col_debug)
			#self.surf_window.blit(surf_frontiers, (1, 13))

			pygame.display.update()
			self.clock.tick(self.simulation.world.robot.CAM_FPS)

			for event in pygame.event.get():
				if event.type == QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == KEYDOWN:
					if event.key == K_ESCAPE:
						pygame.event.post(pygame.event.Event(QUIT))
					elif event.key == K_v:
						visited_count = 0
						for y in xrange(self.grid_h):
							for x in xrange(self.grid_w):
								if self.simulation.world.grid.get_cell_status((x, y)) == simulation.CELL_OPEN:
									visited_count += 1
						print 'visited:', (100. * visited_count / (self.grid_w * self.grid_h)), '%'
					elif event.key == K_f:
						show_fps = not show_fps
					elif event.key == K_j:
						for _ in xrange(1000):
							self.simulation.tick()


	def get_grid_line_width(self, index):
		if (index*self.simulation.world.grid.cell_size) % 10 == 0:
			return 4
		elif (index*self.simulation.world.grid.cell_size) % 1 == 0:
			return 2
		else:
			return 1

	def draw_grid(self, world):
		pxarray = pygame.PixelArray(self.surf_rawgrid)
		for y in xrange(self.grid_h):
			for x in xrange(self.grid_w):
				if world.grid.get_cell_status((x, y)) == simulation.CELL_OPEN:
					# color priorities:
					if (x, y) in world.objects:
						# hidden object
						color = self.col_object
					elif (x, y) in self.drawables.frontiers:
						color = self.col_frontier
					elif (x, y) in self.drawables.visible_cells:
						# visible cone
						color = self.col_white
					elif (x, y) in self.drawables.safe_area:
						color = self.col_safe
					else:
						# other visited cells
						color = self.col_visited
				else:
					color = self.col_unknown
				pxarray[x, y] = color
		del pxarray
		surf_grid = pygame.transform.scale(self.surf_rawgrid, (self.grid_w*5, self.grid_h*5))
		# cell seperator lines
		for y in xrange(self.grid_h+1):
			pygame.draw.line(surf_grid, self.col_grid, (0, y*5), (self.grid_w*5, y*5), self.get_grid_line_width(y))
		for x in xrange(self.grid_w+1):
			pygame.draw.line(surf_grid, self.col_grid, (x*5, 0), (x*5, self.grid_h*5), self.get_grid_line_width(x))
		surf_grid = pygame.transform.smoothscale(surf_grid, (SCREEN_WIDTH, SCREEN_HEIGHT))
		self.surf_window.blit(surf_grid, (0, 0))

	def draw_robot(self):
		'''draws the robot/copter circle'''
		x, y, _ = self.simulation.world.robot.position
		f = SCREEN_WIDTH / self.simulation.world.WORLD_WIDTH	# scaling factor
		x, y = int(x*self.hscale), int(y*self.vscale)
		r = int(self.simulation.world.robot.BOT_RADIUS / self.simulation.world.WORLD_WIDTH * SCREEN_WIDTH)
		pygame.draw.circle(self.surf_window, self.col_unknown, (x, y), r, 2)
		pygame.draw.circle(self.surf_window, self.col_robot, (x, y), r-2)

	def draw_path(self):
		'''draw waypoint history'''
		points = map(lambda ((x, y), r): (x*self.hscale, y*self.vscale), self.simulation.world.robot.position_history)
		if len(points) > 1:
			pygame.draw.aalines(self.surf_window, self.col_robot, False, points)

	def draw_samples(self):
		for x, y in map(lambda (x, y): (x*self.hscale, y*self.vscale), self.drawables.samples):
			#self.surf_window.set_at()
			pygame.draw.aaline(self.surf_window, self.col_samples, (x-.5, y-.5), (x+.5, y+.5))
			pygame.draw.aaline(self.surf_window, self.col_samples, (x-.5, y+.5), (x+.5, y-.5))

	def draw_debug_coords(self):
		if not self.drawables.debug_coords is None:
			x, y = self.drawables.debug_coords
			x *= self.hscale
			y *= self.vscale
			pygame.draw.line(self.surf_window, self.col_debug, (x-5, y-5), (x+5, y+5), 2)
			pygame.draw.line(self.surf_window, self.col_debug, (x-5, y+5), (x+5, y-5), 2)

if __name__ == '__main__':
	display = Display()
	display.run()
