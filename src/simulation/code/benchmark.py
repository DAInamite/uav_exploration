#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, subprocess, threading, time

PROCESSES = 8
SCENARIOS = 100
CELLS = 120000
SCRIPT_NAME = 'simulation.py'
BENCH_NAME = 'test'

class Process(threading.Thread):
	def __init__(self, scenario_id, scenario):
		threading.Thread.__init__(self) 
		self.scenario_id = scenario_id
		self.scenario = scenario
		self.visited_cells = 0
		self.result = None

	def run(self):
		rx, ry, rot, o1x, o1y, o2x, o2y, o3x, o3y = self.scenario
		proc = subprocess.Popen(['pypy', SCRIPT_NAME, rx, ry, rot, o1x, o1y, o2x, o2y, o3x, o3y],
			stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		for line in proc.stderr:
			if 'cells visited' in line:
				self.visited_cells = int(line.split()[2])
		self.result = proc.stdout.read()



# read scenario
scen = []
with open('scenario-pool.txt', 'r') as f:
	for _ in xrange(SCENARIOS):
		rx, ry, rot = f.readline().split()
		o1x, o1y, o2x, o2y, o3x, o3y = f.readline().split()
		scen.append((rx, ry, rot, o1x, o1y, o2x, o2y, o3x, o3y))

#print scen[37]
#exit(0)

scenarios_to_do = range(SCENARIOS)

args = list(sys.argv)

if '--help' in args:
	print 'USAGE: %s <benchmark name> [--threads n] [<start scenario>]' % args[0]
	exit(1)
if '--threads' in args:
	i = args.index('--threads')
	PROCESSES = int(args[i+1])
	del args[i:i+2]
if '--script' in args:
	i = args.index('--script')
	SCRIPT_NAME = args[i+1]
	del args[i:i+2]
if len(args) > 1:
	BENCH_NAME = args[1]
if len(args) > 2:
	scenarios_to_do = map(int, args[2:])

proc = [None for _ in xrange(PROCESSES)]
next_scenario = 0

while True:

	sys.stdout.write('\r')

	for i in xrange(len(proc)):
		p = proc[i]

		if (not p is None) and (not p.result is None):
			# p is a process and has a result
			with open('benchmark/%s_%00i.txt' % (BENCH_NAME, p.scenario_id), 'w') as benchfile:
				benchfile.write(p.result)
			proc[i] = None
			p = None

		if p is None and next_scenario < len(scenarios_to_do):
			s_id = scenarios_to_do[next_scenario]
			proc[i] = Process(s_id, scen[s_id])
			p = proc[i]
			p.start()
			next_scenario += 1

		color = '\033[94m' if i % 2 == 0 else '\033[95m'
		sys.stdout.write(color)
		if p is None:
			sys.stdout.write(' finished ')
		else:
			sys.stdout.write('%4i: %3i%%' % (p.scenario_id, 100. * p.visited_cells / CELLS))
	sys.stdout.write('\033[0m')
	sys.stdout.flush()

	# all processes are None?
	if not any(map(bool, proc)):
		# done!
		sys.stdout.write('\nDone.\n')
		sys.exit(0)

	time.sleep(0.2)
