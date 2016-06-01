#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

verbose = False

argv = sys.argv[1:]
if '--verbose' in argv:
	argv = [item for item in argv if item != '--verbose']
	verbose = True

all_ok = True

for filename in argv:
	if verbose:
		print filename
	position_history = 0
	stats = 0
	objects = 0
	progress_ticks = 0
	with open(filename, 'r') as f:
		for line in f:
			if '+++ position history +++' in line:
				position_history += 1
			if '+++ STATS +++' in line:
				stats += 1
			if 'Objects found after:' in line:
				objects = int((len(line.split()) - 3) / 2.)
			if '+++ following progress ticks +++' in line:
				progress_ticks += 1

	if verbose:
		print 'position history', '✓' if position_history else '×'
		print 'stats', '✓' if stats else '×'
		print 'objects found:', objects, '✓' if objects == 3 else '×'
		print 'progress_ticks', '✓' if progress_ticks else '×'
	else:
		if (not position_history) or (not stats) or (not progress_ticks) or objects != 3:
			print filename, 'corrupted'
			all_ok = False

if all_ok:
	print 'checked', len(argv), 'files, all of which are nice.'
