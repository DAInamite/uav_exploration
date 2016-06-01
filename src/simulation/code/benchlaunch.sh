#!/usr/bin/env bash

pypy benchmark.py --script simulation-concentric.py --threads 14 concentric
pypy benchmark.py --script simulation-frontier_util.py --threads 14 frontier_util
pypy benchmark.py --script simulation-random.py --threads 14 random
pypy benchmark.py --script simulation-sampling.py --threads 14 sampling
pypy benchmark.py --script simulation-srt.py --threads 14 srt
