"""
    Conduct a test to check and see if my idea for a distance sensor will work by sweeping a box back and forth and
    noting the collision with a sphere given a non-colliding rectangle.
"""

import sys, os, random, time
import itertools
import math
import multiprocessing as mpc

from test_simulation import Simulation

# Initialize the Simulation component
simulation = Simulation(log_frames=True, run_num=0, eval_time=10, dt=.02, n=4)

simulation.validator()