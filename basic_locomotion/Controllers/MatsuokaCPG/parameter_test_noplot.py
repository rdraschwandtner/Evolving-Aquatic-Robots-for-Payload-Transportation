"""
	Conduct an evolutionary test to see how well we can evolve a signal to fit a given input.
"""

import argparse
import math
import numpy as np
import random

from MNO import MNO

def main():
    parser = argparse.ArgumentParser(description="Conduct an evolutionary test run.")
    args = parser.parse_args()

    mno = MNO(a=2.5,b=2.5,tau=0.25,T=0.5,dt=.1)
    signal = [mno.step(c=1.5) for i in xrange(1000)]

    print(','.join(map(str,signal)))


if __name__ == "__main__":
    main()