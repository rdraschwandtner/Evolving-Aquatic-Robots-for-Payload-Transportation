"""
	Conduct an evolutionary test to see how well we can evolve a signal to fit a given input.
"""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
import random

from MNO import MNO

# Input signal to evolve towards.
truth = [math.cos(i) for i in xrange(0,100)]

def evaluate_individual(individual):
	""" Evaluate the fitness of an individual. """

	signal = [individual.step() for i in xrange(100)]

	return sum([abs(t-s) for t,s in zip(truth,signal)])

def evaluate_population(population):
    """ Evaluate the given population based on the fitness metric chosen.

    Args:
       population: population of individual muscle groups to evaluate
       fitness_metric: which fitness function to use
    """

    return [evaluate_individual(x) for x in population]

def tournament_selection(individuals):
    """ Select the best individual from those provided.

    Args:
        individuals: 2d list of tuple (index,fitness)
    Returns:
        index of the highest performing individual
    """
    return (min(individuals, key=lambda x: x[1]))[0] 

def next_generation(population,fitnesses):
    """ Create the next generation.

    Args:
        population: the population to evaluate
        fitnesses: the fitnesses of the individuals in the population
    """

    new_population = []

    # Keep the best individual
    new_population.append(population[fitnesses.index(min(fitnesses))].copy())

    for i in xrange(len(population)-1):
        sample = [[x,fitnesses[x]] for x in random.sample(xrange(len(fitnesses)), 3)]
        best_ind = tournament_selection(sample)
        new_population.append(population[best_ind].copy())
        new_population[i+1].mutate()
    return new_population 

def write_best_individual_signal(gen,fit,mno,run_num,output_path):
    """ Write out the best performing individual from a generation.

    Args:
        gen: which generation
        fit: fitness reached
        mno: matsuoka neural oscillator
        run_num: run number
        output_path: path to write file to
    """
    with open(output_path+str(run_num)+"_best_individuals.dat","a") as f:
		f.write(str(gen)+","+str(fit)+",")
		for i in xrange(399):
			f.write(str(individual.step())+",")
		f.write(str(individual.step()+"\n"))

def write_best_individual(gen,fit,mno,run_num,output_path):
    """ Write out the best performing individual from a generation.

    Args:
        gen: which generation
        fit: fitness reached
        mno: matsuoka neural oscillator
        run_num: run number
        output_path: path to write file to
    """
    with open(output_path+str(run_num)+"_best_individuals.dat","w") as f:
        f.write(str(gen)+","+str(fit)+"\n")
        f.write(str(mno))

def write_population_fitnesses(gen,fitnesses,run_num,output_path):
    """ Write out the population fitnesses for the generation.

    Args:
        gen: which generation
        fitnesses: list of fitnesses for the population
        run_num: run number
        output_path: path to write file to
    """
    with open(output_path+str(run_num)+"_fitnesses.dat","a") as f:
        f.write("Generation,Individual,Fitness\n")
        for i,fit in enumerate(fitnesses):
            f.write(str(gen)+","+str(i)+","+str(fit)+"\n")

def main():
    parser = argparse.ArgumentParser(description="Conduct an evolutionary test run.")
    args = parser.parse_args()

    mno = MNO(a=2.5,b=2.5,tau=0.25,T=0.5,dt=.1)

    fig = plt.figure(1)
    fig.suptitle("Final Evolved Signal versus Base Signal")
    ax1 = fig.add_subplot(111)

    signal = [mno.step(c=1.5) for i in xrange(1000)]
    ax1.plot([i for i in range(1000)], signal, color="r")

    plt.axis([0.,1000.,-1.,1.])
    plt.ylabel("Activation Level")
    plt.xlabel("Input Value")
    
    plt.show()


if __name__ == "__main__":
    main()