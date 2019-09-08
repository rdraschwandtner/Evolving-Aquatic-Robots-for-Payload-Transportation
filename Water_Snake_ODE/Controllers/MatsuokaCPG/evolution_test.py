"""
	Conduct an evolutionary test to see how well we can evolve a signal to fit a given input.
"""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
import random

from MNO import MNO

TS = 0.25

# Input signal to evolve towards.
truth = [2.*math.sin(i*TS)+math.sin(3.*i*TS) for i in xrange(0,400)]

def evaluate_individual(individual):
	""" Evaluate the fitness of an individual. """

	signal = [individual.step(4.0) for i in xrange(100)]

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
			f.write(str(individual.step(4.0))+",")
		f.write(str(individual.step(4.0)+"\n"))

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
    parser.add_argument("--pop_size", type=int, default=150, help="Population Size")
    parser.add_argument("--num_gens", type=int, default=100, help="Number of Generations")
    parser.add_argument("--debug", action="store_true", help="Debugging Output")
    parser.add_argument("--run_num", type=int, default=0, help="Run number for logging files.")
    parser.add_argument("--output_path", type=str, default="", help="Output path")
    args = parser.parse_args()

    population = [MNO(dt=.1) for x in xrange(args.pop_size)]

    if(args.debug):
    	print("Created the population.")

    # Conduct the evolutionary runs over generations.
    for gen in xrange(args.num_gens):
        fitnesses = evaluate_population(population)
       
        if(args.debug):
            print(gen,min(fitnesses))
 
        write_best_individual(gen,min(fitnesses),population[fitnesses.index(min(fitnesses))],args.run_num,args.output_path)

        write_population_fitnesses(gen,fitnesses,args.run_num,args.output_path)

        population = next_generation(population,fitnesses) 

    fig = plt.figure(1)
    fig.suptitle("Final Evolved Signal versus Base Signal")
    ax1 = fig.add_subplot(111)

    signal = [population[0].step(4.0) for i in xrange(400)]
    ax1.plot([i for i in range(400)], signal, color="r")
    ax1.plot([i for i in range(400)], truth, color="b")

    #plt.axis([0.,100.,-1.,1.])
    plt.ylabel("Activation Level")
    plt.xlabel("Input Value")
    
    plt.show()

if __name__ == "__main__":
    main()