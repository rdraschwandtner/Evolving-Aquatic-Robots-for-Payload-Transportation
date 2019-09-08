"""
    Evolve a worm robot that contains an ANN Controller.  
"""

import argparse
import sys, os, random, time
import itertools
import math
import multiprocessing as mpc
import GlobalVarWorkaround
import json

from worm_simulation import Simulation

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

def store(fname, fitness, generation, index, choice):
    text = json.dumps((generation, index,fitness,choice)) + '\n'
    with open(fname, 'a') as json_file:
        json_file.write(text)

def evaluate_individual(individual):
    """ Wrapper to call Simulation which will evaluate an individual.  

    Args:
        individual: arguments to pass to the simulation

    Returns:
        fitness of an individual
    """

    simulation = Simulation(log_frames=GlobalVarWorkaround.args.log_frames, run_num=GlobalVarWorkaround.args.run_num, eval_time=GlobalVarWorkaround.args.eval_time, dt=.02, n=4, num_joints=GlobalVarWorkaround.args.num_joints, aquatic=GlobalVarWorkaround.args.aquatic, periodic=(not GlobalVarWorkaround.args.no_periodic))
    ret = simulation.evaluate_individual(individual)
    #print ret
    return ret

def evolutionary_run(**kwargs):
    """ Conduct an evolutionary run using the worm.  
    
    Args:
        gens: generations of evolution
        pop_size: population size
        mut_prob: mutation probability
    """
    global current_network, run_num, output_path, population_size, simulation

    params = NEAT.Parameters()  
    params.CompatTreshold = 5.0
    params.CompatTresholdModifier = 0.3
    params.YoungAgeTreshold = 15
    params.SpeciesMaxStagnation = 1000
    params.OldAgeTreshold = 35
    params.MinSpecies = 1 
    params.MaxSpecies = 25
    params.RouletteWheelSelection = False
    params.CrossoverRate = 0.4
    params.RecurrentProb = 0.25
    params.OverallMutationRate = 0.33
    params.MutateWeightsProb = 0.90
    params.WeightMutationMaxPower = 1.0
    params.WeightReplacementMaxPower = 5.0
    params.MutateWeightsSevereProb = 0.5
    params.WeightMutationRate = 0.75
    params.MaxWeight = 20

    params.MutateAddNeuronProb = 0.04
    params.MutateAddLinkProb = 0.1
    params.MutateRemSimpleNeuronProb = 0.04
    params.MutateRemLinkProb = 0.1

    # Phased Searching
    params.PhasedSearching = True;
    params.SimplifyingPhaseMPCTreshold = 20;
    params.SimplifyingPhaseStagnationTreshold = 20;
    params.ComplexityFloorGenerations = 20;

    params.PopulationSize = kwargs['pop_size'] 

    # Calculate the number of inputs (without bias and periodic signal) and outputs based on the number of joints.
    num_outputs = kwargs['num_joints']
    num_inputs = kwargs['num_joints'] + 1 + 3
   
    # Initialize the populations
    genome = NEAT.Genome(0, num_inputs+1, 0, num_outputs, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    # If not including a periodic input.
    if GlobalVarWorkaround.args.no_periodic:
        genome = NEAT.Genome(0, num_inputs, 0, num_outputs, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    population = NEAT.Population(genome, params, True, 1.0)

    mnlog.write_population_statistics_headers(output_path+str(run_num)+"_fitnesses.dat",optional_additions="Ball_Pos")

    # Setup multiprocessing
    cores = mpc.cpu_count()
    pool = mpc.Pool(initializer=initProcess, initargs=(GlobalVarWorkaround.args,), processes=cores)

    for gen in xrange(kwargs['gens']):
        genome_list = NEAT.GetGenomeList(population)

        zippedret = pool.map(evaluate_individual,genome_list)
        #print str(zippedret)
        fitnesses = [fit for fit,choice in zippedret]
        choices = [choice for fit,choice in zippedret]
        # fitnesses = []
        # for g_l in genome_list:
        #    fitnesses.append(evaluate_individual(g_l))
        for g,f in zip(genome_list,fitnesses):
            g.SetFitness(f)
        print("Generation "+str(gen)+"\t: "+str(max(fitnesses)))

        # Write the best performing individual to a file.
        mnlog.write_best_individual(output_path+"best_individuals/Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen)+".dat", 
                genome_list[fitnesses.index(max(fitnesses))])
        store(output_path+str(run_num)+'_choices.dat', max(fitnesses), gen, fitnesses.index(max(fitnesses)), choices[fitnesses.index(max(fitnesses))])

        # Log the progress of the entire population.
        mnlog.write_population_statistics(output_path+str(run_num)+"_fitnesses.dat", genome_list, fitnesses, gen)

        # Log the final population for later evaluation.
        if gen == kwargs['gens'] - 1:
            population.Save(output_path+"run_"+str(run_num)+"_population_generation_"+str(gen)+".dat")

        # Create the next generation
        population.Epoch()

######################################################################
# windows global variable workaround
# http://stackoverflow.com/questions/1675766/how-to-combine-pool-map-with-array-shared-memory-in-python-multiprocessing
def initProcess(paramargs):
   GlobalVarWorkaround.args = paramargs

if __name__ == '__main__':
	# Process inputs.
	parser = argparse.ArgumentParser()
	parser.add_argument("--validator", action="store_true", help="Validate current results.")
	parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
	parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
	parser.add_argument("--mut_prob", type=float, default=0.05, help="Mutation probability for evolution.")
	parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
	parser.add_argument("--run_num", type=int, default=0, help="Run Number")
	parser.add_argument("--output_path", type=str, default="./", help="Output path")
	parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
	parser.add_argument("--val_num",type=int,default=-1,help="Number to identify the validator run.")
	parser.add_argument("--debug_runtime",action="store_true",help="Evaluate the run time of a simulation.")
	parser.add_argument("--no_periodic",action="store_true",help="Whether we're including a periodic signal or not.")
	parser.add_argument("--num_joints", type=int, default=9,help="Number of joints in the worm.")
	parser.add_argument("--aquatic", action="store_true", help="Whether to simulate in the aquatic environment.")
	GlobalVarWorkaround.args = parser.parse_args()

	running = True
	eval_time = GlobalVarWorkaround.args.eval_time 

	output_path = GlobalVarWorkaround.args.output_path
	run_num = GlobalVarWorkaround.args.run_num

	# Seed only the evolutionary runs.
	random.seed(run_num)

	if GlobalVarWorkaround.args.debug_runtime:
		# Initialize the Simulation component
		simulation = Simulation(log_frames=GlobalVarWorkaround.args.log_frames, run_num=GlobalVarWorkaround.args.run_num, eval_time=GlobalVarWorkaround.args.eval_time, dt=.02, n=4, num_joints=GlobalVarWorkaround.args.num_joints, aquatic=GlobalVarWorkaround.args.aquatic, periodic=(not GlobalVarWorkaround.args.no_periodic))

		simulation.debug_validator()
	elif GlobalVarWorkaround.args.validator:
		assert (GlobalVarWorkaround.args.log_frames == True), 'This implementation assumes --log_frames with --validator'

		# Ensure that the file exists before trying to open to avoid hanging.
		NEAT_file = output_path+"best_individuals/Evo_NEAT_run_"+str(GlobalVarWorkaround.args.run_num)+"_best_gen_"+str(GlobalVarWorkaround.args.gens)+".dat"
		if not os.path.isfile(NEAT_file):
			print("NEAT Genome file doesn't exist! "+NEAT_file)
			exit()

		for i in [-1,1]:
			# Initialize the Simulation component
			simulation = Simulation(log_frames=GlobalVarWorkaround.args.log_frames, run_num=GlobalVarWorkaround.args.run_num, eval_time=GlobalVarWorkaround.args.eval_time, dt=.02, n=4, num_joints=GlobalVarWorkaround.args.num_joints, aquatic=GlobalVarWorkaround.args.aquatic, periodic=(not GlobalVarWorkaround.args.no_periodic),validator_condition=i)
			simulation.validator(NEAT_file)

			# Rename the log file so we don't overwrite it.
			file_num = "n1" if i == -1 else "p1"
			os.rename(str(GlobalVarWorkaround.args.run_num)+"_logged_output.dat", str(GlobalVarWorkaround.args.run_num)+"_logged_output_"+str(file_num)+".dat")

	else:
		# Ensure that the necessary folders are created for recording data.
		if not os.path.exists(output_path+"best_individuals"):
			os.makedirs(output_path+"best_individuals")
		evolutionary_run(gens=GlobalVarWorkaround.args.gens,pop_size=GlobalVarWorkaround.args.pop_size,mut_prob=GlobalVarWorkaround.args.mut_prob, num_joints=GlobalVarWorkaround.args.num_joints)