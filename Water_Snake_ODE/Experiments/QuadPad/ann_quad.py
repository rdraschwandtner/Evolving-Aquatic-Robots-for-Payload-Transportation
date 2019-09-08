"""
    Evolve a quadruped robot that contains an ANN controller which dictates joint angle and joint power.  
"""

import argparse
import sys, os, random, time
import itertools
import math
import multiprocessing as mpc

from quad_simulation import Simulation

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

class MorphGenomes(object):
    """ Container for methods relating to evolving the morphology component of population. """

    def __init__(self,pop_size):
        """ Initialize the population. """
        self.pop_size = pop_size
        self.genomes = {}

    def addIndividual(self,NEAT_id):
        """ Add an individual to the population keyed with the NEAT genome ID. """
        self.genomes[NEAT_id] = [float("{0:.6f}".format(0.01+(random.random()*0.99))),float("{0:.6f}".format(0.01+(random.random()*0.99)))]

    def getGenomes(self):
        """ Return the genomes in the population. """
        return self.genomes

    def logGenomes(self,file_prefix):
        """ Log the genomes to an output file. """
        pass

    def logGenome(self,NEAT_id,file_prefix,output_path):
        """ Log a genome to an output file. """
        with open(output_path+"/best_individuals/"+file_prefix+"_morph_genome.dat","w") as f:
            f.write(str(self.genomes[NEAT_id][0])+","+str(self.genomes[NEAT_id][1]))

    def NextGen(self):
        """ Prepare the genomes for the next generation. """
        self.parent_genomes = self.genomes
        self.genomes = {}

    def Crossover(self,NEAT_id,par_1,par_2):
        """ Crossover two individuals to create a new individual. """
        # Crossover occurred 
        if par_2 > 0:
            if random.random() < 0.5:
                self.genomes[NEAT_id] = [self.parent_genomes[par_1][0],self.parent_genomes[par_2][1]]
            else:
                self.genomes[NEAT_id] = [self.parent_genomes[par_2][0],self.parent_genomes[par_1][1]]
        else:
            self.genomes[NEAT_id] = [self.parent_genomes[par_1][0],self.parent_genomes[par_1][1]]

    def MutatePopulation(self):
        """ Mutate the population. """
        for ind in self.genomes:
            for i in range(len(self.genomes[ind])):
                if random.random() < 0.1:
                    self.genomes[ind][i] = float("{0:.6f}".format(self.genomes[ind][i]+random.gauss(0,0.05)))

                    # Check the bounds (0.1,0.99)
                    if self.genomes[ind][i] < 0.1:
                        self.genomes[ind][i] = 0.1
                    elif self.genomes[ind][i] > 0.99:
                        self.genomes[ind][i] = 0.99

def dump_ann_activations(out_path, activations):
    """ Write out the ANN activations to a file. """

    with open(out_path, "w") as f:
        for line in activations:
            f.write((','.join(str(x) for x in line))+"\n")

def dump_joint_feedback(out_path, jf):
    """ Write out the joint_feedback to a file. """

    with open(out_path, "w") as f:
        f.write("Time,Joint_ID,F1,T1,F2,T2\n")
        for second in jf:
            for joint in second:
                f.write((','.join(str(x) for x in joint))+"\n")            

def check_valid_fitnesses(fitnesses):
    """ Ensure that the fitnesses are valid.

    Args:
        fitnesses: list of fitness values to Validate

    Returns:
        validated list of fitnesses adjusted for invalid fitnesses.
    """
    # Check each of the three fitness components for nan.  Set all three to low number if one is so.
    # Prevents the hanging problem seen in earlier runs.
    for i in range(len(fitnesses)):
        if math.isnan(fitnesses[i][0]) or math.isnan(fitnesses[i][1]) or math.isnan(fitnesses[i][2]):
            fitnesses[i][0] = 0.0000001
            fitnesses[i][1] = 0.0000001
            fitnesses[i][2] = 0.0000001                
    return fitnesses

def set_sliding_window_fitnesses(genome_list,fitnesses,window_perc,fitness_metric):
    """ Set the fitnesses of the various genomes using a sliding window.

    Args:
        genome_list: list of genomes to set fitnesses of
        fitnesses: fitness values to use when setting fitnesses
        window_perc: what is the size of the sliding window
        fitness_metric: which index to use as fitness (efficiency or distance)

    Returns:
        genome_list with fitnesses set appropriately and the adjusted fitness list
    """
    # Process the fitnesses so that we enforce a sliding window of absolute distance travelled
    # while still evaluating based on efficiency.
    proc_fitnesses = zip(*fitnesses)
    max_distance = max(proc_fitnesses[1])
    distance_cutoff = window_perc*max_distance
    dist_fit = [f[1] for f in fitnesses]
    eff_fit = [f[2] for f in fitnesses]

    for i,(g,f) in enumerate(zip(genome_list,fitnesses)):
        if f[1] > distance_cutoff:
            g.SetFitness(f[fitness_metric])
        else:
            g.SetFitness(0.00001)
            fitnesses[i][2] = 0.000001
            dist_fit[i] = 0.000001
    return genome_list, fitnesses, dist_fit

def evaluate_individual(individual):
    """ Wrapper to call Simulation which will evaluate an individual.  

    Args:
        individual: arguments to pass to the simulation

    Returns:
        fitness of an individual
    """

    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)
    return simulation.evaluate_individual(individual)

def evolutionary_run(**kwargs):
    """ Conduct an evolutionary run using the snake and muscle model.  
    
    Args:
        gens: generations of evolution
        pop_size: population size
        mut_prob: mutation probability
    """
    global args,current_network,fitness_function,run_num,output_path,population_size,simulation

    params = NEAT.Parameters()  
    params.CompatTreshold = 5.0
    params.CompatTresholdModifier = 0.3
    params.YoungAgeTreshold = 15
    params.SpeciesMaxStagnation = 1000
    params.OldAgeTreshold = 35
    params.MinSpecies = 1 
    params.MaxSpecies = 25
    params.RouletteWheelSelection = False
    params.RecurrentProb = 0.25
    params.OverallMutationRate = 0.33
    params.MutateWeightsProb = 0.90
    params.WeightMutationMaxPower = 1.0
    params.WeightReplacementMaxPower = 5.0
    params.MutateWeightsSevereProb = 0.5
    params.WeightMutationRate = 0.75
    params.MaxWeight = 20
    params.MutateAddNeuronProb = 0.4
    params.MutateAddLinkProb = 0.4
    params.MutateRemLinkProb = 0.05

    params.PopulationSize = kwargs['pop_size'] 
   
    # Initialize the population
    if args.fixed_strength:
        genome = NEAT.Genome(0, 22, 0, 16, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
        # If not including a periodic input.
        if args.no_periodic:
            genome = NEAT.Genome(0, 21, 0, 16, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    else:
        genome = NEAT.Genome(0, 22, 0, 24, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
        # If not including a periodic input.
        if args.no_periodic:
            genome = NEAT.Genome(0, 21, 0, 24, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    population = NEAT.Population(genome, params, True, 1.0)
    genome_list = NEAT.GetGenomeList(population)

    morph_pop = MorphGenomes(kwargs['pop_size'])
    for ind in genome_list:
        morph_pop.addIndividual(ind.GetID())
    morph_genomes = morph_pop.getGenomes()

    # Zip the two genome components together for use in the parallel call.
    zip_args = [(ind,morph_genomes[ind.GetID()]) for ind in genome_list]

    mnlog.write_population_statistics_headers(output_path+str(run_num)+"_fitnesses.dat",optional_additions="Distance,Efficiency")

    # Setup multiprocessing
    cores = mpc.cpu_count()
    pool = mpc.Pool(processes=cores-2)

    for gen in xrange(kwargs['gens']):
        #fitnesses = []
        #for z in zip_args:
        #    fitnesses.append(evaluate_individual(z))
        fitnesses = pool.map(evaluate_individual,zip_args)

        # Validate the fitnesses.
        fitnesses = check_valid_fitnesses(fitnesses)

        # Set the fitnesses appropriately.
        genome_list, fitnesses, dist_fit = set_sliding_window_fitnesses(genome_list,fitnesses,args.window_size,fitness_function)

        print("Generation "+str(gen)+"\t: "+str(max(zip(*fitnesses)[1])))

        # Write the best performing individual to a file.
        mnlog.write_best_individual(output_path+"best_individuals/Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen)+".dat", 
                genome_list[dist_fit.index(max(dist_fit))])#genome_list[fitnesses.index(max(zip(*fitnesses)[2]))]
        morph_pop.logGenome(genome_list[fitnesses.index(max(fitnesses))].GetID(),"Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen),output_path)

        # Write information about the best individual we wrote.
        with open(output_path+"/"+str(run_num)+"_best_individuals_logging.dat","a") as f:
            f.write("Generation: "+str(gen)+" Individual is: "+str(genome_list[dist_fit.index(max(dist_fit))].GetID())+\
                " Fitness is: "+str(max(dist_fit))+"\n")

        # Log the progress of the entire population.
        mnlog.write_population_statistics_multi_component(output_path+str(run_num)+"_fitnesses.dat", genome_list, fitnesses, gen)

        # Log the final population for later evaluation.
        if gen == kwargs['gens'] - 1:
            population.Save(output_path+"run_"+str(run_num)+"_population_generation_"+str(gen)+".dat")

        # Create the next generation
        population.Epoch()
        morph_pop.NextGen()
        genome_list = NEAT.GetGenomeList(population)
        zip_args = []
        for ind in genome_list:
            pid1 = ind.GetPID1()
            pid2 = ind.GetPID2()
            gid = ind.GetID()

            # Handle Crossover
            morph_pop.Crossover(gid,pid1,pid2)

        # Handle Mutation
        morph_pop.MutatePopulation()

        # Zip the arguments for calling the evolution function.
        morph_genomes = morph_pop.getGenomes()
        zip_args = [(ind,morph_genomes[ind.GetID()]) for ind in genome_list]

######################################################################

# Process inputs.
parser = argparse.ArgumentParser()
parser.add_argument("--validator", action="store_true", help="Validate current results.")
parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
parser.add_argument("--run_num", type=int, default=0, help="Run Number")
parser.add_argument("--output_path", type=str, default="./", help="Output path")
parser.add_argument("--log_run",action="store_true", help="Load data from log file and run.")
parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
parser.add_argument("--val_num",type=int,default=-1,help="Number to identify the validator run.")
parser.add_argument("--debug_runtime",action="store_true",help="Evaluate the run time of a simulation.")
parser.add_argument("--config_file",type=str,default="../../config_files/default.cfg")
parser.add_argument("--config_string",type=str,default="",help="Use a string instead of file for configuration.")
parser.add_argument("--no_periodic",action="store_true",help="Whether we're including a periodic signal or not.")
parser.add_argument("--fit_func",type=str,default="Distance",help="What fitness function to use.")
parser.add_argument("--window_size",type=float,default=0.0,help="What is the window size (0.0-1.0).")
parser.add_argument("--fixed_strength",action="store_true",help="What type of ANN, fixed strength?")
args = parser.parse_args()

running = True
eval_time = args.eval_time 

output_path = args.output_path
run_num = args.run_num

# Seed only the evolutionary runs.
random.seed(run_num)

# Set the fitness function to use in the experiment.
if "Distance" in str(args.fit_func):#str(args.fit_func)[0] == "D":
    fitness_function = 1
else:
    fitness_function = 2
print("Distance Metric is: "+str(fitness_function)+" Argument is: "+str(args.fit_func))
sys.stdout.flush()

if args.debug_runtime:
    # Initialize the Simulation component
    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)

    ann_activations = simulation.validator("")

elif args.validator:
    # Initialize the Simulation component
    simulation = Simulation(log_frames=args.log_frames, run_num=args.run_num, eval_time=args.eval_time, dt=.02, n=4)

    # Ensure that the file exists before trying to open to avoid hanging.
    NEAT_file = output_path+"best_individuals/Evo_NEAT_run_"+str(args.run_num)+"_best_gen_"+str(args.gens)+".dat"
    if not os.path.isfile(NEAT_file):
        print("NEAT Genome file doesn't exist! "+NEAT_file)
        exit()

    # Open the morphology component of the genome.
    Morph_file = output_path+"best_individuals/Evo_NEAT_run_"+str(args.run_num)+"_best_gen_"+str(args.gens)+"_morph_genome.dat"
    if not os.path.isfile(Morph_file):
        print("Morph Genome file doesn't exist! "+Morph_file)
        exit()

    ann_activations,joint_feedback = simulation.validator(NEAT_file,Morph_file)

    dump_ann_activations(output_path+"/validation_logging/"+str(args.run_num)+"/ann_activations/"+str(args.run_num)+"_gen_"+str(args.gens)+"_ann_activations.dat", ann_activations)
    dump_joint_feedback(output_path+"/validation_logging/"+str(args.run_num)+"/joint_feedback/"+str(args.run_num)+"_gen_"+str(args.gens)+"_joint_feedback.dat", joint_feedback)

else:
    # Ensure that the necessary folders are created for recording data.
    if not os.path.exists(output_path+"best_individuals"):
        os.makedirs(output_path+"best_individuals")
    evolutionary_run(gens=args.gens,pop_size=args.pop_size)