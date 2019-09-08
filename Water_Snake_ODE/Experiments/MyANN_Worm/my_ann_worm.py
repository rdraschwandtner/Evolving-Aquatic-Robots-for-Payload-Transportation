import argparse
import GlobalVarWorkaround
import MultiNEAT as NEAT
import random
import os
import multiprocessing as mpc
import math
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from Controllers import MultiNEAT_logging as mnlog
from ODESystem import ODEManager
import my_worm_simulation

class MorphGenomes(object):
    """ Container for methods relating to evolving the morphology component of population. """

    def __init__(self,pop_size):
        """ Initialize the population. """
        self.pop_size = pop_size
        self.genomes = {}
        self.freq_min = 0.01
        self.freq_max = 4.

    def addIndividual(self,NEAT_id):
        """ Add an individual to the population keyed with the NEAT genome ID. """
        self.genomes[NEAT_id] = [float("{0:.6f}".format(random.uniform(self.freq_min, self.freq_max)))]

    def getGenomes(self):
        """ Return the genomes in the population. """
        return self.genomes

    def logGenomes(self,file_prefix):
        """ Log the genomes to an output file. """
        pass

    def logGenome(self,NEAT_id,file_prefix,output_path):
        """ Log a genome to an output file. """
        with open(output_path+"/best_individuals/"+file_prefix+"_morph_genome.dat","w") as f:
            f.write(str(self.genomes[NEAT_id][0])) #+","+str(self.genomes[NEAT_id][1]))

    def NextGen(self):
        """ Prepare the genomes for the next generation. """
        self.parent_genomes = self.genomes
        self.genomes = {}

    def Crossover(self,NEAT_id,par_1,par_2):
        """ Crossover two individuals to create a new individual. """
        # Crossover occurred 
        #if par_2 > 0:
        #    if random.random() < 0.5:
        #        self.genomes[NEAT_id] = [self.parent_genomes[par_1][0],self.parent_genomes[par_2][1]]
        #    else:
        #        self.genomes[NEAT_id] = [self.parent_genomes[par_2][0],self.parent_genomes[par_1][1]]
        #else:
        #    self.genomes[NEAT_id] = [self.parent_genomes[par_1][0],self.parent_genomes[par_1][1]]

        # check if crossover happended in NEAT by checking parent2
        # if negative -> no crossover happened
        if par_2 > 0:
            if random.random() < 0.5:
                self.genomes[NEAT_id] = [self.parent_genomes[par_1][0]]
            else:
                self.genomes[NEAT_id] = [self.parent_genomes[par_2][0]]
        else:
            self.genomes[NEAT_id] = [self.parent_genomes[par_1][0]]

    def MutatePopulation(self):
        """ Mutate the population. """
        for ind in self.genomes:
            #for i in range(len(self.genomes[ind])):
            if random.random() < 0.1:
                self.genomes[ind] = [float("{0:.6f}".format(random.uniform(self.freq_min, self.freq_max)))]

def replace_nanfitnesses(fitnesses):
    """ Ensure that the fitnesses are valid.

    Args:
        fitnesses: list of fitness values to Validate

    Returns:
        validated list of fitnesses adjusted for invalid fitnesses.
    """
    # Check each of the three fitness components for nan.  Set all three to low number if one is so.
    # Prevents the hanging problem seen in earlier runs.
    for i in range(len(fitnesses)):
        if math.isnan(fitnesses[i]):
            fitnesses[i] = 0.0000001
    return fitnesses

def evaluate_individual(individual):
    """ Wrapper to call Simulation which will evaluate an individual.  

    Args:
        individual: arguments to pass to the simulation

    Returns:
        fitness of an individual
    """
    log_frames = GlobalVarWorkaround.args.log_frames
    run_num = GlobalVarWorkaround.args.run_num
    eval_time = GlobalVarWorkaround.args.eval_time
    dt = .02
    n = 4

    
    GlobalVarWorkaround.man = ODEManager(my_worm_simulation.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0)
    GlobalVarWorkaround.worm = my_worm_simulation.Worm(man=GlobalVarWorkaround.man, morphology_genome=individual[1], num_joints=7, logging=log_frames)

    simulation = my_worm_simulation.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm)
    return simulation.evaluate_individual(individual)

def evolutionary_run(gens, pop_size, output_path, run_num, numofjoints):
    """ Conduct an evolutionary run using the snake and muscle model. 
    """
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
    params.CrossoverRate = 0.4

    assert pop_size >= 0, "wrong population size argument! pop_size: %d" % pop_size 
    params.PopulationSize = pop_size

    # worm has only one dof (turning around y-axis) per joint
    num_outputs = numofjoints
    # the inputs for the ANN are the 7 current joint positions and the amplitude of a sine wave as well as a bias
    num_inputs = numofjoints + 1 + 1

    # Initialize the population
    # Genome(ID, NumInputs, NumHidden, NumOutputs, ActivationFunction?, Output activation function, Hidden layer acitvation function, seed, params)
    genome = NEAT.Genome(0, num_inputs, 0, num_outputs, False, NEAT.ActivationFunction.SIGNED_SIGMOID, NEAT.ActivationFunction.SIGNED_SIGMOID, 0, params)
    # Population(Genome, params, randomizedweights?, randomrange)
    population = NEAT.Population(genome, params, True, 1.0)
    genome_list = NEAT.GetGenomeList(population)

    morph_pop = MorphGenomes(pop_size)
    for ind in genome_list:
        morph_pop.addIndividual(ind.GetID())
    morph_genomes = morph_pop.getGenomes()

    # ANN genome and morphology genome are zipped together
    # Zip the two genome components together for use in the parallel call.
    zip_args = [(ind,morph_genomes[ind.GetID()]) for ind in genome_list]

    mnlog.write_population_statistics_headers(output_path+str(run_num)+"_fitnesses.dat")

    # Setup multiprocessing
    cores = mpc.cpu_count()
    #cores = 1
    pool = mpc.Pool(initializer=initProcess, initargs=(GlobalVarWorkaround.args,GlobalVarWorkaround.man, GlobalVarWorkaround.worm,), processes=cores)

    assert gens >= 0, "wrong number of generations as argument! gens: %d" % gens

    for gen in xrange(gens):
        print gen
        #fitnesses = map(evaluate_individual,zip_args) # serial execution
        fitnesses = pool.map(evaluate_individual,zip_args)

        replace_nanfitnesses(fitnesses)

        for g,f in zip(genome_list,fitnesses):
            g.SetFitness(f)

        print("Generation "+str(gen)+"\t: "+str(max(fitnesses)))

        # Write the best performing individual to a file.
        mnlog.write_best_individual(output_path+"best_individuals/Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen)+".dat", 
                genome_list[fitnesses.index(max(fitnesses))])
        morph_pop.logGenome(genome_list[fitnesses.index(max(fitnesses))].GetID(),"Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen),output_path)

        # Write information about the best individual we wrote.
        with open(output_path+"/"+str(run_num)+"_best_individuals_logging.dat","a") as f:
            f.write("Generation: "+str(gen)+" Individual is: "+str(genome_list[fitnesses.index(max(fitnesses))].GetID())+\
                " Fitness is: "+str(max(fitnesses))+"\n")

        # Log the progress of the entire population.
        mnlog.write_population_statistics(output_path+str(run_num)+"_fitnesses.dat", genome_list, fitnesses, gen)

        # Log the final population for later evaluation.
        if gen == gens - 1:
            population.Save(output_path+"run_"+str(run_num)+"_population_generation_"+str(gen)+".dat")

        # Create the next generation
        population.Epoch()
        genome_list = NEAT.GetGenomeList(population)
        morph_pop.NextGen()        
        zip_args = []
        for ind in genome_list:
            # PID .. parent ID
            pid1 = ind.GetPID1()

            # if pid2 is negative it means that no crossover happend!
            pid2 = ind.GetPID2()
            gid = ind.GetID()

            # Handle Crossover
            morph_pop.Crossover(gid,pid1,pid2)

        # Handle Mutation
        morph_pop.MutatePopulation()

        # Zip the arguments for calling the evolution function.
        morph_genomes = morph_pop.getGenomes()
        zip_args = [(ind,morph_genomes[ind.GetID()]) for ind in genome_list]

# windows global variable workaround
# http://stackoverflow.com/questions/1675766/how-to-combine-pool-map-with-array-shared-memory-in-python-multiprocessing
def initProcess(paramargs, paramman, paramworm):
   GlobalVarWorkaround.args = paramargs
   GlobalVarWorkaround.man = paramman
   GlobalVarWorkaround.worm = paramworm

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
    parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
    parser.add_argument("--output_path", type=str, default="./", help="Output path")
    parser.add_argument("--run_num", type=int, default=0, help="Run Number")
    parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
    parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
    parser.add_argument("--validator", action="store_true", help="Validate current results.")
    GlobalVarWorkaround.args = parser.parse_args()

    print GlobalVarWorkaround.args

    if GlobalVarWorkaround.args.validator:
        # Ensure that the file exists before trying to open to avoid hanging.
        NEAT_file = GlobalVarWorkaround.args.output_path+"best_individuals/Evo_NEAT_run_"+str(GlobalVarWorkaround.args.run_num)+"_best_gen_"+str(GlobalVarWorkaround.args.gens)+".dat"
        if not os.path.isfile(NEAT_file):
            print("NEAT Genome file doesn't exist! "+NEAT_file)
            exit()

        # Open the morphology component of the genome.
        Morph_file = GlobalVarWorkaround.args.output_path+"best_individuals/Evo_NEAT_run_"+str(GlobalVarWorkaround.args.run_num)+"_best_gen_"+str(GlobalVarWorkaround.args.gens)+"_morph_genome.dat"
        if not os.path.isfile(Morph_file):
            print("Morph Genome file doesn't exist! "+Morph_file)
            exit()

        log_frames = GlobalVarWorkaround.args.log_frames
        run_num = GlobalVarWorkaround.args.run_num
        eval_time = GlobalVarWorkaround.args.eval_time
        dt = .02
        n = 4
        numofjoints = 7

        print GlobalVarWorkaround.args.output_path
        output_path = GlobalVarWorkaround.args.output_path
        gen = GlobalVarWorkaround.args.gens

        my_worm_simulation.validator(NEAT_file, Morph_file, log_frames, run_num, eval_time, dt, n, numofjoints, output_path, gen)
    else:
        # Ensure that the necessary folders are created for recording data.
        if not os.path.exists(GlobalVarWorkaround.args.output_path+"best_individuals"):
            os.makedirs(GlobalVarWorkaround.args.output_path+"best_individuals")
        evolutionary_run(gens=GlobalVarWorkaround.args.gens,pop_size=GlobalVarWorkaround.args.pop_size, output_path=GlobalVarWorkaround.args.output_path, run_num=GlobalVarWorkaround.args.run_num, numofjoints=7)