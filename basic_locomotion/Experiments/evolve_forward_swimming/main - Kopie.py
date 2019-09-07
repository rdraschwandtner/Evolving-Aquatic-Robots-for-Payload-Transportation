import os
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from ODESystem import ODEManager
import GlobalVarWorkaround
import simulation
import operator
import argparse
import math
import array
import random
import parse

# DEAP components
from deap import creator
from deap import base
from deap import tools

import multiprocessing


################################################################################
# Project specific store and load functions
################################################################################
def logGenome(file_prefix,output_path,genome):
    """ Log a genome to an output file. """
    with open(output_path+"/best_individuals/"+file_prefix+"_morph_genome.dat","w") as f:
        f.write(str(genome)) #+","+str(self.genomes[NEAT_id][1]))

def store_gen_stats(fname, generation, individualindices, fitnesses):
    ''' Write generation statistics to a CSV file:
        generation i; individualindex i; fitness i
        generation i+1; individualindex i+1; fitness i+1
    '''

    f = 0
    if os.path.isfile(fname):
        # file exists - append new conctent
        f = open(fname, 'a')
    else:
        f = open(fname, 'w')
        # write header
        f.write('generation;index;fitness\n')

    # write body
    for idx in individualindices:
        f.write(`generation`+';'+`idx`+';'+`fitnesses[idx]`+'\n')    
    f.close()
################################################################################


def evolve(output_path, run_num, num_gens, pop_size, seed):
    random.seed(seed)

    #
    # Evolution parameters
    #
    assert num_gens >= 0, "wrong number of generations argument! num_gens: %d" % num_gens 
    NUM_GEN = num_gens #64
    assert pop_size >= 0, "wrong population size argument! pop_size: %d" % pop_size 
    POP_SIZE = pop_size #64
    CXPB = 0.9

    # Create and Evaluate the initial population
    pop = tb.population(n=POP_SIZE)
    fitnesses = tb.map(tb.evaluate, pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    # Log population data
    # Write the best performing individual to a file.
    init_gen = 0
    print "Generation: {}".format(init_gen) + "\t fitness="+str(max(fitnesses)) + "\t @idx:" + str(fitnesses.index(max(fitnesses)))
    store_gen_stats(output_path+str(run_num)+"_fitnesses.dat",init_gen,range(len(fitnesses)),fitnesses)
    logGenome("Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(init_gen),output_path
                , pop[fitnesses.index(max(fitnesses))])

    # Write information about the best individual we wrote.
    with open(output_path+"/"+str(run_num)+"_best_individuals_logging.dat","a") as f:
        f.write("Generation: "+str(init_gen)+" Individual is: "+str(fitnesses.index(max(fitnesses)))+\
            " Fitness is: "+str(max(fitnesses))+"\n")

    for gen in range(1, NUM_GEN):
        # Select and clone to create the next generation of individuals
        offspring = tb.select(pop, POP_SIZE)
        offspring = [tb.clone(ind) for ind in offspring]

        # Perform crossover and mutation on the offspring
        for ind1, ind2 in zip(offspring[::2], offspring[1::2]):
            if random.random() <= CXPB:
                tb.mate(ind1, ind2)

            tb.mutate(ind1)
            tb.mutate(ind2)
            del ind1.fitness.values, ind2.fitness.values

        # Evaluate offspring
        fitnesses = tb.map(tb.evaluate, offspring)
        for ind, fit in zip(offspring, fitnesses):
            ind.fitness.values = fit


        # Log population data
        # Write the best performing individual to a file.
        print "Generation: {}".format(gen) + "\t fitness="+str(max(fitnesses)) + "\t @idx:" + str(fitnesses.index(max(fitnesses)))
        store_gen_stats(output_path+str(run_num)+"_fitnesses.dat",gen,range(len(fitnesses)),fitnesses)
        
        logGenome("Evo_NEAT_run_"+str(run_num)+"_best_gen_"+str(gen),output_path
                    , offspring[fitnesses.index(max(fitnesses))])

        # Write information about the best individual we wrote.
        with open(output_path+"/"+str(run_num)+"_best_individuals_logging.dat","a") as f:
            f.write("Generation: "+str(gen)+" Individual is: "+str(fitnesses.index(max(fitnesses)))+\
                " Fitness is: "+str(max(fitnesses))+"\n")

        # Select best from offspring and parents
        pop = tb.select(pop + offspring, POP_SIZE)


#
# Problem specific evaluation function
#
def eval_ribbot(individual):
    """Evaluate Ribbot for distance traveled.
    """

    log_frames = False
    dt = .02
    n = 4
    run_num = GlobalVarWorkaround.args.run_num
    eval_time = GlobalVarWorkaround.args.eval_time
    num_joints = GlobalVarWorkaround.args.num_joints

    # Extract genes
    amps = [individual[0][jointidx][0][0] for jointidx in range(num_joints)]
    frqs = [individual[0][jointidx][0][1] for jointidx in range(num_joints)]
    phss = [individual[0][jointidx][0][2] for jointidx in range(num_joints)]
    bias = 0
    use = [individual[0][jointidx][1] for jointidx in range(num_joints)]
    default_amp_freq_phss = individual[1]

    # replace all parameters flagged with False with the default parameters from the genome
    for jointidx in range(num_joints):
        if use[jointidx] == False:
            amps[jointidx], frqs[jointidx], phss[jointidx] = default_amp_freq_phss

    GlobalVarWorkaround.man = ODEManager(simulation.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)

    GlobalVarWorkaround.worm = simulation.Worm(man=GlobalVarWorkaround.man,num_joints=num_joints,logging=log_frames)

    print '--amps:' + str(amps) + '\n--frqs:'  + str(frqs) + '\n--phss:' + str(phss)
    jointctrlers = [simulation.JointActuationController(amplitude=amps[i], frequency=frqs[i], phaseshift=phss[i], offset=bias) for i in range(num_joints)]
    simulationobj = simulation.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm, jointctrlers=jointctrlers)
    
    fit = simulationobj.evaluate_individual()
    print '--fit=' + str(fit)

    fitness = fit[0] # reward only distance traveled in positive x direction

    return fitness,


# windows global variable workaround
# http://stackoverflow.com/questions/1675766/how-to-combine-pool-map-with-array-shared-memory-in-python-multiprocessing
def initProcess(paramargs, paramman, paramworm):
   GlobalVarWorkaround.args = paramargs
   GlobalVarWorkaround.man = paramman
   GlobalVarWorkaround.worm = paramworm




#
# DEAP creators must be defined globally
# see https://groups.google.com/forum/#!msg/deap-users/G_Zod2OSGMo/gxHZYMsMiEsJ
#
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)


################################################################################
# Main entry point
################################################################################
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
    parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
    parser.add_argument("--output_path", type=str, default="./", help="Output path")
    parser.add_argument("--num_joints", type=int, default=8, help="Number of robot joints.")
    parser.add_argument("--validator", action="store_true", help="Validate current results.")
    parser.add_argument("--run_num", type=int, default=0, help="Run Number")
    parser.add_argument("--eval_time", type=float, default=20., help="Simulation time for an individual.")
    GlobalVarWorkaround.args = parser.parse_args()

    print GlobalVarWorkaround.args

    num_joints = GlobalVarWorkaround.args.num_joints

    run_num = GlobalVarWorkaround.args.run_num
    eval_time = GlobalVarWorkaround.args.eval_time
    dt = .02
    n = 4

    if GlobalVarWorkaround.args.validator == False: # simulate

        #
        # Evaluation parameters
        #
        NUM_JOINTS = num_joints
        NDIM = 3
        AMP, FRQ, PHS = 0, 1, 2

        TOURN_SIZE = 3

        #
        # Register individual creator
        # - amplitude   : [0.1 1.5]
        # - frequency   : [0.1 3.0]
        # - phase       : [0.0 2*pi]
        #
        tb = base.Toolbox()
        BOUND_LO = [0.1] + [0.1]+ [0.0]
        BOUND_HI = [1.5] + [3.0] + [math.pi * 2]

        #
        # Create individual object (array of floating point values)
        #
        def uniform(lo, hi):
            """Return a list of uniform values given the lists of hi and lo bounds."""
            return [random.uniform(a, b) for a, b in zip(lo, hi)]

        def randbool():
            return bool(random.randint(0, 1))

        def switchable_jointgenome(num_joints, lo, hi):
            return [[uniform(lo,hi), randbool()] for i in range(num_joints)]

        tb.register("attr_uniform", uniform, BOUND_LO, BOUND_HI)
        tb.register("attr_switchable_jointgenome", switchable_jointgenome, NUM_JOINTS, BOUND_LO, BOUND_HI)
        tb.register("individual", tools.initCycle, creator.Individual, [tb.attr_switchable_jointgenome, tb.attr_uniform],n=1)

        #
        # Register population creator
        #
        tb.register("population", tools.initRepeat, list, tb.individual)

        #
        # Register the evaluation function
        #
        tb.register("evaluate", eval_ribbot)

        #
        # Register the genetic operators
        #
        ETA = 20.0
        INDPB = 1.0 / NDIM


        def custom_mutation(individual, low, up, eta, indpb):
            polyboundmut = tools.mutPolynomialBounded
            flipbitmut = tools.mutFlipBit

            for individualidx in range(len(individual[:])):
                individual[0][individualidx][0] = polyboundmut(individual[0][individualidx][0], low=low, up=up, eta=eta, indpb=indpb)[0]
                individual[0][individualidx][1] = flipbitmut([individual[0][individualidx][1]],indpb=indpb)[0][0]
    
            individual[1] = polyboundmut(individual[1], low=low, up=up, eta=eta, indpb=indpb)[0]  

        crossover = tools.cxTwoPoint

        tb.register("mate", crossover)
        tb.register("mutate", custom_mutation, low=BOUND_LO, up=BOUND_HI, eta=ETA, indpb=INDPB)
        tb.register("select", tools.selTournament, tournsize=TOURN_SIZE)

        #
        # Make DEAP multiprocessed when using map()
        #
        cores = multiprocessing.cpu_count()
        pool = multiprocessing.Pool(initializer=initProcess, initargs=(GlobalVarWorkaround.args,GlobalVarWorkaround.man, GlobalVarWorkaround.worm,), processes=cores)
        tb.register("map", pool.map)


        # Ensure that the necessary folders are created for recording data.
        if not os.path.exists(GlobalVarWorkaround.args.output_path+"best_individuals"):
            os.makedirs(GlobalVarWorkaround.args.output_path+"best_individuals")

        evolve(GlobalVarWorkaround.args.output_path,GlobalVarWorkaround.args.run_num, GlobalVarWorkaround.args.gens, GlobalVarWorkaround.args.pop_size, run_num)

    else:   # validate
        log_frames = True

        # Open the morphology component of the genome.
        Morph_file = GlobalVarWorkaround.args.output_path+"best_individuals/Evo_NEAT_run_"+str(GlobalVarWorkaround.args.run_num)+"_best_gen_"+str(GlobalVarWorkaround.args.gens)+"_morph_genome.dat"
        if not os.path.isfile(Morph_file):
            print("Morph Genome file doesn't exist! "+Morph_file)
            exit()

        GlobalVarWorkaround.man = ODEManager(simulation.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)
        GlobalVarWorkaround.worm = simulation.Worm(man=GlobalVarWorkaround.man,num_joints=num_joints,logging=log_frames, log_path=GlobalVarWorkaround.args.output_path, logfileprefix='rep' + str(run_num)+ '_gen'+str(GlobalVarWorkaround.args.gens))

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        # Must be placed after creating the quadruped.
        if log_frames:
            GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

        amps=[]
        frqs=[]
        phss=[]
        use=[]
        default_amp_freq_phss=[]
        # Load the morphology
        with open(Morph_file,"r") as f:
            line = f.readline()
            individual_str = parse.parse('[[[[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}],'\
               ' [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}],'\
               ' [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}], [[{}, {}, {}], {}],'\
               ' [[{}, {}, {}], {}], [[{}, {}, {}], {}]], [{}, {}, {}]]', line)

            print individual_str
            for jointindex in range(0,num_joints*4,4):
                amps.append(float(individual_str[jointindex+0]))
                frqs.append(float(individual_str[jointindex+1]))
                phss.append(float(individual_str[jointindex+2]))
                use.append(bool(individual_str[jointindex+3]))

            default_amp_freq_phss = [individual_str[num_joints*4],individual_str[num_joints*4+1],individual_str[num_joints*4+2]]


        bias = 0

        print 'amps:' + str(amps) + '\nfrqs:'  + str(frqs) + '\nphss:' + str(phss) +'\nuse:' + str(use)
        print 'default:'+str(default_amp_freq_phss)
        exit()
        jointctrlers = [simulation.JointActuationController(amplitude=amps, frequency=frqs, phaseshift=phss, offset=bias) for i in range(num_joints)]
        simulation = simulation.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm, jointctrlers=jointctrlers)
        
        fit = simulation.evaluate_individual()
        print fit
        fitness = fit[0]
        print 'fitness=' + str(fitness)