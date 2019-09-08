
""" Evolve Ribbot for maximum distance traveled.

Evolve:
- frequency,
- amplitude, 
- and phase
for each hinge joint.
"""

# Python libraries
import math
import array
import random
import sys

# DEAP components
from deap import creator
from deap import base
from deap import tools

# My libraries
from ribbot import Ribbot, actuate_robot_sinusoids

#
# Evaluation parameters
#
NUM_JOINTS = 7
NDIM = 4 * NUM_JOINTS
AMP, FRQ, PHS = 0, NUM_JOINTS, NUM_JOINTS * 2

TOURN_SIZE = 3

#
# Problem specific evaluation function
#


def eval_ribbot(individual, json_fname=None):
    """Evaluate Ribbot for distance traveled.
    """
    # Extract genes
    amps = [individual[0][jointidx][0][0] for jointidx in range(NUM_JOINTS)]
    frqs = [individual[0][jointidx][0][1] for jointidx in range(NUM_JOINTS)]
    phss = [individual[0][jointidx][0][2] for jointidx in range(NUM_JOINTS)]
    bias = [0] * NUM_JOINTS
    use = [individual[0][jointidx][1] for jointidx in range(NUM_JOINTS)]
    gen_amp_freq_phss = individual[1]
    
    # Evaluate the individual
    robot = Ribbot(segment_count=NUM_JOINTS + 1, json_fname=json_fname)
    while robot.time < robot.time_stop:
        actuate_robot_sinusoids(robot, amps, frqs, bias, phss)
        robot.step()

    position = robot.bodies[0].getPosition()
    x_sign = 1 if position[0] > 0 else -1

    fitness = x_sign * math.sqrt(position[0] * position[0] + position[1] * position[1])
    return fitness,

#
# Register individual creator
# - amplitude   : [0.1 1.5]
# - frequency   : [0.1 3.0]
# - phase       : [0.0 2*pi]
#
tb = base.Toolbox()
BOUND_LO = [0.1] + [0.1] + [0.0]
BOUND_HI = [1.5] + [3.0] + [math.pi * 2]

#
# Create fitness object (maximize fitness --> weight = 1.0)
#
creator.create("FitnessMax", base.Fitness, weights=(1.0,))

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
	
creator.create("Individual", list, fitness=creator.FitnessMax)
tb.register("attr_switchable_jointgenome", switchable_jointgenome, NUM_JOINTS, BOUND_LO, BOUND_HI)
tb.register("attr_uniform_genome", uniform, BOUND_LO, BOUND_HI)
tb.register("individual", tools.initCycle, creator.Individual, [tb.attr_switchable_jointgenome, tb.attr_uniform_genome],n=1)
print tb.individual()


#
# Register population creator
#
tb.register("population", tools.initRepeat, list, tb.individual)

#
# Register the evaluation function
#
tb.register("evaluate", eval_ribbot)


def custom_mutation(individual, low, up, eta, indpb):
    polyboundmut = tools.mutPolynomialBounded
    flipbitmut = tools.mutFlipBit

    for individualidx in range(len(individual[:])):
        individual[0][individualidx][0] = polyboundmut(individual[0][individualidx][0], low=low, up=up, eta=eta, indpb=indpb)[0]
        individual[0][individualidx][1] = flipbitmut([individual[0][individualidx][1]],indpb=indpb)[0][0]
    
	individual[1] = polyboundmut(individual[1], low=low, up=up, eta=eta, indpb=indpb)[0]


#
# Register the genetic operators
#
ETA = 20.0
INDPB = 1.0 / NDIM

crossover = tools.cxTwoPoint
tb.register("mate", crossover)
tb.register("mutate", custom_mutation, low=BOUND_LO, up=BOUND_HI, eta=ETA, indpb=INDPB)
tb.register("select", tools.selTournament, tournsize=TOURN_SIZE)

#
# Main evolution function
#
#log_file = open('all_data.csv', 'w')
#ps = "\tAMPLITUDE{0}\tFREQUENCY{0}\tPHASE{0}"
#log_header = "GEN\tFIT" + "".join([ps.format(j) for j in xrange(NUM_JOINTS)]) + "\n"
#log_file.write(log_header)
#record_string = "\t{}" * NDIM + "\n"


def evolve(seed=None):
    random.seed(seed)

    #
    # Evolution parameters
    #
    NUM_GEN = 64
    POP_SIZE = 64
    CXPB = 0.9

    # Create and Evaluate the initial population
    pop = tb.population(n=POP_SIZE)
    fitnesses = tb.map(tb.evaluate, pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    # Log population data
    #for ind in pop:
    #    record = "{}".format(0)
    #    record += "\t{}".format(ind.fitness.values[0])
    #    record += record_string.format(*ind)
    #    log_file.write(record)

    for gen in range(1, NUM_GEN):
        print "Generation: {}".format(gen)

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

        # Select best from offspring and parents
        pop = tb.select(pop + offspring, POP_SIZE)

        # Log population data
        #for ind in pop:
        #    record = "{}".format(gen)
        #    record += "\t{}".format(ind.fitness.values[0])
        #    record += record_string.format(*ind)
        #    log_file.write(record)

    #log_file.close()

if __name__ == '__main__':

    if len(sys.argv) == 1:
        print "This program requires either a seed for the PRNG or a genome to evaluate."
    elif len(sys.argv) == 2:
        seed = int(sys.argv[1])
        evolve(seed=seed)
    else:
        ribbot_fname = "/Users/msu/Desktop/ribbot_validate.json"
        generation, fitness, genome = sys.argv[1], sys.argv[2], sys.argv[3:]
        genome = [float(g) for g in genome]
        result = eval_ribbot(genome, ribbot_fname)

        print "IN  : " + fitness
        print "OUT : " + str(result[0])
