# My first Watersnake code
import ode
from VisualizationJSON import *
import copy
import math
import random
import itertools
import operator
import argparse
import fnmatch
import os
import multiprocessing as mpc
from functools import partial

# calculation taken from Jared
def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 

##############################################
# My GA code

# Evolve following joint properties:
#   - Amplitude
#   - Frequency
#   - Pahse shift
# under the assumtion that joints behave like sine waves.
# estimated position = A * sin(2*pi*f*t+phi)


def init_genotype(genotype, numofrepititions, amp_min, amp_max, freq_min, freq_max, phi_min, phi_max):    
    ''' Initializes amplitude, frequency and phi by overwrites theses values (numofrepititions) in the genotype object.
        amplitude = [amp_min,amp_max]
        frequency = [freq_min,freq_max]
        phase shift = [phi_min,phi_max]
        Values are initialized according a uniform distribution.
    '''
    del genotype[:] # remove old values in genotype object
    for repidx in range(numofrepititions):
        # floating point precision see http://stackoverflow.com/questions/15263597/python-convert-floating-point-number-to-certain-precision-then-copy-to-string
        a = float("{0:.6f}".format(random.uniform(amp_min, amp_max))) # init amplitude
        f = float("{0:.6f}".format(random.uniform(freq_min, freq_max))) # init frequency
        phi = float("{0:.6f}".format(random.uniform(phi_min, phi_max))) # init phase shift
        genotype.append((a,f,phi))

def mutation(genotype, amp_min, amp_max, freq_min, freq_max, phi_min, phi_max):
    ''' This function returns a mutated genotype.
        Mutation rate is hardcoded to 1%.
        Mutation of a parameter occurs along a uniform distribution with hardcoded min and max values.
    '''
    mutated_genotype = copy.deepcopy(genotype)

    # mutation probability:
    #  1% : mutation_chance = 100
    # 10% : mutation_chance = 10
    # 50% : mutation_chance = 2
    mutation_chance = 100 # 1 / mutation_chance that a mutation occurs per gene

    for jointidx in range(len(mutated_genotype)):
        a = mutated_genotype[jointidx][0]
        f = mutated_genotype[jointidx][1]
        phi = mutated_genotype[jointidx][2]
        # todo: rethink those values
        if int(random.random()*mutation_chance) == 1:
            # floating point precision see http://stackoverflow.com/questions/15263597/python-convert-floating-point-number-to-certain-precision-then-copy-to-string
            a = float("{0:.6f}".format(random.uniform(amp_min, amp_max)))
        if int(random.random()*mutation_chance) == 1:
            f = float("{0:.6f}".format(random.uniform(freq_min, freq_max)))
        if int(random.random()*mutation_chance) == 1:
            phi = float("{0:.6f}".format(random.uniform(phi_min, phi_max)))

        mutated_genotype[jointidx] = (a,f,phi)

    return mutated_genotype

def crossover(genotype1, genotype2):
    """ This function performs a one point crossover between two genotypes and returns the resulting two new genotypes.
        Crossover position is determined with a uniform position.
        Crossover probability is 100%.
    """
    numofrepititions = len(genotype1) # todo assert that genotype1 has the same length as genotype2
    # assumption: crossover only allowed with full joints
    jointidx = random.randint(0, numofrepititions-1)
    return (genotype1[:jointidx]+genotype2[jointidx:], genotype2[:jointidx]+genotype1[jointidx:])

def fitness(startpos, endpos):
    ''' Fitness is calculated with euclidean distance between start position and end position.
    '''
    # force worm to swim in one direction
    # multiply the euclidean distance with the -1 if the endpos lies in -x area
    return math.copysign(1, endpos[0]) * euclidean_distance(startpos, endpos)

def roulette_wheel_prop_list(individual_fitnesses):
    """ Returns the proportion of individuals on the roulette wheel
        taken from http://sirineslihan.blogspot.com/2013/09/python-roulette-wheel-selection.html
        as an ordered list.
    """
    total_fit = float(sum(individual_fitnesses))
    relative_fitness = [f/total_fit for f in individual_fitnesses]
    # cumulative probabilities
    probabilities = [sum(relative_fitness[:i+1]) for i in range(len(relative_fitness))]
    return probabilities

def roulette_wheel_pop(population, probabilities, numberofindividuals):
    """ Returns the choosen individual according to roulette wheel selection
        taken from http://sirineslihan.blogspot.com/2013/09/python-roulette-wheel-selection.html
    """
    chosen = []
    for n in xrange(numberofindividuals):
        r = random.random()
        # find the individual on the roulette wheel
        for (individualidx, individual) in enumerate(population):
            # relate the random value to the individual on the wheel
            # depends on cumulative probabilities
            if r <= probabilities[individualidx]:
                chosen.append(list(individual))
                break
    return chosen

def tournament_selection(population, fitnesses, k):
    ''' choose k (the tournament size) individuals from the population at random.
        Assumes theat population and fitnesses lists correspond.
        Returns the winner genotype.
    '''
    assert (len(population) >= k),"Populationsize smaller than tournament size"
    assert (len(population) == len(fitnesses)),"population and fitness lists are not of the same size"

    # choose randomly k tournament member indices from population with replacement 
    tournamentmembers = random.sample(population, k);
    tournamentmemberidc = [population.index(mem) for mem in tournamentmembers]
    tournamentmember_fitnesses = [fitnesses[idx] for idx in tournamentmemberidc]
    # index of best member
    winneridx = fitnesses.index(max(tournamentmember_fitnesses))

    #return the genotype of the winner
    return copy.deepcopy(population[winneridx]);


##############################################

def create_surfaces(bodie):
    ''' Add bodie surface to global surface container.
        (Taken from Jared.)
    '''
    side_lens = bodie.boxsize
    x = side_lens[0]
    y = side_lens[1]
    z = side_lens[2]

    amp_adj = 1

    surfaces = []
    surfaces.append({'area':y*z, 'norm':[1,0,0], 'amp_adj':amp_adj})
    surfaces.append({'area':x*z, 'norm':[0,1,0], 'amp_adj':amp_adj})
    surfaces.append({'area':y*z, 'norm':[-1,0,0], 'amp_adj':amp_adj})
    surfaces.append({'area':x*z, 'norm':[0,-1,0], 'amp_adj':amp_adj})
    surfaces.append({'area':y*x, 'norm':[0,0,1], 'amp_adj':amp_adj})
    surfaces.append({'area':y*x, 'norm':[0,0,-1], 'amp_adj':amp_adj})

    return surfaces

def rotate3(m, v):
    '''Returns the rotation of 3-vector v by 3x3 (row major) matrix m.
        (Taken from Jared)
    '''
    return (v[0] * m[0] + v[1] * m[1] + v[2] * m[2],
        v[0] * m[3] + v[1] * m[4] + v[2] * m[5],
        v[0] * m[6] + v[1] * m[7] + v[2] * m[8])

#
# Global simulation parameters for aquatic envirnment
#
TIME_STOP = 20
TIME_STEP = 0.005
# in aquatic environment:
# y accelaration = gravity - buoyancy = 9.81 + 9.81 = 0
GRAVITY = (0, 0, 0) # (X,Y,Z) acceleration
SIMULATION_ENV = "AQUATIC"

# Wrappes ODE functions in a class in order to create
# multiple ODE instance at runtime (needed for parallelization)
class odewrapper_t:
    def __init__(self):
        ''' Set global Simulation variables and initialize global storage containers.
        '''
        self.world = ode.World()
        GRAVITY = (0, 0, 0)
        SIMULATION_ENV = "AQUATIC"
        self.world.setGravity(GRAVITY)
        self.space = ode.Space()
        ground_normal = (0., 1., 0.)
        ground = ode.GeomPlane(self.space, ground_normal, 0)
        self.contact_group = ode.JointGroup()

        self.bodies = []
        self.geoms = []
        self.joints = []
        self.no_collide = []
        self.surfaces_per_obj_map = []


    def apply_fluiddymics(self,simu_stepsize):
        """ Simulate an aquatic environment.
        Only done if the fluid_dynamics flag is on in manager.
        NOTE: Fluid dynamics are only in place for capsules and boxes.
        Modified Jared's apply_fluiddymics() method.
        """

        drag_coefficient = .35 # taken from Jared
        amp_adj = 1. # taken from Jared

        # apply forces for each body.
        for bodyidx in range(len(self.bodies)):
            vel = self.bodies[bodyidx].getLinearVel()
            rot = self.bodies[bodyidx].getRotation()

            # apply forces to each face of a body.
            # Iterate through the 6 faces of a body (assuming box x,y,z parameters)
            for surfaceidx in xrange(6):
                area = self.surfaces_per_obj_map[bodyidx][surfaceidx]['area'] # Area of the surface
                norm = self.surfaces_per_obj_map[bodyidx][surfaceidx]['norm'] # Norm vector [x,y,z] of the surface
                amp_adj = self.surfaces_per_obj_map[bodyidx][surfaceidx]['amp_adj'] # Apply an amplitude adjustment for a body generating thrust

                adnorm = rotate3(rot,norm) # Reorient the normal to the bodies rotation in the world.

                component = adnorm[0]*vel[0]+adnorm[1]*vel[1]+adnorm[2]*vel[2]  # Get the component of force perpendicular to surface.
                component *= drag_coefficient*area  # Compensating for the size of the surface.
                if(component < 0):
                    component = 0    # If less than zero, no drag force to apply (Surface is opposite to drag force.)
                
                force = self.world.impulseToForce(simu_stepsize,[-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])
                self.bodies[bodyidx].addForce([-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])  # Add force to the body from calculated surface.

    def inNoCollide(self, geom1, geom2):
        ''' Is geom1 - geom2 pair in the no_collide container?
            Returns true or false.
        '''
        # Check for restricted collisions
        # and do nothing if geom1 - geom2 is in no_collide container
        for pair in self.no_collide:
            if geom1.getBody() in pair and geom2.getBody() in pair:
                return "true"
        return "false"

    def Collide(self, geom1, geom2):
        ''' Wrapper method for ODE collide function.
        '''
        return ode.collide(geom1, geom2)

    def CreateJoint(self, world, jointgroup, contact):
        ''' Wrapper method for ODE contact joint function.
        '''
        return ode.ContactJoint(world, jointgroup, contact)


    def add_cube(self, size, density, position, quaternion=None, color=0x555555):
        """Create a cube/box object, inlcuding visualization parameters in the ODE environment.
        """

        # Create rigid body and set position and rotation
        cube = ode.Body(self.world)
        cube.setPosition(position)
        if quaternion is not None:
            cube.setQuaternion(quaternion)

        mass = ode.Mass()
        mass.setBox(density, *size) # * .. like ANSI C varargs
        cube.setMass(mass)
        cube.boxsize = list(size)

        geom = ode.GeomBox(self.space, lengths=size)
        geom.setBody(cube)    
        self.geoms.append(geom)

        ## Tony's visualization code specific
        vis_prim = create_json_primitive("cube", size, color)
        json_object["primitives"].append(vis_prim)

        self.bodies.append(cube)
        return cube

    def simulationrun(self, genotype, store='false', fname=''):
        ''' Executes an ODE simulation with a given genotype.
            Visualization is optional.
            Returns the position of the first segment.
        '''

        # init the visualization module if store parameter is true.
        if store == 'true':
            visualizationJSON_init (TIME_STOP, TIME_STEP)

        #############################################################
        # setup the ODE simulation environment for the simulation run

        # specify the segment as box
        # (length, width, height)
        cube_size = (1.5, 0.5, 0.5)
        cube_density = .1 # determined by trial and error
        initial_cube_position_middle = list((1., cube_size[2], cube_size[2] * 0.5)) # place the following box half the length behind the front box

        NUM_OF_SEGMENTS = 8 # hardcoded 8 segments

        initial_cube = self.add_cube(cube_size, cube_density, initial_cube_position_middle)
        initial_cube_surfaces = create_surfaces(initial_cube)
        self.surfaces_per_obj_map.append(initial_cube_surfaces)

        prev_cube = initial_cube
        prev_cube_position = copy.deepcopy(initial_cube_position_middle)

        # create NUM_OF_SEGMENTS segments with hinge joint in between
        for i in range(1, NUM_OF_SEGMENTS):
            # create a cube
            cur_cube_position = copy.deepcopy(prev_cube_position)
            cur_cube_position[0] += cube_size[0] #+ cube_size[0]/16
            #cur_cube_position = list(cur_cube_position) # todo check if necessary
            cur_cube = self.add_cube(cube_size, cube_density, cur_cube_position)
            cur_cube_surfaces = create_surfaces(initial_cube)
            self.surfaces_per_obj_map.append(cur_cube_surfaces)

            joint = ode.HingeJoint(self.world)
            joint.attach(prev_cube, cur_cube)
            jointpos = copy.deepcopy(prev_cube_position)
            # place the joint exactly in the middle (x-axis) of both segments
            jointpos[0] = prev_cube_position[0] + (cur_cube_position[0] - prev_cube_position[0])/2
            joint.setAnchor((jointpos))

            joint.setAxis((0,1,0)) # turn arround y- axis
            self.joints.append(joint)

            # joint angle can be a value of [-1.6,+1.6]
            joint.setParam(ode.ParamLoStop, -1.6) # where lo < hi (radians)
            joint.setParam(ode.ParamHiStop, +1.6) # where hi > lo (radians)
            joint.setParam(ode.ParamFMax, 10) # maximum force on joint

            # avoid ODE collision of two neighboring segments
            self.no_collide.append((prev_cube, cur_cube))

            prev_cube = cur_cube
            prev_cube_position = copy.deepcopy(cur_cube_position)

        #############################################################
        # execute the simulation
        counter = 0
        time = 0.0
        while time < TIME_STOP:
            
            # calculate all joint velocities
            for i in range(0, NUM_OF_SEGMENTS-1):
                # genotype[i][0] .. amplitude
                # genotype[i][1] .. frequency
                # genotype[i][2] .. phi
                joint_vel = velocity_sinusoidal(self.joints[i].getAngle(), time, TIME_STEP, genotype[i][0], genotype[i][1], genotype[i][2], i)
                self.joints[i].setParam(ode.ParamVel, joint_vel)
            
            # apply aquatic forces if predefined only
            if SIMULATION_ENV == "AQUATIC":
                self.apply_fluiddymics(TIME_STEP)

            # Collide all objects
            self.space.collide((self.world, self.contact_group, self), near_callback)

            # Advance the physics world through time
            self.world.step(TIME_STEP)
            time += TIME_STEP

            # Reset collisions
            self.contact_group.empty()

            #print joint.getAngle()

            # Update json primitives
            #if int(time / TIME_STEP) % divisor == 0:
            if store == 'true':
                update_json(self.bodies)

        #############################################################
        if store == 'true':
            print '# write visualization file #'
            write_json(fname)

        if store == 'true':
            print euclidean_distance((0,0,0), self.bodies[0].getPosition()) # (0,0,0) assumed starting position

        return self.bodies[0].getPosition() # return position of the first segment


def velocity_sinusoidal(cur_angle, cur_time, time_step, amp, freq, phi, i):
    """ Calculate velocity to move from the current angle to the expted angle
        within the next timestep.
        Sine wave is utilized to calculate the expected position at a point of time.
        Amplite, Frequency and Phi are given as arguments an shall represent a joint's properties.
    """
    
    w = 2. * math.pi * freq # calc angular frequency
    offset = 0 # no offset is applied

    # calc the expected angle
    exp_angle = amp * math.sin(w * cur_time+i*phi) + offset
    # calculate the differnce between the expected and current angle
    delta_angle = exp_angle - cur_angle

    # calculate how fast the joint shall move within one timestep in order to reach
    # the expected position.
    v = delta_angle / time_step
        
    return v

def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms collide and
    creates contact joints if necessary.
    """

    world, contact_group, odewrapper = args # arg = (world, contact_group, odewrapper)

    if odewrapper.inNoCollide(geom1, geom2) == "true":
        return

    # if the geom1 - geom2 pair isn't in the no_collide container
    # then handle the collision
             
    # Check for collisions
    odewrapper.contacts = odewrapper.Collide(geom1, geom2)

    # Create contact joints
    # world, contact_group = args # arg = (world, contact_group)
    for c in odewrapper.contacts:
        c.setBounce(1.)
        c.setMu(5000)
        j = odewrapper.CreateJoint(world, contact_group, c)
        j.attach(geom1.getBody(), geom2.getBody())

#######################################################################
# Project specific store and load functions

def store_gen_stats(fname, generation, individualindices, fitnesses):
    ''' Write generation statistics to a CSV file:
        generation i; individualindex i; fitness i
        generation i+1; individualindex i+1; fitness i+1
    '''
    f = open(fname, 'w')
    # write header
    f.write('generation;index;fitness\n')
    # write body
    for idx in individualindices:
        f.write(`generation`+';'+`idx`+';'+`fitnesses[idx]`+'\n')    
    f.close()

def store(fname, genotype, fitness, generation, index):
    '''Write the genotype and fitness, generation and index data to a json file.
    '''
    text = json.dumps((genotype, fitness, generation, index))
    with open(fname, 'w') as json_file:
        json_file.write(text)

def load(fname):
    ''' Load the genotype and fitness, generation and index data to a json file.
        Returns a object stored by store().
    '''
    f = open(fname, 'r')
    x = json.load(f)
    return x
#######################################################################

def evaluate_genotype(genotype):
    ''' Create a new ODE instantiation and run a simulation with the given genotype.
        This function returns the fitness of the evaluated genotype.
    '''
    odewrapper = odewrapper_t()
    endpos = odewrapper.simulationrun(genotype)
    dist = euclidean_distance((0,0,0), endpos)    
    return fitness((0,0,0), endpos) # equals the distance, in this case

def findNumOfParents(MaxNumOfOffsprings):
    ''' Find the closest number of parents for a maximum number of offsprings.
        Maybe useless in future GA implementations.
    '''
    numofparents=MaxNumOfOffsprings

    while sum(xrange(numofparents)) > MaxNumOfOffsprings:
        numofparents-= 1

    return numofparents

def generateoffspring(population, fitnesses):
    # parent selection with tournament selection, with replacement (= parent1 != parent2)
    parent1 = tournament_selection(population, fitnesses, 2)
    parent1idx = population.index(parent1)
    parent2 = tournament_selection(population[:parent1idx] + population[(parent1idx + 1):] # exclude parent1 from population
                                    , fitnesses[:parent1idx] + fitnesses[(parent1idx + 1):] # exclude the fitness value of parent1
                                    , 2) # tournament size  = 2
        
    # Perform a one point cross
    # 100% crossover probabiltiy
    children = crossover(parent1, parent2) # returns two crossovers

    # Perform mutation
    child = mutation(children[0], AMPLITUDE_min, AMPLITUDE_max, FREQUENCY_min, FREQUENCY_max, PHASESHIFT_min, PHASESHIFT_max)
    # don't use child 2
    # child2 = mutation(children[1], AMPLITUDE_min, AMPLITUDE_max, FREQUENCY_min, FREQUENCY_max, PHASESHIFT_min, PHASESHIFT_max)

    # calculate fitness of the child
    childfitness = evaluate_genotype(child)

    return child, childfitness

def generateoffspring_chunkwrapper(number, population, fitnesses):
   # print number
   ret = generateoffspring(population, fitnesses)
   return [ret[0], ret[1]]

#def test(i):
#    print i
#    return i*i

#def test2(a,b,i):
#    return a+b+i

#def test3(a,b,i):
#    return ([9,99,99],a,b,i)

AMPLITUDE_min = 0.001
AMPLITUDE_max = 2.
FREQUENCY_min = 0.001
FREQUENCY_max = 4.
PHASESHIFT_min = 0.
PHASESHIFT_max = math.pi * 2.

def dosimulation(populationsize, numofgenerations, fpath='./'):
    ''' Executes my simple GA with a give population size and with a give number of generations.
    '''
    population = []
    individual_fitnesses = []

    overallbestindividual = []
    overallbestindividualfitness = 0
    overallbestindividualgeneration = -1

    # Create initial population
    # No randomized individual genotypes!
    for individualidx in range(populationsize):
        genotype = []
        init_genotype(genotype, 8-1, AMPLITUDE_min, AMPLITUDE_max, FREQUENCY_min, FREQUENCY_max, PHASESHIFT_min, PHASESHIFT_max) #8-1 .. 7 joints
        population.append(genotype)

    # Initialize multiprocessing
    # all processes are used for calculation
    print '## used processes: ', `(mpc.cpu_count())`
    pool = mpc.Pool(processes=mpc.cpu_count())
    individual_fitnesses = pool.map(evaluate_genotype, population)

    for generation in range(numofgenerations-1): #do while
        # print the best individual
        bestindividual_fitness = max(individual_fitnesses)
        bestindividual_index = individual_fitnesses.index(bestindividual_fitness)
        print 'gen: ', generation, ' bestfitness: ', bestindividual_fitness, ' @idx: ', bestindividual_index
        # store the best individual's properties
        json_filename = fpath + 'snake_gen' + `generation` + '_bestindvidx' + `bestindividual_index` + '.json'
        store(json_filename, population[bestindividual_index], bestindividual_fitness, generation, bestindividual_index)
        # store generation statistics
        store_gen_stats(fpath + 'gen' + `generation` + 'stats.json' , generation, range(len(individual_fitnesses)), individual_fitnesses)

        partial_generateoffspring_chunkwrapper = partial(generateoffspring_chunkwrapper, population=population, fitnesses=individual_fitnesses)
        # serial version pool.map()
        # result = map(partial_generateoffspring_mpcwrapper, range(populationsize))
        newpop_and_fit = pool.map(partial_generateoffspring_chunkwrapper, range(populationsize))

        # new population becomes next generation's population
        newpop = [row[0] for row in newpop_and_fit] # first column contains the individuals
        population = copy.deepcopy(newpop) # take the population column from newpop_and_fit

        # new fitness becomes next generation's fitness
        newfit = [row[1] for row in newpop_and_fit] # second column contains the fitness
        individual_fitnesses = copy.deepcopy(newfit) #take the fitness column from newpop_and_fit

    # Last generation
    generation += 1
    bestindividual_fitness = max(individual_fitnesses)
    bestindividual_index = individual_fitnesses.index(bestindividual_fitness)
    print 'gen: ', generation, ' bestfitness: ', bestindividual_fitness, ' @idx: ', bestindividual_index
    # store the best individual's properties
    json_filename = fpath + 'snake_gen' + `generation` + '_bestindvidx' + `bestindividual_index` + '.json'
    store(json_filename, population[bestindividual_index], bestindividual_fitness, generation, bestindividual_index)
    # store generation statistics
    store_gen_stats(fpath + 'gen' + `generation` + 'stats.json' , generation, range(len(individual_fitnesses)), individual_fitnesses)
    # store all individuals of last generation
    for indvidualidx in range(len(population)):
        json_indivfilename = fpath + 'snake_gen' + `generation` + '_indvidx' + `indvidualidx` + '.json'
        store(json_indivfilename, population[indvidualidx], individual_fitnesses[indvidualidx], generation, indvidualidx)

    print '##end'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--createvis", action="store_true", help="Create visualization file of a stored individual. All files with the pattern snake*.json will be taken as input file.")
    parser.add_argument("--input_path", type=str, default="./", help="Input path")
    parser.add_argument("--output_path", type=str, default="./", help="Output path")
    parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
    parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
    parser.add_argument("--seed", type=int, default=0, help="Sets the seed for pseudo random generator.")
    args = parser.parse_args()

    # Seed only the evolutionary runs.
    random.seed(args.seed)

    print args.input_path
    if args.createvis:
        for file in os.listdir(args.input_path):
            if fnmatch.fnmatch(file, 'snake*.json'):
                filepath = args.input_path + file
                print 'process file' + filepath
                (genotype, fitness, generation, index) = load(filepath)
                print 'gen: ', generation, ' bestfitness: ', fitness, ' @idx: ', index, ' genotype: ', genotype
                odewrapper = odewrapper_t()
                # use input path also as output path for the visualization
                odewrapper.simulationrun(genotype, 'true', args.input_path + 'vis_'+ file)
    else:
        dosimulation(populationsize=args.pop_size, numofgenerations=args.gens, fpath=args.output_path)



if __name__ == '__main__':
    main()
