"""
    Wrapper to conduct the actual simulation of a worm robot.  Access through methods: 
    evaluate individual, and physics only validation.
"""

import sys, os, random
import itertools
import math

sys.path.insert(0, '../../')

from ODESystem import ODEManager
from ODESystem import Placement

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

from Robot import Sensors

import ode

man = 0
worm = 0

# Food collection information
collected_food = 0
food_map = {} # Track if food has been collected yet or not.

class GridPattern(object):
    """ Create an environment with balls uniformly distributed on a grid pattern. """

    def __init__(self,man,grid_dist=1,grid_width=40,grid_sector=1):
        """ Initialize the environment by adding them to the simulation. """

        self.man = man

        # Create the boundary walls.
        self.__create_walls(grid_width,grid_sector)

    def __create_walls(self,grid_width,grid_sector):
        """ Create the four walls bounding the environment. """

        global food_map

        # Add the four walls.
        self.man.create_box(100,[grid_width,2.0,0.5],[0,1,grid_width/2.0],terrain=True)
        self.man.create_box(101,[grid_width,2.0,0.5],[0,1,-grid_width/2.0],terrain=True)
        self.man.create_box(102,[0.5,2.0,grid_width],[grid_width/2.0,1,0],terrain=True)
        self.man.create_box(103,[0.5,2.0,grid_width],[-grid_width/2.0,1,0],terrain=True)

        # Add the balls.
        ball_num = 104
        for i in range(2,20,2):
            for j in range(2,20,2):
                #self.man.create_sphere(ball_num,0.0,0.5,[i,1,j],terrain=True)
                #self.man.create_sphere(ball_num+1,0.0,0.5,[i,1,-j],terrain=True)        
                #self.man.create_sphere(ball_num+2,0.0,0.5,[-i,1,j],terrain=True)
                #self.man.create_sphere(ball_num+3,0.0,0.5,[-i,1,-j],terrain=True)

                # Add the balls to the food map.
                #for k in range(0,4):
                #    food_map[ball_num+k] = 0

                #ball_num += 4

                self.man.create_sphere(ball_num,0.0,0.5,[i,1,grid_sector*j],terrain=True)

                # Add the ball to the food map.
                food_map[ball_num] = 0

                ball_num += 1

class Worm(object):
    """ Represent the worm robot. """

    def __init__(self,man,base_pos=[0,0,0],morphology_genome={},num_joints=2):
        """ Initialize the robot in the ODE environment. 

        Arguments:
            man: ODE Manager for the Physics Simulation
            base_pos: base position to start the robot from
            morphology_genome: dict of dicts which contain different parameters for the morphology (TODO)
            num_joints: number of joints in the worm robot
        """
        self.man = man
        self.body_keys = []

        # Sensors for robot.
        self.sensor = Sensors(man)

        self._num_joints = num_joints

        # Hardware Limits

        # Initialize the robot.
        self.__create_robot(base_pos=base_pos,morphology=morphology_genome)

    def __create_robot(self,base_pos=[0,0,0],morphology={}):
        """ Create the robot used in the experiment. 

        Arguments:
            base_pos: base position to start the robot from
            morphology: optional dict of dicts defining measurements for various parts of the robot.
        """

        # Constants for the different body parts.
        
        # Main Body
        WORM_LENGTH = 10.
        SEG_DIMS = morphology['seg_dims'] if 'seg_dims' in morphology else [WORM_LENGTH/(self._num_joints+1),.50,.50]
        
        BODY_POS  = [morphology['body_pos'][0]-base_pos[0],morphology['body_pos'][1]-base_pos[1],morphology['body_pos'][2]-base_pos[2]] \
            if 'body_pos' in morphology else [((SEG_DIMS[0]*self._num_joints)/2.)+base_pos[0],1+base_pos[1],0.0+base_pos[2]]
        BODY_MASS = morphology['body_mass'] if 'body_mass' in morphology else 10.

        # Joint Power for the legs
        JOINT_FORCE = [10000.,10000.]

        joint_range = math.radians(90.)

        # Keep track of body and joint numbers.
        b_num = 0
        j_num = 0

        # Create the Segments
        self.body_keys.append(self.man.create_box(b_num,SEG_DIMS,BODY_POS,density=BODY_MASS/(self._num_joints+1))) 
        BODY_POS[0] -= SEG_DIMS[0]
        b_num += 1

        for s in range(self._num_joints):
            self.body_keys.append(self.man.create_box(b_num,SEG_DIMS,BODY_POS,density=BODY_MASS/(self._num_joints+1))) 
            con_point = [BODY_POS[0]+SEG_DIMS[0]/2.,BODY_POS[1],BODY_POS[2]]
            self.man.create_universal(j_num, con_point,[b_num-1,b_num],axis1=[0,0,-1],axis2=[0,1,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=JOINT_FORCE[0],fmax2=JOINT_FORCE[1])
            BODY_POS[0] -= SEG_DIMS[0]
            b_num += 1
            j_num += 1

        # Add in joint positions sensors.
        self.sensor.register_joint_sensors([i for i in range(j_num)])

        # Add distance sensors to the head.  

        # Reset body position so we put sensors on the head.
        BODY_POS  = [morphology['body_pos'][0]-base_pos[0],morphology['body_pos'][1]-base_pos[1],morphology['body_pos'][2]-base_pos[2]] \
            if 'body_pos' in morphology else [((SEG_DIMS[0]*self._num_joints)/2.)+base_pos[0],1+base_pos[1],0.0+base_pos[2]]

        # Forward looking sensor
        sensor_pos = [BODY_POS[0]+SEG_DIMS[0]/2.,BODY_POS[1],BODY_POS[2]]
        self.man.create_ray(b_num,sensor_pos,[0,-90,0])
        self.man.create_hinge(j_num,sensor_pos,[0,b_num],axis=[0.,1.,0.],lims=[0.,0.],max_force=100.)
        
        # Register the sensor.
        self.sensor.add_dist_sensor(self.man.geoms[b_num],5.0)

        b_num += 1
        j_num += 1

        # Right looking sensor
        sensor_pos = [BODY_POS[0]+SEG_DIMS[2]/2.,BODY_POS[1],BODY_POS[2]+SEG_DIMS[2]/2.]
        self.man.create_ray(b_num,sensor_pos,[0,0,0])
        self.man.create_hinge(j_num,sensor_pos,[0,b_num],axis=[0.,1.,0.],lims=[0.,0.],max_force=100.)
        
        # Register the sensor.
        self.sensor.add_dist_sensor(self.man.geoms[b_num],15.0)

        b_num += 1
        j_num += 1

        # Left looking sensor
        sensor_pos = [BODY_POS[0]+SEG_DIMS[2]/2.,BODY_POS[1],BODY_POS[2]-SEG_DIMS[2]/2.]
        self.man.create_ray(b_num,sensor_pos,[0,90,0])
        self.man.create_hinge(j_num,sensor_pos,[0,b_num],axis=[0.,1.,0.],lims=[0.,0.],max_force=100.)
        
        # Register the sensor.
        self.sensor.add_dist_sensor(self.man.geoms[b_num],15.0)

        b_num += 1
        j_num += 1

    def actuate_joints_by_pos(self,positions):
        """ Actuate the joints of the worm to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """

        for i,p in enumerate(positions):
            self.man.actuate_universal(i,p[0],p[1])

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        #sensors = [i for i in self.sensor.get_touch_sensor_states()]
        sensors =  [i for i in self.sensor.get_joint_sensors()]
        sensors =  [sensors[i] for i in range(0,len(sensors),2)] # Remove sensors for vertical axis movement (Irrelevant).
        sensors += [i for i in self.get_dist_sensor_data()] # Add distance sensor data.
        return sensors

    def reset_touch_sensors(self):
        """ Reset the touch sensors. """
        self.sensor.clear_touching()

    def get_head_position(self):
        """ Return the position of the head of the worm. """
        return man.get_body_position(0)

    def get_avg_position(self):
        """ Get the average position of the robot. """
        avg_pos = [0,0,0]
        for i in range(self._num_joints+1):
            avg_pos = [sum(x) for x in zip(avg_pos,man.get_body_position(i))]

        # Average the positions.
        avg_pos = [i/(self._num_joints+1) for i in avg_pos]

        return avg_pos

    def get_ray_distance(self,geom):
        return self.sensor.get_raw_distance(self.man.geoms[self.man.get_geom_key(geom)])

    def get_scaled_ray_distance(self,geom):
        return self.sensor.get_scaled_distance(self.man.geoms[self.man.get_geom_key(geom)])    

    def get_dist_sensor_data(self):
        return self.sensor.get_scaled_distances()    

    def set_ray_distance(self,geom,dist):
        self.sensor.set_distance(self.man.geoms[self.man.get_geom_key(geom)],dist)

    def reset_distance_sensors(self):
        self.sensor.reset_distance_sensors()

    def clear_sensors(self,cur_time):
        """ Reset the sensor data. """
        self.sensor.clear_sensors(cur_time)

############################################################################################################        

def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 

############################################################################################################

def calc_distance():
    """ Calculate the distance between the worm's head and the ball. """
    global worm
    
    # Calculate the distance from the ball to the worm's head.
    return euclidean_distance(worm.get_head_position(),[0,1,0])

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """
    global man, worm, collected_food, food_map

    # Check to see if the two objects are connected.  Don't collide.
    if(man.are_connected(geom1.getBody(), geom2.getBody())):
        return

    # Check if the objects do collide
    contacts = man.generate_contacts(geom1, geom2)

    # Check to see if one of the two objects is a ray.
    if(type(geom1) == ode.GeomRay or type(geom2) == ode.GeomRay):
        ray = geom1 if type(geom1) == ode.GeomRay else geom2
        min_dist = worm.get_ray_distance(ray)
        for c in contacts:
            min_dist = euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) if \
                euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) < min_dist else min_dist
        worm.set_ray_distance(ray,min_dist)
    # Check to see if one of the two objects is a sphere (food)
    elif(type(geom1) == ode.GeomSphere or type(geom2) == ode.GeomSphere):
        sphere = geom1 if type(geom1) == ode.GeomSphere else geom2
        sphere_key = man.get_terrain_geom_key(sphere)

        # Flag for collecting a piece of food.
        if food_map[sphere_key] == 0:
            food_map[sphere_key] = 1
            man.set_current_alpha(sphere_key,0.0)
            man.disable_terrain_geom(sphere_key)
            collected_food += 1
        return
    else:
        # Create contact joints
        man.world,man.contactgroup = args
        for c in contacts:
            c.setBounce(0.2)
            c.setMu(5000)
            j = man.create_contact_joint(c)
            j.attach(geom1.getBody(), geom2.getBody())

class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.02, n=4, con_type="single",hyperNEAT=False,substrate=False,periodic=True, num_joints=1,aquatic=False,validator_condition=0):
        """ Initialize the simulation class. """

        # Whether or not we simulate in the aquatic environment or not.
        self.aquatic = aquatic

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        # Settings for the simulation.
        self.log_frames = log_frames
        self.run_num = run_num
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        self._num_joints = num_joints

        self.current_network = 0

        self.hyperNEAT = True if hyperNEAT else False
        self.substrate = substrate

        # Whether we include a periodic oscillating input signal.
        self.periodic = periodic

        # Check for explosions
        self.exploded = False

        self.cumulative_fitness = 0

        # Parameter to determine where the ball should be placed for validation.
        self.validator_condition = validator_condition

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """
        global man, worm

        if self.current_network:
            inputs = []
            if self.periodic:
                inputs.append(math.sin(2.0*math.pi*(self.elapsed_time)))
            inputs += worm.get_sensor_states()                
            inputs.append(1.0) # Bias node


            # Send inputs to the ANN and get the outputs.
            self.current_network.Input(inputs) 
            self.current_network.Activate()
            nn_activations = self.current_network.Output()

            activations = [[0,nn_activations[i]] for i in range(0,len(nn_activations))]

            worm.actuate_joints_by_pos(positions=activations)

        else:   
            # Actuate all joints at a steady speed.
            time_offset = 0.15
            positions = [([0.,math.sin(2.0*math.pi*(self.elapsed_time-(time_offset*i)))]) for i in xrange(worm._num_joints)]

            worm.actuate_joints_by_pos(positions=positions)
        
        if self.aquatic:
            man.sim_fluid_dynamics()

    def reset_simulation(self):
        """ Reset the simulation. """
        global man, collected_food, food_map
        
        man.delete_joints()
        man.delete_bodies()
        self.elapsed_time = 0.
        self.exploded = False
        collected_food = 0
        food_map = {}

    def simulate(self):
        """ Perform physics simulation. """
        global worm, man, collected_food, food_map

        if self.elapsed_time < self.eval_time:

            self.update_callback()

            # Check to see about explosions.
            ang_vel = man.bodies[0].getAngularVel()
            if self.elapsed_time % 1.0 == 0.0 and (man.get_body_position(0)[1] > 4. or math.fabs(ang_vel[0]) > 10 \
                or math.fabs(ang_vel[1]) > 10 or math.fabs(ang_vel[2]) > 10):
                self.exploded = True

        if self.elapsed_time >= self.eval_time:
            if not self.exploded:
                fit = collected_food
                food_map = {}
            else:
                fit = 0.00001
            self.reset_simulation()
            return False, fit

        # Reset sensors.
        worm.clear_sensors(self.elapsed_time)
        
        return True, 0 

    def init_simulation(self):
        """ Initialize the objects in a simulation. """
        global man, worm

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num, gravity=grav, fluid_dynamics=fluid_dyn, eval_time=self.eval_time)

        # Create the worm.
        worm = Worm(man=man, num_joints=self._num_joints)

        retcond = 99 # init with some random number
        # Create the environment
        if self.validator_condition == 0:
            choice = random.choice([-1,1])
            environment = GridPattern(man,grid_sector=choice)
            print("Choice: "+str(choice))
            retcond = choice
        else:
            environment = GridPattern(man,grid_sector=self.validator_condition)
            print("Validator Condition: "+str(self.validator_condition))
            retcond = self.validator_condition

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup()

        return retcond

    def sim_loop(self):
        """ Loop through the simulation. """

        global man, worm

        cond = self.init_simulation()

        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            man.step(near_callback, self.n)
            self.elapsed_time += self.dt
            go_on, fit = self.simulate()

        return (fit, cond)

    def physics_only_simulation(self):
       
        fit,cond = self.sim_loop()

        return (fit,cond)

    def evaluate_individual(self,genome):
        """ Evaluate an individual solution. 

        Args:
            genome: genome of the individual to evaluate

        Returns:
            fitness value of the individual
        """

        # Load in the ANN from the population
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        # Conduct the evaluation
        fit,cond = self.physics_only_simulation()

        return (fit,cond)

    def evaluate_individual_wrap(self,com_args):
        """ Decompress the arguments provided to multiprocessing.  

        Args:
            com_args: compressed arguments for evaluate_individual()
        """

        return self.evaluate_individual(*com_args)

    def validator(self,NEAT_file):
        """ Validate a single run. 

        Args:
            NEAT_File: file for the NEAT genome
        """

        # Load in the best performing NEAT genome
        genome = NEAT.Genome(NEAT_file)
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        fit,cond = self.physics_only_simulation()

        print(fit)

    def debug_validator(self):
        """ Validate a single run. """

        fit = self.physics_only_simulation()

        print(fit)
