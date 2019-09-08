import GlobalVarWorkaround

from Robot import Sensors
import math
import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog
from ODESystem import ODEManager
import GlobalVarWorkaround

import ode

def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 

def isgeom_worm(geomid, worm):
    return geomid in worm.body_keys

def isgeom_wall(geomid):
    return True if geomid == 100 or geomid == 101 or geomid == 102 or geomid == 103 else False

def isgeom_sphere(geomid):
    return True if geomid > 103 else False

def isfood_eaten(sphereid, foodenv):
    assert sphereid in foodenv.sphere_exists, 'given id is not in the sphere_exists array'
    return not foodenv.sphere_exists[sphereid] # invert(exists)

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check to see if the two objects are connected.  Don't collide.
    if(GlobalVarWorkaround.man.are_connected(geom1.getBody(), geom2.getBody())):
        return

    # Check walls -> don't collide
    if isgeom_wall(geom1.getBody()) or isgeom_wall(geom2.getBody()):
        return

    geom1key = GlobalVarWorkaround.man.get_geom_key(geom1)
    geom2key = GlobalVarWorkaround.man.get_geom_key(geom2)
    # Check if sphere has already been eaten -> don't collide
    if isgeom_sphere(geom1key) and isfood_eaten(geom1key, GlobalVarWorkaround.foodenv):
        return

    if isgeom_sphere(geom2key) and isfood_eaten(geom2key, GlobalVarWorkaround.foodenv):
        return

    # Check if one of the objects is food sphere. Don't collide.
    #if geom1.getBody() > 104 or geom2.getBody() > 104:
    #    return

    # Check if the objects do collide
    contacts = GlobalVarWorkaround.man.generate_contacts(geom1, geom2)

    # Create contact joints
    GlobalVarWorkaround.man.world,GlobalVarWorkaround.man.contactgroup = args

    # Check to see if one of the two objects is a ray.
    if(type(geom1) == ode.GeomRay or type(geom2) == ode.GeomRay):
        ray = geom1 if type(geom1) == ode.GeomRay else geom2
        min_dist = GlobalVarWorkaround.worm.get_ray_distance(ray)
        for c in contacts:
            min_dist = euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) if \
                euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) < min_dist else min_dist
        GlobalVarWorkaround.worm.set_ray_distance(ray,min_dist)

    # worm eats the sphere on touch
    elif isgeom_sphere(geom1key) and isgeom_worm(geom2key, GlobalVarWorkaround.worm):
        GlobalVarWorkaround.foodenv.sphere_exists[geom1key] = False # food has is eaten
        return
    elif isgeom_sphere(geom2key) and isgeom_worm(geom1key, GlobalVarWorkaround.worm):
        GlobalVarWorkaround.foodenv.sphere_exists[geom2key] = False # food has is eaten
        return

    else:
        for c in contacts:
            c.setBounce(0.2)
            c.setMu(5000)
            j = GlobalVarWorkaround.man.create_contact_joint(c)
            j.attach(geom1.getBody(), geom2.getBody())
        




class FoodEnvironment(object):
    def __init__(self,man,square_size,eucquadrant):
        self.man = man
        self.__create_walls(square_size)
        self.__create_spheres(square_size,eucquadrant)
        #self.__create_spheres(square_size,1)
        #self.__create_spheres(square_size,2)
        #self.__create_spheres(square_size,3)
        #self.__create_spheres(square_size,4)

    def __create_walls(self,square_size):
        # 100, 101, 102, 103 wall: ids
        self.man.create_box(100,[square_size,2.0,0.5],[0,1,square_size/2.0])
        self.man.create_box(101,[square_size,2.0,0.5],[0,1,-square_size/2.0])
        self.man.create_box(102,[0.5,2.0,square_size-0.5],[square_size/2.0,1,0])
        self.man.create_box(103,[0.5,2.0,square_size-0.5],[-square_size/2.0,1,0])

    def __create_spheres(self,square_size,quadrant):
        '''square_size.. size of the full square
           quadrant.. number of the cartesian quadrant (https://en.wikipedia.org/wiki/Quadrant_(plane_geometry)) '''
        assert quadrant > 0 and quadrant <= 4, "not euclidean quadrant"

        # 1: positive X, positive Z
        # 2: negative X, positive Z
        # 3: negative X, negative Z
        # 4: positive X, negative Z
        x_multiplier = 0
        z_multiplier = 0
        if quadrant == 1:
            x_multiplier = +1
            z_multiplier = +1
        elif quadrant == 2:
            x_multiplier = -1
            z_multiplier = +1
        elif quadrant == 3:
            x_multiplier = -1
            z_multiplier = -1
        elif quadrant == 4:
            x_multiplier = +1
            z_multiplier = -1

        assert square_size % 2 == 0, "square_size must be divideable by 2"
        sphereid = 104
        self.sphere_exists = {}
        for x_pos in range(0, int(square_size/2-round(0.5)), 2):
            for z_pos in range(0, int(square_size/2-round(0.5)), 2):
                self.man.create_sphere(sphereid,0.000001,0.5,[x_pos*x_multiplier,1,z_pos*z_multiplier])
                self.sphere_exists[sphereid] = True # food_map
                sphereid += 1
        sphereid -= 1



class Worm(object):

    def __init__(self,man,morphology_genome={},base_pos=[0,0,0],num_joints=2, logging=False, log_path='./', logfileprefix=''):
        """ Initialize the robot in the ODE environment. 

        Arguments:
            man: ODE Manager for the Physics Simulation
            base_pos: base position to start the robot from
            morphology_genome: dict of dicts which contain different parameters for the morphology (TODO)
            num_joints: number of joints in the worm robot
        """
        self.man = man
        self.body_keys = []

        self.logging = logging
        self.logfileprefix = logfileprefix

        # Sensors for robot.
        self.sensor = Sensors(man=man, logging=logging, log_path=log_path)

        self._num_joints = num_joints

        # morphology of the robot
        # first element in the list is the
        self.freq = morphology_genome[0]
        assert self.freq >= 0, "frequency shall not be negative!"

        # Initialize the robot.
        self.__create_robot(base_pos=base_pos)

    def __create_robot(self,base_pos=[0,0,0]):
        """ Create the robot used in the experiment. 

        Arguments:
            base_pos: base position to start the robot from
            morphology: optional dict of dicts defining measurements for various parts of the robot.
        """

        # Constants for the different body parts.
        
        # Main Body
        WORM_LENGTH = 10.
        #SEG_DIMS = morphology['seg_dims'] if 'seg_dims' in morphology else [WORM_LENGTH/(self._num_joints+1),.50,.50]
        # calculate the dimensions of a segment
        SEG_DIMS = [WORM_LENGTH/(self._num_joints+1)    # x -length
                    ,.50                                # y -length
                    ,.50]                               # z -length
        
        #BODY_POS  = [morphology['body_pos'][0]-base_pos[0],morphology['body_pos'][1]-base_pos[1],morphology['body_pos'][2]-base_pos[2]] \
        #    if 'body_pos' in morphology else [((SEG_DIMS[0]*self._num_joints)/2.)+base_pos[0],1+base_pos[1],0.0+base_pos[2]]
        # calculate the starting point of the body in the simulation environment
        BODY_POS  = [((SEG_DIMS[0]*self._num_joints)/2.)+base_pos[0]    # align the body in x - direction, middle of the body is at base position
                     ,1+base_pos[1]                                     # lift the body in +y - direction by 1
                     ,0.0+base_pos[2]]                                  # set the middle of the body's z starting position to the base position 
        #BODY_MASS = morphology['body_mass'] if 'body_mass' in morphology else 5.
        BODY_MASS = 50 # hardcoded body mass value, that worked in jared's simulations

        # joint maximum forces, todo think about the values
        JOINT_FORCE = [100.,100.]

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
            print(b_num,j_num)

        # Add in joint positions sensors.
        self.sensor.register_joint_sensors([i for i in range(j_num)])

        # Add distance sensors to the head.
        # taken from Jared's code
        # Reset body position so we put sensors on the head.
        BODY_POS  = [((SEG_DIMS[0]*self._num_joints)/2.)+base_pos[0]
                     ,1+base_pos[1]
                     ,0.0+base_pos[2]]

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

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        sensors = [i for i in self.sensor.get_joint_sensors()]
        sensors =  [sensors[i] for i in range(0,len(sensors),2)] # Remove sensors for vertical axis movement (Irrelevant).
        sensors += [i for i in self.get_dist_sensor_data()] # Add distance sensor data.s
        return sensors

    def actuate_joints_by_pos(self,positions):
        """ Actuate the joints of the worm to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """
        # actuate all joint according to the expected position
        for i,p in enumerate(positions):
            self.man.actuate_universal(i,0,p) # joint1 = 0, because we use only one joint here -> movement in z axis

    def get_avg_position(self):
        """ Get the average position of the robot. """
        avg_pos = [0,0,0]
        for i in range(self._num_joints+1):
            avg_pos = [sum(x) for x in zip(avg_pos,self.man.get_body_position(i))]

        # Average the positions.
        avg_pos = [i/(self._num_joints+1) for i in avg_pos]

        return avg_pos

    def clear_sensors(self,cur_time):
        """ Clears all sensor values
        """
        self.sensor.clear_sensors(cur_time)

    def log_sensor_data(self):
        self.sensor.dump_sensor_data(self.logfileprefix)

    def sensor_step(self, cur_time):
        self.sensor.step(cur_time)

    def reset_touch_sensors(self):
        """ Reset the touch sensors. """
        self.sensor.clear_touching()

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

class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, eval_time, dt, n, man, worm):
        """ Initialize the simulation class. """

        # aquatic simulation
        self.aquatic = True

        self.man = man
        self.worm = worm
        #man = ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, run_num=run_num, gravity=0, fluid_dynamics=1)

        # Settings for the simulation.
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        self._num_joints = worm._num_joints

        # init worm object - one worm per simulation
        worm = ""

        # init current network
        self.current_network = 0

        # Use an periodic (oscilating) input signal
        self.periodic = True

        # Check for explosions
        self.exploded = False

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """
        
        assert self.current_network != 0, "ANN is not set!"
        inputs = []
           
        # periodic input signal as first input signal
        inputs.append(1.*math.sin(2.*math.pi*self.worm.freq*self.elapsed_time+0.0))
        #print "input sig: " + `(1.*math.sin(2.*math.pi*self.worm.freq*self.elapsed_time+0.0))`
        #inputs.append(math.sin(2.*math.pi*(self.elapsed_time)))
        # use sensors as input
        inputs += self.worm.get_sensor_states()
        # explicitly add bias node
        inputs.append(1.0)
        #if self.man.log_data:
        #    print inputs



        # Send inputs to the ANN and get the outputs.
        self.current_network.Input(inputs) 
        self.current_network.Activate()
        # the NN sets the output nodes to a value that represents the expected
        # position of the joint with 1 dof
        nn_outputnodevals = self.current_network.Output()

        self.worm.actuate_joints_by_pos(positions=nn_outputnodevals)

        self.man.sim_fluid_dynamics()

    def reset_simulation(self):
        """ Reset the simulation. """
        
        self.man.delete_joints()
        self.man.delete_bodies()
        self.elapsed_time = 0.
        self.exploded = False

    def simulate(self):
        """ Perform physics simulation. """
        what = True
        fit = 0

        if self.elapsed_time < self.eval_time: 
            self.update_callback()

        if self.elapsed_time >= self.eval_time:
            what = False
            endpos = self.worm.get_avg_position()
            # force worm to swim in one direction
            # multiply the euclidean distance with the -1 if the endpos lies in -x area
            for sphexist in GlobalVarWorkaround.foodenv.sphere_exists:
                if GlobalVarWorkaround.foodenv.sphere_exists[sphexist] == False: # sphere has been eaten
                    fit += 1
            if self.man.log_data:
                self.worm.log_sensor_data()
            self.reset_simulation()

        if self.worm.logging:
            self.worm.sensor_step(self.elapsed_time) 

        self.worm.clear_sensors(self.elapsed_time)
        return what, fit

    def physics_only_simulation(self):
       
        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            self.man.step(near_callback, self.n)
            self.elapsed_time += self.dt
            go_on, fit = self.simulate()


        return fit

    def evaluate_individual(self,genome):
        """ Evaluate an individual solution. 

        Args:
            genome: genome of the individual to evaluate

        Returns:
            fitness value of the individual
        """

        # Load in the ANN from the population
        self.current_network = NEAT.NeuralNetwork()
        # ANN part of the genome
        genome[0].BuildPhenotype(self.current_network)

        #self.freq = genome[1][0]

        # Conduct the evaluation
        fit = self.physics_only_simulation() 

        return fit

def validator(NEAT_file,Morph_file, log_frames, run_num, eval_time, dt, n, num_joints, output_path, gennumber, eucquad):

    genome = [0,{}]

    # Load in the best performing NEAT genome
    genome[0] = NEAT.Genome(NEAT_file)

    # Load the morphology
    with open(Morph_file,"r") as f:
        line = f.readline()
        line = line.strip().split(',')
        genome[1][0] = float(line[0]) # read frequency
   
    GlobalVarWorkaround.man = ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)
    GlobalVarWorkaround.worm = Worm(man=GlobalVarWorkaround.man, morphology_genome=genome[1], num_joints=num_joints, logging=log_frames, log_path=output_path, logfileprefix='rep' + str(run_num)+ '_gen'+str(gennumber))
    GlobalVarWorkaround.foodenv = FoodEnvironment(man=GlobalVarWorkaround.man,square_size=40, eucquadrant=eucquad)

    # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
    # Must be placed after creating the quadruped.
    if log_frames:
        GlobalVarWorkaround.man.log_world_setup()

    simulation = Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm)
    #simulation.current_network = NEAT.NeuralNetwork()
    #genome[0].BuildPhenotype(simulation.current_network)


    fit = simulation.evaluate_individual(genome)
    print (fit)