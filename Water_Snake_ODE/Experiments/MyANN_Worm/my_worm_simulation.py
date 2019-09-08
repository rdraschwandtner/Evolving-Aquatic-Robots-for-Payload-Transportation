import GlobalVarWorkaround

from Robot import Sensors
import math
import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog
from ODESystem import ODEManager
import GlobalVarWorkaround


def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 


# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check to see if the two objects are connected.  Don't collide.
    if(GlobalVarWorkaround.man.are_connected(geom1.getBody(), geom2.getBody())):
        return

    # Check if the objects do collide
    contacts = GlobalVarWorkaround.man.generate_contacts(geom1, geom2)

    # Create contact joints
    GlobalVarWorkaround.man.world,GlobalVarWorkaround.man.contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(5000)
        j = GlobalVarWorkaround.man.create_contact_joint(c)
        j.attach(geom1.getBody(), geom2.getBody())

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

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        sensors = [i for i in self.sensor.get_joint_sensors()]
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

    def clear_sensors(self):
        """ Clears all sensor values
        """
        self.sensor.clear_sensors()

    def log_sensor_data(self):
        self.sensor.dump_sensor_data(self.logfileprefix)

    def sensor_step(self, cur_time):
        self.sensor.step(cur_time)


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
            fit = math.copysign(1, endpos[0]) * euclidean_distance(endpos,[0,0,0])
            if self.man.log_data:
                self.worm.log_sensor_data()
            self.reset_simulation()

        if self.worm.logging:
            self.worm.sensor_step(self.elapsed_time) 

        self.worm.clear_sensors()
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

def validator(NEAT_file,Morph_file, log_frames, run_num, eval_time, dt, n, num_joints, output_path, gennumber):

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

    # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
    # Must be placed after creating the quadruped.
    if log_frames:
        GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

    simulation = Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm)
    #simulation.current_network = NEAT.NeuralNetwork()
    #genome[0].BuildPhenotype(simulation.current_network)


    fit = simulation.evaluate_individual(genome)
    print (fit)