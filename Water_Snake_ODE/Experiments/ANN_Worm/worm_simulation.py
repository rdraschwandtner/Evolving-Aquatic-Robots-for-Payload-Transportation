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
        BODY_MASS = morphology['body_mass'] if 'body_mass' in morphology else 5.

        # Joint Power for the legs
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

        # Add in information about the feet.
        self.sensor.add_touch_sensor([i for i in range(b_num)])

        # Add in joint positions sensors.
        self.sensor.register_joint_sensors([i for i in range(j_num)])

    def actuate_joints_by_pos(self,positions):
        """ Actuate the joints of the worm to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """

        for i,p in enumerate(positions):
            self.man.actuate_universal(i,p[0],p[1])

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        sensors = [i for i in self.sensor.get_touch_sensor_states()]
        sensors += [i for i in self.sensor.get_joint_sensors()]
        return sensors

    def reset_touch_sensors(self):
        """ Reset the touch sensors. """
        self.sensor.clear_touching()

    def get_avg_position(self):
        """ Get the average position of the robot. """
        avg_pos = [0,0,0]
        for i in range(self._num_joints+1):
            avg_pos = [sum(x) for x in zip(avg_pos,man.get_body_position(i))]

        # Average the positions.
        avg_pos = [i/(self._num_joints+1) for i in avg_pos]

        return avg_pos

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

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check to see if the two objects are connected.  Don't collide.
    if(man.are_connected(geom1.getBody(), geom2.getBody())):
        return

    # Check if the objects do collide
    contacts = man.generate_contacts(geom1, geom2)

    # Create contact joints
    man.world,man.contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(5000)
        j = man.create_contact_joint(c)
        j.attach(geom1.getBody(), geom2.getBody())

class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.02, n=4, con_type="single",hyperNEAT=False,substrate=False,periodic=True, num_joints=1,aquatic=False):
        """ Initialize the simulation class. """
        global simulate

        # Whether or not we simulate in the aquatic environment or not.
        self.aquatic = aquatic

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        man = ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, run_num=run_num, gravity=grav, fluid_dynamics=fluid_dyn)

        # Settings for the simulation.
        self.log_frames = log_frames
        self.run_num = run_num
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        self._num_joints = num_joints

        # Worm Specific Setup
        worm = ""

        self.current_network = 0

        self.hyperNEAT = True if hyperNEAT else False
        self.substrate = substrate

        # Whether we include a periodic oscillating input signal.
        self.periodic = periodic

        # Check for explosions
        self.exploded = False

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """
        
        if self.current_network:
            inputs = []
            if self.periodic:
                inputs.append(math.sin(2.*math.pi*(self.elapsed_time)))
            inputs += worm.get_sensor_states()
            inputs.append(1.0) # Bias node

            # Send inputs to the ANN and get the outputs.
            self.current_network.Input(inputs) 
            self.current_network.Activate()
            nn_activations = self.current_network.Output()

            activations = [[nn_activations[i],nn_activations[i+1]] for i in range(0,len(nn_activations),2)]

            worm.actuate_joints_by_pos(positions=activations)
        else:   
            # Actuate all joints at a steady speed.
            time_offset = 0.55
            positions = [([0.,0.15+0.5*math.sin(0.5*math.pi*(self.elapsed_time-(time_offset*i)))] if self.elapsed_time > time_offset*i else [0,0]) for i in xrange(worm._num_joints)]

            worm.actuate_joints_by_pos(positions=positions)

        if self.aquatic:
            man.sim_fluid_dynamics()

    def reset_simulation(self):
        """ Reset the simulation. """
        
        man.delete_joints()
        man.delete_bodies()
        self.elapsed_time = 0.
        self.exploded = False

    def simulate(self):
        """ Perform physics simulation. """

        if self.elapsed_time < self.eval_time: 
            self.update_callback()

            # Check to see about explosions.
            ang_vel = man.bodies[0].getAngularVel()
            if self.elapsed_time % 1.0 == 0.0 and (man.get_body_position(0)[1] > 4. or math.fabs(ang_vel[0]) > 10 \
                or math.fabs(ang_vel[1]) > 10 or math.fabs(ang_vel[2]) > 10):
                self.exploded = True

        if self.elapsed_time >= self.eval_time:
            if not self.exploded:
                # Calculate the average position of the robot.
                fit = euclidean_distance(worm.get_avg_position(),[0,0,0])
            else:
                fit = 0.00001
            self.reset_simulation()
            return False, fit
        return True, 0 

    def physics_only_simulation_validator(self):
        """ Conduct a physics only simulation while logging the body data.

        We then use this information to play back the simulation on a different machine.
        
        Args:
            output_path: path to write the log file to.
            ind_num: individual number
        """
      
        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            man.step(near_callback, self.n)
            self.elapsed_time += self.dt
            go_on, fit = self.simulate()

        print(fit)

    def physics_only_simulation(self):
       
        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            man.step(near_callback, self.n)
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
        global man,current_network,worm

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, gravity=grav, fluid_dynamics=fluid_dyn)

        # Initialize the worm
        worm = Worm(man=man, num_joints=self._num_joints)

        # Load in the ANN from the population
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        # Conduct the evaluation
        fit = self.physics_only_simulation() 

        return fit

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
        global man, worm

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num, gravity=grav, fluid_dynamics=fluid_dyn)

        # Initialize the worm
        worm = Worm(man=man, num_joints=self._num_joints)

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        # Load in the best performing NEAT genome
        genome = NEAT.Genome(NEAT_file)
        self.current_network = NEAT.NeuralNetwork()
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        fit = self.physics_only_simulation_validator()

        print(fit)

    def debug_validator(self):
        """ Validate a single run. """
        global man, worm

        if self.aquatic:
            grav = 0
            fluid_dyn = 1
        else:
            grav = -9.81
            fluid_dyn = 0

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num, gravity=grav, fluid_dynamics=fluid_dyn)

        # Initialize the worm
        worm = Worm(man=man, num_joints=self._num_joints)

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        fit = self.physics_only_simulation_validator()

        print(fit)
