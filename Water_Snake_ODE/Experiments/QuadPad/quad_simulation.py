"""
    Wrapper to conduct the actual simulation of a quadruped robot.  Access through methods: 
    evaluate individual, and physics only validation.

    Quadruped has three body segments with flexible joints in between each.
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
quadruped = 0

class Quadruped(object):
    """ Represent the quadruped robot. """

    def __init__(self,man,base_pos=[0,0,0],morphology_genome={}):
        """ Initialize the robot in the ODE environment. 

        Arguments:
            man: ODE Manager for the Physics Simulation
            base_pos: base position to start the robot from
            morphology_genome: dict of dicts which contain different parameters for the morphology (TODO)
        """
        self.man = man
        self.body_keys = []

        # Sensors for robot.
        self.sensor = Sensors(man)

        # Hardware Limits
        self.joint_max_power = 80.

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
        BODY_DIMS = morphology['body_dims'] if 'body_dims' in morphology else [3.0/3.,0.5,1.0]
        BODY_POS  = [
            [ 1.0+base_pos[0],1.60+base_pos[1],0.0+base_pos[2]],
            [ 0.0+base_pos[0],1.60+base_pos[1],0.0+base_pos[2]],
            [-1.0+base_pos[0],1.60+base_pos[1],0.0+base_pos[2]]
            ]
        BODY_MASS = morphology['body_mass'] if 'body_mass' in morphology else 10./3.

        # Upper Legs
        F_UPP_DIMS = R_UPP_DIMS = morphology['u_l_dims'] if 'u_l_dims' in morphology else [0.5,.10]
        F_UPP_MASS = R_UPP_MASS = morphology['u_l_mass'] if 'u_l_mass' in morphology else 2.

        R_UPP_DIMS = [.5,.10]
        R_UPP_MASS = 2.

        # Custom Measurements Per Leg Group

        # Front Upper Legs
        if 'f_u_dims' in morphology:
            F_UPP_DIMS = morphology['f_u_dims']
        if 'f_u_mass' in morphology:
            F_UPP_MASS = morphology['f_u_mass']

        # Rear Upper Legs
        if 'r_u_dims' in morphology:
            R_UPP_DIMS = morphology['r_u_dims']
        if 'r_u_mass' in morphology:
            R_UPP_MASS = morphology['r_u_mass']

        # Middle Legs
        F_MID_DIMS = R_MID_DIMS = morphology['l_m_dims'] if 'l_m_dims' in morphology else [.75,.10]
        F_MID_MASS = R_MID_MASS = morphology['l_m_mass'] if 'l_m_mass' in morphology else 1.

        # Custom Measurements Per Leg Group

        # Front Lower Legs
        if 'f_m_dims' in morphology:
            F_MID_DIMS = morphology['f_m_dims']
        if 'f_m_mass' in morphology:
            F_MID_MASS = morphology['f_m_mass']

        # Rear Lower Legs
        if 'r_m_dims' in morphology:
            R_MID_DIMS = morphology['r_m_dims']
        if 'r_m_mass' in morphology:
            R_MID_MASS = morphology['r_m_mass']

        # Joint Power for the legs
        F_UPP_FORCE = R_UPP_FORCE = F_MID_FORCE = R_MID_FORCE = F_LOW_FORCE = R_LOW_FORCE = [self.joint_max_power,self.joint_max_power]

        joint_range = math.radians(120.)

        # Keep track of body and joint numbers.
        b_num = 0
        j_num = 0

        # Create the Main Body (Front)
        self.body_keys.append(self.man.create_box(b_num,BODY_DIMS,BODY_POS[0],density=BODY_MASS)) 
        b_num += 1

        self.body_keys.append(self.man.create_box(b_num,BODY_DIMS,BODY_POS[1],density=BODY_MASS)) 
        con_point = [0.5,BODY_POS[1][1],BODY_POS[1][2]]
        self.man.create_flexible_universal(j_num,con_point,[b_num-1,b_num],axis1=[0,0,-1],axis2=[0,1,0],
            erp1=morphology['erp'],erp2=morphology['erp'],cfm1=morphology['cfm'],cfm2=morphology['cfm'])
        b_num += 1
        j_num += 1

        self.body_keys.append(self.man.create_box(b_num,BODY_DIMS,BODY_POS[2],density=BODY_MASS)) 
        con_point = [-0.5,BODY_POS[2][1],BODY_POS[2][2]]
        self.man.create_flexible_universal(j_num,con_point,[b_num-1,b_num],axis1=[0,0,-1],axis2=[0,1,0],
            erp1=morphology['erp'],erp2=morphology['erp'],cfm1=morphology['cfm'],cfm2=morphology['cfm'])
        b_num += 1
        j_num += 1

        # Create the Right Upper Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        #rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [(BODY_DIMS[0]*3.)/2.*.85,BODY_POS[0][1],BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [(BODY_DIMS[0]*3.)/2.*.85,BODY_POS[0][1],-(BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05)]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Upper Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-(BODY_DIMS[0]*3.)/2.*.8,BODY_POS[2][1],BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[2,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-(BODY_DIMS[0]*3.)/2.*.8,BODY_POS[2][1],-(BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05)]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[2,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Turn feedback on for the joints.
        for i in range(8):
            self.man.joints[i+2].setFeedback()

        # Add in information about the feet.
        self.sensor.add_touch_sensor([7,8,9,10])

        # Add in joint positions sensors.
        self.sensor.register_joint_sensors([2,3,4,5,6,7,8,9])

    def get_joint_feedback(self):
        """ Get the feedback from the joints. """

        joint_feedback = []
        for i in range(8):
            joint_feedback.append(self.man.joints[i+2].getFeedback())
        return joint_feedback

    def set_joint_power(self,power=[15. for i in xrange(8)]):
        """ Set the power of the joints. 

        Arguments:
            power: joint power for each of the joints to set.
        """
        for i,p in enumerate(power):
            if p < -0.6:
                p = 0
            else:
                p = (p+1.)/2.
            self.man.set_uni_joint_force(i+2,self.joint_max_power*p)    

    def actuate_joints_by_pos(self,positions=[[0.,0.] for i in xrange(8)]):
        """ Actuate the joints of the quadruped to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """

        for i,p in enumerate(positions):
            self.man.actuate_universal(i+2,p[0],p[1])

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        sensors = [i for i in self.sensor.get_touch_sensor_states()]
        sensors += [i for i in self.sensor.get_joint_sensors()]
        return sensors

    def reset_touch_sensors(self):
        """ Reset the touch sensors. """
        self.sensor.clear_touching()

    def get_max_force(self,jnum):
        return self.man.joints[2].getParam(ode.ParamFMax)

############################################################################################################        

def vector_3d_magnitude(vec):
    """ Return the magnitude of a vector in 3D. """

    vec = [vec[0],vec[1],vec[2]]

    # Filter out low values in the vector.
    for i in range(len(vec)):
        if math.fabs(vec[i]) < 0.0001:
            vec[i] = math.copysign(0.0001,vec[i])
        elif math.fabs(vec[i]) > 1000.:
            vec[i] = math.copysign(1000.,vec[i])

    if sum(vec) < 0.01 or (math.isnan(vec[0]) or math.isnan(vec[1]) or math.isnan(vec[2])):
        return 0
    force_mag = 0    
    try:
        force_mag = math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
    except OverflowError as e:
        print("\n\n\n")
        print("OverflowError: ",e,vec)
        print("\n\n\n")
    return force_mag

def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 

def distance_per_unit_of_power(p1,p2,forces):
    """ Calculate the distance per unit of power. """
    distance = math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2)

    forces = [i for f in forces for i in f]
    forces = zip(*forces)
    total_power = sum([abs(i) for i in forces[0]]) + sum([abs(j) for j in forces[1]])

    return (distance)/total_power

def norm_dist_efficiency_fitness(dist,eff):
    """ Calculate the normalized fitness including distance travelled and mechanical efficiency.  """
    return (dist/30.) + (eff/0.5)

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

    # Don't collide body parts.
    if(geom1 != man.floor and geom2 != man.floor):
        return        

    # Check to see if one of the objects is a foot sensor and the other
    # is the ground.
    if ((geom1 == man.floor and quadruped.sensor.is_touch_sensor(man.get_body_key(geom2.getBody()))) or
        (quadruped.sensor.is_touch_sensor(man.get_body_key(geom1.getBody())) and geom2 == man.floor)):
        body_id = -1
        if geom1 == man.floor:
            body_id = man.get_body_key(geom2.getBody())
        else:
            body_id = man.get_body_key(geom1.getBody())
        quadruped.sensor.activate_touch_sensor(body_id)

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

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.02, n=4, con_type="single",hyperNEAT=False,substrate=False,periodic=True,dir_change_limit=1000):
        """ Initialize the simulation class. """
        global simulate

        man = 0 #ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, run_num=run_num,erp=0.5,cfm=1E-2)

        # Settings for the simulation.
        self.log_frames = log_frames
        self.run_num = run_num
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        # Quadruped Specific Setup
        quadruped = 0 #Quadruped(man=man)

        self.current_network = 0

        self.hyperNEAT = True if hyperNEAT else False
        self.substrate = substrate

        # Whether we include a periodic oscillating input signal.
        self.periodic = periodic

        # Explosion condition.
        self.exploded = False

        # Log ANN activations
        self.ann_activations = []
        self.joint_feedback = []

        # If validating.
        self.validating = False

        self.forces = []

        # Track the number of directional changes in the joints.
        self.ann_acts = []
        self.dir_changes = 0
        self.dir_change_limit = dir_change_limit

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """
        
        if self.current_network:
            inputs = []
            if self.periodic:
                inputs.append(math.sin(2.*math.pi*(self.elapsed_time)))
            inputs += quadruped.get_sensor_states()
            inputs.append(1.0) # Bias node

            # Send inputs to the ANN and get the outputs.
            self.current_network.Input(inputs) 
            self.current_network.Activate()
            nn_activations = self.current_network.Output()

            # Check the ann for direction changes.
            self.ann_acts.append(nn_activations)
            ann_len = len(self.ann_acts)
            if ann_len > 3:
                # Remove the unneccesary ann activations (more than 2 steps back.)
                if ann_len > 3:
                    self.ann_acts.pop(0)
                    ann_len = 3
                for pprev,prev,cur in zip(self.ann_acts[ann_len-3],self.ann_acts[ann_len-2],self.ann_acts[ann_len-1]):
                    if (prev > pprev and prev > cur) or (prev < pprev and prev < cur):
                        self.dir_changes += 1

            # Record joint forces for calculation in the fitness function.
            jf = quadruped.get_joint_feedback()
            # self.forces.append([(jf[0] if jf[0] < quadruped.joint_max_power else quadruped.joint_max_power),
            #                         (jf[2] if jf[2] < quadruped.joint_max_power else quadruped.joint_max_power)])
            self.forces.append([vector_3d_magnitude(jf[i][0]) if vector_3d_magnitude(jf[i][0]) < 8.0*quadruped.joint_max_power else 8.0*quadruped.joint_max_power,
                vector_3d_magnitude(jf[i][2]) if vector_3d_magnitude(jf[i][2]) < 8.0*quadruped.joint_max_power else 8.0*quadruped.joint_max_power] for i in range(len(jf)))
            
            if self.validating:
                # Record the ANN activations and joint feedback for validation.
                self.ann_activations.append(nn_activations)

                self.joint_feedback.append([self.elapsed_time,
                                            i,
                                            sum(jf[i][0]),
                                            sum(jf[i][1]),
                                            sum(jf[i][2]),
                                            sum(jf[i][3])] for i in range(len(jf)))

            quadruped.set_joint_power(power=nn_activations[16:])

            quadruped.actuate_joints_by_pos(positions=[nn_activations[i:i+2] for i in xrange(0, 16, 2)])
        else:   
            quadruped.set_joint_power(power=[1. for i in range(8)])
            
            jf = quadruped.get_joint_feedback()

            # Actuate all joints at a steady speed.
            quadruped.actuate_joints_by_pos(positions=[[math.sin(2.*math.pi*(self.elapsed_time)),math.sin(2.*math.pi*(self.elapsed_time))] for i in xrange(8)])

    def reset_simulation(self):
        """ Reset the simulation. """
        
        man.delete_joints()
        man.delete_bodies()
        self.elapsed_time = 0.
        self.exploded = False
        self.forces = []
        self.ann_acts = []
        self.dir_changes = 0

    def simulate(self):
        """ Perform physics simulation. """

        if self.elapsed_time < self.eval_time: 
            self.update_callback()
            # Periodically check to see if we exploded.
            if int(self.elapsed_time*10.) % 2 == 0 and (man.get_body_position(0)[1] > 5. \
                or man.get_body_position(0)[0] > 40. \
                or man.get_body_position(0)[2] > 40.): 
                self.exploded = True
        if self.elapsed_time >= self.eval_time or self.exploded:
            fit = [0.,0.,0.]
            if not self.exploded:
                fit[0] = norm_dist_efficiency_fitness(fit[1],fit[2])
                fit[1] = euclidean_distance(man.get_body_position(0),[0,0,0])
                fit[2] = distance_per_unit_of_power(man.get_body_position(1),[0,0,0],self.forces)
                
                # Check to see if we exceeded the direction change limit.
                if self.dir_changes > self.dir_change_limit:
                    fit[0] = 0.

            self.reset_simulation()
            return False, fit
        quadruped.reset_touch_sensors()
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
            self.ann_activations = []

        return fit

    def evaluate_individual(self,genome):
        """ Evaluate an individual solution. 

        Args:
            genome: genome of the individual to evaluate

        Returns:
            fitness value of the individual
        """
        global man,current_network,quadruped

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames,erp=0.5,cfm=1E-2)

        # Initialize the quadruped
        quadruped = Quadruped(man=man,morphology_genome={'erp':genome[1][0],'cfm':genome[1][1]})

        # Load in the ANN from the population
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome[0].BuildPhenotype(self.current_network)
        else:
            genome[0].BuildHyperNEATPhenotype(self.current_network,self.substrate)

        # Conduct the evaluation
        fit = self.physics_only_simulation() 
        #print(genome.GetID(), fit)

        return fit

    def validator(self,NEAT_file,Morph_file):
        """ Validate a single run. 

        Args:
            NEAT_File: file for the NEAT genome
            Morph_file: file for the morphology components
        """
        global man, quadruped

        self.validating = True

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num,erp=0.5,cfm=1E-2)

        genome = [0,{}]
        if NEAT_file:
            # Load in the best performing NEAT genome
            genome[0] = NEAT.Genome(NEAT_file)
            self.current_network = NEAT.NeuralNetwork()
            if not self.hyperNEAT:
                genome[0].BuildPhenotype(self.current_network)
            else:
                genome[0].BuildHyperNEATPhenotype(self.current_network,self.substrate)

        if Morph_file:
            with open(Morph_file,"r") as f:
                line = f.readline()
                line = line.strip().split(',')
                genome[1]['erp'] = float(line[0])
                genome[1]['cfm'] = float(line[1])

        # Initialize the quadruped
        quadruped = Quadruped(man=man,morphology_genome={'erp':genome[1]['erp'],'cfm':genome[1]['cfm']})

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        # Must be placed after creating the quadruped.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        fit = self.physics_only_simulation_validator()

        return self.ann_activations,self.joint_feedback
