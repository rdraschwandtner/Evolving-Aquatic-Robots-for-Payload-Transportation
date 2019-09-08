import GlobalVarWorkaround

from Robot import Sensors
import math
import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog
from ODESystem import ODEManager
import GlobalVarWorkaround
import math


def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

############
# see http://math.stackexchange.com/questions/878785/how-to-find-an-angle-in-range0-360-between-2-vectors
def determinant2d(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]

# http://en.wikipedia.org/wiki/Trigonometric_functions
# http://en.wikipedia.org/wiki/Determinant#2.C2.A0.C3.97.C2.A02_matrices
# tan2(sine/cosine)
def angle2_2d(v1,v2):
    return math.atan2(determinant2d(v1,v2),dotproduct(v1,v2))

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


class Target(object):
    def __init__(self,man, base_pos=[0,0,0]):
        man.create_sphere(99, 1., 1,base_pos)
        self.position = base_pos

    def get_position(self):
        return self.position

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

    def get_joint_pos(self, jointidx):
        assert (jointidx >= 0 and jointidx <= self._num_joints), "Invalid joint index: %r" % jointidx

        return self.man.get_body_position(jointidx)

    def clear_sensors(self):
        """ Clears all sensor values
        """
        self.sensor.clear_sensors()

    def log_sensor_data(self):
        self.sensor.dump_sensor_data(self.logfileprefix)

    def sensor_step(self, cur_time):
        self.sensor.step(cur_time)

    def log_avg_bodyposition(self,timestamp):
        avgbodypos = self.get_avg_position()
        with open("avg_bodypos.out", "a") as myfile:
            myfile.write(`timestamp` + ' ' + `avgbodypos` + '\n')

    def log_bodysegmentcenter(self,timestamp,bodyidx):
        bodypos = self.man.get_body_position(bodyidx)
        with open("body_" + `bodyidx`+ "_pos.out", "a") as myfile:
            myfile.write(`timestamp` + ' ' + `bodypos` + '\n')


class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, eval_time, dt, n, man, worm, target):
        """ Initialize the simulation class. """

        # aquatic simulation
        self.aquatic = True

        self.man = man
        self.worm = worm
        self.target = target
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
        #self.current_network = 0

        # Use an periodic (oscilating) input signal
        self.periodic = True

        # Check for explosions
        self.exploded = False

        self.avgpositioncontainer = []
        self.directionjointoffset = 6

        self.nextlogtimestamp = 0.

    def calcaverageposition(self,pos):
        windowsize = 300
        if len(self.avgpositioncontainer) >= windowsize:
            self.avgpositioncontainer.pop(0)
            
        self.avgpositioncontainer.append(pos)

        #avg_x = sum([self.avgpositioncontainer[row][0] for row in range(len(self.avgpositioncontainer))])/len(self.avgpositioncontainer)
        #avg_y = sum([self.avgpositioncontainer[row][1] for row in range(len(self.avgpositioncontainer))])/len(self.avgpositioncontainer)
        #avg_z = sum([self.avgpositioncontainer[row][2] for row in range(len(self.avgpositioncontainer))])/len(self.avgpositioncontainer)
        avg = sum([self.avgpositioncontainer[row] for row in range(len(self.avgpositioncontainer))])/len(self.avgpositioncontainer)

        #return (avg_x, avg_y, avg_z)
        return avg


    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """     

        # state machine
        MOTIONSTATES = {'forward':(0,1,0.6),'right':(-0.05,1,0.6),'left':(0.05,1,0.6),'stop':(0.,0.,0.6)}

        # position sensor
        targetpos = self.target.get_position()

        # head position
        # head center position
        headpos_center = self.man.get_body_position(0)

        headdist = euclidean_distance(targetpos, headpos_center)
        #print headdist

        offset = 0
        amp = 0
        freq = 0

        if headdist > 2.0:
            # calc angle to target
            #avgbodypos = self.worm.get_avg_position()
            #angle = math.atan2(targetpos[2]-headpos[2],targetpos[0]-headpos[0]) #headpos[2] .. Z, headpos[0] .. X
            #angle = math.atan(targetpos[2]-headpos[2]/targetpos[0]-headpos[0])
            #print 'z:' + `targetpos[2]-headpos[2]`
            #print 'x:' + `targetpos[0]-headpos[0]`


            # approximate the center of mass with using both semgents in the middle
            # and taking the average of the two middle positions
            #centerofbody4 = self.man.get_body_position(4)
            #centerofbody3 = self.man.get_body_position(3)
            ## center of mass absolute world position
            #com = ((centerofbody4[0]+centerofbody3[0])/2    # average position in x direction
            #       ,(centerofbody4[1]+centerofbody3[1])/2   # average position in y direction
            #       ,(centerofbody4[2]+centerofbody3[2])/2)  # average position in z direction

            

            #com = self.calcaverageposition(self.worm.get_avg_position())
            com = self.worm.get_avg_position()

            # normalize both vectors
            normalizedvec_a = (headpos_center[0] - com[0]
                               , headpos_center[2] - com[2])

            normalizedvec_b = (targetpos[0] - com[0]
                               , targetpos[2] - com[2])

            #print 'normalizedvec_a: ' + `normalizedvec_a` + 'normalizedvec_b: ' + `normalizedvec_b`


            #angle between both vecotrs
            #theta = angle(normalizedvec_a, normalizedvec_b)
            theta = angle2_2d(normalizedvec_a, normalizedvec_b)
            #theta = 0
            #print 'a: ' + `normalizedvec_a` + ';b: ' + `normalizedvec_b` + ';theta: ' + `theta`

            smoothed_theta = self.calcaverageposition(theta)

            angle_deg = math.degrees(smoothed_theta)
            print 'theta: ' + `angle_deg`
            if angle_deg > 10:
                offset, amp, freq = MOTIONSTATES['right']
                print 'right'
            elif angle_deg < -10:
                offset, amp, freq = MOTIONSTATES['left']
                print 'left'
            else:
                offset, amp, freq = MOTIONSTATES['forward']
                #print 'forward'

        else:
            offset, amp, freq = MOTIONSTATES['stop']
            #print 'stop'

        if self.nextlogtimestamp <= self.elapsed_time:
            self.worm.log_avg_bodyposition(self.elapsed_time)
            self.worm.log_bodysegmentcenter(self.elapsed_time,0) # log headpos
            self.nextlogtimestamp = self.elapsed_time + self.dt * 10

        pos = []

        # direction of movement is dependent on the order of actuation of the joints!
        # forward (+x-axis) actuation begins with last joints and vice versa
        for jointidx in range(self.worm._num_joints-1, -1, -1):
            phi = 2.*math.pi / self.worm._num_joints * jointidx # calc the phase shift for each joint

            assert freq >= 0, "frequency shall not be negative!"
            pos.append(offset + amp * math.sin(2.*math.pi*freq*self.elapsed_time+phi))

        self.worm.actuate_joints_by_pos(positions=pos)

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

    def evaluate_individual(self):
        """ Evaluate an individual solution. 

        Args:
            instructions: list of instructions for the ribosomal robot

        Returns:
            fitness value of the individual
        """

        # Conduct the evaluation
        fit = self.physics_only_simulation() 

        return fit