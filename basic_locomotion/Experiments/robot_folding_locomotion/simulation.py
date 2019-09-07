import GlobalVarWorkaround

from Robot import Sensors
import math
import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog
from ODESystem import ODEManager
import GlobalVarWorkaround
import math
import csv
import itertools


def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 


def calc_angle_for_circle(num_joints):
    assert num_joints > 0, "num_joints must be positive: %r" % num_joints

    return math.pi/num_joints

# convert value from a range to another range, taken from
# http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
# use normalize() from python angles package if you want to limit the result
def convert_range(from_min, from_max, to_min, to_max, from_val):
    ''' Convert from_val to a a value within to_min and to_max.
        All values are converted to floats.
    '''
    assert from_max != from_min, "from_max and from_min must not be the same value! from_max: %r from_min: %r" % (from_max,from_min)
    from_min=float(from_min)
    from_max=float(from_max)
    to_min=float(to_min)
    to_max=float(to_max)
    from_val=float(from_val)
    from_range = (from_max - from_min)
    to_range = (to_max - to_min)
    to_val = (((from_val - from_min) * to_range) / from_range) + to_min
    return to_val


def managerjointactuation_scale(from_minangle, from_maxangle, from_angle):
    # see manager.py actuate_universal()
    LOWERBOUND = -1
    UPPERBOUND = +1
    
    assert (from_minangle < from_maxangle), "Minimum angle shall be less than maximum angle!"
    scale1 = abs(from_maxangle - from_minangle)
    dist1 = abs(from_angle - from_minangle)
    prop1 = dist1/scale1

    scale2 = abs(UPPERBOUND - LOWERBOUND)
    # calculate factor between those two scales
    fact = scale2/scale1
    dist2 = fact * dist1

    absval2 = LOWERBOUND + dist2
    return absval2

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
        man.create_sphere(99, 1., 0.5,base_pos)
        self.position = base_pos

    def get_position(self):
        return self.position

class Worm(object):

    def __init__(self, man, num_joints, logging=False, log_path='./', logfileprefix=''):

        self.man = man
        self.body_keys = []

        self.logging = logging
        self.logfileprefix = logfileprefix

        # Sensors for robot.
        self.sensor = Sensors(man=man, logging=logging, log_path=log_path)

        assert num_joints >= 0, "num_joints must be positive or zero: %r" % num_joints
        self._num_joints = num_joints
        self._num_bodies = num_joints +1

        # Initialize the robot.
        self.__create_robot()

    def __create_robot(self):

        # small segment
        SEG_DIMS = [1.50   # x -length
                    ,.50    # y -length
                    ,.50]   # z -length

        BODY_POS = [-SEG_DIMS[0] / 2  # x-pos
                    ,1.    # y -pos
                    ,0.]   # z -pos
       
        BODY_DENSITY = 50

        MAXJOINT_FORCE = [100000.,100000.]

        joint_range = math.radians(90.)

        assert self._num_bodies >= 2, "number of bodies must be greater than 2 to use this create_robot(): %r" % self._num_bodies
        
        # create front segment
        b_num = 0
        self.body_keys.append(self.man.create_box(key=b_num,dim=SEG_DIMS,pos=BODY_POS,density=BODY_DENSITY
                                                  , active_surfaces={'x':1,'y':1,'z':1,'-x':0,'-y':1,'-z':1}))
        print 'robot front xpos=' + str(BODY_POS[0]+SEG_DIMS[0]/2)

        # create middle segment(s)
        BODY_POS[0] -= SEG_DIMS[0]
        j_num = 0
        for counter in range(1,self._num_bodies-1):
            b_num += 1
            self.body_keys.append(self.man.create_box(key=b_num,dim=SEG_DIMS
                                                      ,pos=BODY_POS
                                                      ,density=BODY_DENSITY
                                                      ,active_surfaces={'x':0,'y':1,'z':1,'-x':0,'-y':1,'-z':1}))

            con_point = [BODY_POS[0]+SEG_DIMS[0]/2.,BODY_POS[1],BODY_POS[2]]
            self.man.create_universal(j_num, con_point,[b_num-1,b_num],axis1=[0,0,-1],axis2=[0,1,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=MAXJOINT_FORCE[0],fmax2=MAXJOINT_FORCE[1])
            j_num += 1
            BODY_POS[0] -= SEG_DIMS[0]

        # create last segment(s)
        b_num += 1
        self.body_keys.append(self.man.create_box(key=b_num,dim=SEG_DIMS
                                                    ,pos=BODY_POS
                                                    ,density=BODY_DENSITY
                                                    ,active_surfaces={'x':0,'y':1,'z':1,'-x':1,'-y':1,'-z':1}))

        con_point = [BODY_POS[0]+SEG_DIMS[0]/2.,BODY_POS[1],BODY_POS[2]]

        self.man.create_universal(j_num, con_point,[b_num-1,b_num],axis1=[0,0,-1],axis2=[0,1,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=MAXJOINT_FORCE[0],fmax2=MAXJOINT_FORCE[1])

        print 'robot end xpos=' + str(BODY_POS[0]+SEG_DIMS[0]/2)

        self.sensor.register_joint_sensors([i for i in range(self._num_joints)])

        # Turn feedback on for the joints in order access the force and torque values
        for i in range(self._num_joints):
            self.man.joints[i].setFeedback()

        self._num_bodies = b_num+1
        self._num_joints = j_num+1

    def get_joint_feedback(self):
        """ Get the feedback from the joints.
        Per joint: force-body1 (x,y,z); torque-body1 (x,y,z); frce-body2 (x,y,z); torque-body2 (x,y,z)"""

        joint_feedback = []
        for i in range(self._num_joints):
            joint_feedback.append(self.man.joints[i].getFeedback())

        return joint_feedback

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

        #return self.man.get_body_position(jointidx)
        return self.man.get_uni_joint_position(jointidx, 1)

    def get_joint_bodies(self,jointidx):
        assert (jointidx >= 0 and jointidx <= self._num_joints), "Invalid joint index: %r" % jointidx
        return [self.man.get_joint(jointidx).getBody(0),self.man.get_joint(jointidx).getBody(0)]

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

    def log_joint_feedback(self,timestamp):
        jfeedback = self.get_joint_feedback();

        chain = []
        # flatten list of tuples
        for jointidx in range(len(jfeedback)):
            chain.append([item for tempList in jfeedback[jointidx] for item in tempList])

        #flatten list of lists
        chain = itertools.chain(*chain)
        with open("joint_feedback.out", "ab") as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter.writerow([`timestamp`] + list(chain))


def managerjointactuation_scale(minangle, maxangle, angle):
    # see manager.py actuate_universal()
    LOWERBOUND = -1
    UPPERBOUND = +1
    
    assert (minangle < maxangle), "Minimum angle shall be less than maximum angle!"
    scale1 = abs(maxangle - minangle)
    dist1 = abs(angle - minangle)
    prop1 = dist1/scale1

    scale2 = abs(UPPERBOUND - LOWERBOUND)
    # calculate factor between those two scales
    fact = scale2/scale1
    dist2 = fact * dist1

    absval2 = LOWERBOUND + dist2
    return absval2


class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, eval_time, dt, n, man, worm):
        """ Initialize the simulation class. """

        # aquatic simulation
        self.aquatic = True

        self.man = man
        self.worm = worm

        # Settings for the simulation.
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        # init worm object - one worm per simulation
        worm = ""

        # init current network
        #self.current_network = 0

        # Use an periodic (oscilating) input signal
        self.periodic = True

        # Check for explosions
        self.exploded = False

        self.expectedpos = []


    def update_callback(self):

        self.worm.get_sensor_states() # necessary for joint angle logging..

        pos = [None] * self.worm._num_joints # create empty list


        f = 1 # frequency (Hz)
        w = 2*math.pi*f # angular frequency (rad/sec)
        
        # optimized for robot with 16 joints
        a_val = 0.2
        offset_val = 0.3

        # left turn
        #a = -a_val
        #offset = offset_val

        # right turn
        a = a_val
        offset = -offset_val

        assert self.worm._num_joints > 8, "this implementation works only with more than 8 joints: %r" % self.worm._num_joints

        for jointidx in range(0,4+1,2):
            cur_joint_angle = self.worm.get_joint_pos(jointidx)
            pos[jointidx] = 0 #cur_joint_angle
            pos[jointidx+1] = convert_range(-90, +90, -1, +1, -90) # degree to angle used by manager, restricted by jointrange

        pos[6] = 0
        pos[7] = convert_range(-90, +90, -1, +1, +45)

        if self.elapsed_time > 3.0:
            for jointidx in range(8,self.worm._num_joints):
                ##phi = 2*math.pi/(self.worm._num_joints-7) * (8-jointidx)
                phi = 2*math.pi/(self.worm._num_joints-7)*1.5 * (8-jointidx) #forward
                #phi = -2*math.pi/(self.worm._num_joints-7)*1.5 * (8-jointidx) # backward
                jointangle = a * math.sin(w*self.elapsed_time+phi) # forward
                #jointangle = a * math.sin(w*self.elapsed_time+phi) + 0.1 # veering
                #jointangle = a * math.sin(w*self.elapsed_time+phi) # backward
                pos[jointidx] = jointangle
        else:
            for jointidx in range(8,self.worm._num_joints):
                pos[jointidx] = 0


        self.worm.sensor_step(self.elapsed_time)

        if len(self.expectedpos) == 0: self.expectedpos = [0.0 for i in range(len(pos))]
        with open("expected_joint_angles.out", "ab") as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter.writerow([`self.elapsed_time`] + [self.expectedpos])

        self.expectedpos = pos # joint angle next time when update_callback() is called


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
            fit = endpos
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