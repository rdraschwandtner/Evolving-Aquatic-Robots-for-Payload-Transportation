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
import json

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


class Destination(object):
    def __init__(self,man, base_pos=[0,0,0]):
        self.man = man
        radius = 8
        man.create_cylinder(98,1.,0.1,radius,(0.,0.1,0.),0,(90.,0.,0.))


class Target(object):
    def __init__(self,man, base_pos=[0,0,0]):
        self.man = man
        #man.create_sphere(99, 1., 0.5,base_pos)
        man.create_sphere(99, 1., 0.5,base_pos)
        self.position = base_pos

    def get_position(self):
        return self.man.get_body_position(99)

    def get_initposition(self):
        return self.position

    def log_avg_position(self,timestamp):
        pos = self.get_position()
        with open("targetpos.out", "a") as myfile:
            myfile.write(`timestamp` + ' ' + `pos` + '\n')

    #def getforce(self):
    #    return self.man.get_body(99).getForce()

    #def setforce(self,f):
    #    self.man.get_body(99).setForce(f)

    def getvelocity(self):
        return self.man.get_body(99).getLinearVel()

    def setvelocity(self,vel):
        self.man.get_body(99).setLinearVel(vel)

def tighten_circle(straightend_jointidx, numofjoints):
    assert straightend_jointidx >= 0 and numofjoints > 0, "straightend_jointidx: %r, numofjoints: %r" % (straightend_jointidx,numofjoints)
    newangles = [None] * (numofjoints-straightend_jointidx) # calculated the number of new angles
    firstcirclejointidx = straightend_jointidx+1
    angle_part = 360./(numofjoints-firstcirclejointidx+1)
    newangles[0] = 0
    for i in range(1,numofjoints-straightend_jointidx):
        newangles[i] = angle_part
    return newangles

def grasp_algo(cnt, numofjoints, rel_zdirection, reverse=False):    
    assert cnt >= 0, "ensure non negative count: (%r)" % cnt
    assert cnt < numofjoints, "cnt(%r) < numofjoints (%r)" % (cnt,numofjoints)
    assert rel_zdirection == +1 or rel_zdirection == -1, "rel_zdirection: %r" % rel_zdirection

    angles = [None] * numofjoints

    if cnt == 0: 
        angle_part = 360./(numofjoints+1)
        angles = [angle_part * rel_zdirection] * numofjoints
    else:
        angles = [0] * numofjoints
        angles[cnt-1:numofjoints] = [angle * rel_zdirection for angle in tighten_circle(cnt-1, numofjoints)]

    if reverse == True:
        angles.reverse()

    return angles


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
        #print 'robot front xpos=' + str(BODY_POS[0]+SEG_DIMS[0]/2)

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

        #print 'robot end xpos=' + str(BODY_POS[0]+SEG_DIMS[0]/2)

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

    def getvelocity(self,bodyidx):
        return self.man.get_body(bodyidx).getLinearVel()

    def setvelocity(self,bodyidx,vel):
        self.man.get_body(bodyidx).setLinearVel(vel)

class JointActuationController(object):
    def __init__(self, amplitude=0, frequency=0, phaseshift=0, offset=0):
        self.amplitude = amplitude
        self.frequency = frequency
        self.phaseshift = phaseshift
        self.offset = offset

    def serialize(self):
        return json.dumps({'a':self.amplitude
                          ,'f':self.frequency
                          ,'phi':self.phaseshift
                          ,'offset':self.offset})
    
    @staticmethod
    def deserialize(json_string):
        vals = json.loads(json_string)
        return JointActuationController(vals['a'], vals['f'], vals['phi'], vals['offset'])

def grasp_algo_wrapper(cnt, numofjoints, rel_zdirection, reverse=False):
    angles = grasp_algo(cnt,numofjoints, rel_zdirection, reverse)
    return [convert_range(-90, +90, -1, +1, angles[i]) for i in range(numofjoints)]


def in_circle(worm, bodyidcs, target):
    """ assumes that the last body segment is the outer segment!"""
    numbodysegmentedges = len(bodyidcs)+1
    x_cnt = z_cnt = 0
    for i in bodyidcs:
        x,y,z = worm.man.get_body(i).getRelPointPos((-1.5/2.,0.,0.))
        #x,y,z = worm.man.get_body_position(i)
        x_cnt = (x_cnt+1) if x >= target.get_position()[0] else (x_cnt-1)
        z_cnt = (z_cnt+1) if z >= target.get_position()[2] else (z_cnt-1)

    x,y,z = worm.man.get_body(bodyidcs[0]).getRelPointPos((+1.5/2.,0.,0.))
    x_cnt = (x_cnt+1) if x >= target.get_position()[0] else (x_cnt-1)
    z_cnt = (z_cnt+1) if z >= target.get_position()[2] else (z_cnt-1)

    return False if (abs(x_cnt) == numbodysegmentedges or abs(z_cnt) == numbodysegmentedges) else True


class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, eval_time, dt, n, man, worm, jointctrlers, target):
        """ Initialize the simulation class. """

        # aquatic simulation
        self.aquatic = True

        self.man = man
        self.worm = worm
        self.target = target

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


        self.jointctrlers = jointctrlers

        self.avgpositioncontainer = []

        self.graspcnt = 0
        self.nextgraspcnt = 0.
        self.grasped = False
        #self.snapdirection = [0,False]
        self.snapdirection = [0,True]

        self.delivered = False


        self.pausetime = 0.
        #self.straightstart = -1.


    def calcaverageposition(self,pos):
        windowsize = 30
        if len(self.avgpositioncontainer) >= windowsize:
            self.avgpositioncontainer.pop(0)
            
        self.avgpositioncontainer.append(pos)

        avg = sum([self.avgpositioncontainer[row] for row in range(len(self.avgpositioncontainer))])/len(self.avgpositioncontainer)

        return avg

    def update_callback(self):

        self.worm.get_sensor_states() # necessary for joint angle logging..

        pos = [None] * self.worm._num_joints # create empty list

        MOTIONSTATES = {#'forward':(0.7,0.9,0.8,0.)
                        #,'right':(0.7,0.9,0.8,-0.2)
                        #'right':(0.7,0.5,0.8,-0.2)
                        'right':(0.7,0.9,0.8,-0.2)
                        #,'left':(0.7,0.9,0.8,+0.05)
                        ,'reverse':(0.7,0.9,-0.8,0.)
                        ,'rest':(0.,0.,0.,0.)
                        }

        destination = (0,0,0)

        assert self.worm._num_joints == 16, "This implementation has been optimized for 16 joints: %r" % self.worm._num_joints

        targetpos = self.target.get_position()
        if euclidean_distance(destination,targetpos) <= 4. or self.delivered == True:
            self.delivered = True
            self.grasped = False
            self.graspcnt = 0
            self.nextgraspcnt = 0.
            #ampl,freq,phs,gamma = MOTIONSTATES['rest']
            #for jointidx in range(self.worm._num_joints-4,self.worm._num_joints):
            #    pos[jointidx] = 0
            ampl,freq,phs,gamma = MOTIONSTATES['reverse']
            for jointidx in range(self.worm._num_joints):
                pos[jointidx] = gamma + ampl * math.sin(2.*math.pi*freq*self.elapsed_time-phs*jointidx)
                #pos[jointidx] = 0
        else:
            if self.grasped == False:
                headpos_center = self.man.get_body_position(0)
                com = self.worm.get_avg_position()

                # normalize both vectors
                normalizedvec_a = (headpos_center[0] - com[0]
                                    , headpos_center[2] - com[2])

                comdist = euclidean_distance(targetpos,com)
                robotlength = (self.worm._num_joints+1) * 1.5

                normalizedvec_b = (targetpos[0] - com[0]
                                    , targetpos[2] - com[2])

                theta = angle2_2d(normalizedvec_a, normalizedvec_b)

                smoothed_theta = self.calcaverageposition(theta)

                ampl,freq,phs,gamma_max = MOTIONSTATES['right']
                gamma = gamma_max/math.pi * smoothed_theta

                if comdist < robotlength:
                    # d = U/pi
                    if comdist < robotlength/math.pi:
                        #print 'grasp'
                        if self.elapsed_time > self.nextgraspcnt and self.graspcnt < self.worm._num_joints-3:
                            self.graspcnt += 1
                            tightentimestep = 0.33 # 1 second
                            self.nextgraspcnt = self.elapsed_time + tightentimestep
                        assert self.snapdirection != 0, "snap direction hasn't been set"
                        pos = grasp_algo_wrapper(self.graspcnt,self.worm._num_joints, self.snapdirection[0], self.snapdirection[1])
                        if self.graspcnt == self.worm._num_joints-3: # last grasp
                            self.grasped = True
                            del self.avgpositioncontainer[:] # TODO move this to movement with target object section
                    else:
                        self.graspcnt = 0
                        self.nextgraspcnt = 0.
                        self.snapdirection[0] = -1 if theta>=0 else +1

                        ampl,freq,phs,gamma_max = MOTIONSTATES['right']
                        for jointidx in range(self.worm._num_joints):
                            assert freq >= 0, "frequency shall not be negative!"
                            pos[jointidx] = gamma + ampl * math.sin(2.*math.pi*freq*self.elapsed_time-phs*jointidx)
                else:
                    self.graspcnt = 0
                    self.nextgraspcnt = 0.
                    ampl,freq,phs,gamma_max = MOTIONSTATES['right']
                    for jointidx in range(self.worm._num_joints):
                        assert freq >= 0, "frequency shall not be negative!"
                        pos[jointidx] = gamma + ampl * math.sin(2.*math.pi*freq*self.elapsed_time-phs*jointidx)

            else:
                #if self.grasped == True:
                #    if in_circle(self.worm,range(self.graspcnt,self.worm._num_joints),self.target) == True:
                #        pos = grasp_algo_wrapper(self.graspcnt,self.worm._num_joints, self.snapdirection[0], self.snapdirection[1])
                #        ampl,freq,phs,gamma_max = MOTIONSTATES['right']
                #        orientation_jointidx = self.graspcnt

                #        pos[orientation_jointidx] = convert_range(-90, +90, -1, +1, +45)

                #    else:
                #        assert 1 == 0, "target not in circle anymore, not handled so far"

                #print 'target in circle? ' + str(in_circle(self.worm,range(self.graspcnt+1,self.worm._num_joints+1),self.target)) + ' t=' + str(self.elapsed_time)
                if in_circle(self.worm,range(0,3),self.target) == True:

                    headpos_center = self.man.get_body_position(0)
                    com = self.worm.get_avg_position()

                    # normalize both vectors
                    normalizedvec_a = (headpos_center[0] - com[0]
                                        , headpos_center[2] - com[2])

                    comdist = euclidean_distance(destination,com)

                    normalizedvec_b = (destination[0] - com[0]
                                        , destination[2] - com[2])

                    theta = angle2_2d(normalizedvec_a, normalizedvec_b)

                    smoothed_theta = self.calcaverageposition(theta)

                    ampl = 0.61
                    freq = 1.82
                    #freq = 1.82/2.
                    phs = 0.75
                    gamma_max = -0.2 # taken from initial right motion
                    gamma = gamma_max/math.pi * smoothed_theta

                    pos = grasp_algo_wrapper(self.graspcnt,self.worm._num_joints, self.snapdirection[0], self.snapdirection[1])

                    pos[3] = convert_range(-90, +90, -1, +1, +45)
                    for jointidx in range(3,self.worm._num_joints):
                        assert freq >= 0, "frequency shall not be negative!"
                        pos[jointidx] = gamma + ampl * math.sin(2.*math.pi*freq*self.elapsed_time-phs*jointidx)

                else:
                    #assert 1==0,"not implemented yet"
                    # pause for a bit and retry

                    #if self.pausetime == 0.:
                    #    self.pausetime = self.elapsed_time+3. # pause for 3 seconds


                    #if self.elapsed_time >= self.pausetime and self.pausetime != 0.:
                    #    self.grasped = False
                    #    self.pausetime = 0.

                    robotlength = (self.worm._num_joints+1) * 1.5
                    com = self.worm.get_avg_position()
                    comdist = euclidean_distance(targetpos,com)
                    if comdist > robotlength*2:
                        self.grasped = False

                    ampl,freq,phs,gamma = MOTIONSTATES['reverse']
                    for jointidx in range(self.worm._num_joints):
                        pos[jointidx] = gamma + ampl * math.sin(2.*math.pi*freq*self.elapsed_time-phs*jointidx)
                    


        # log
        self.worm.log_avg_bodyposition(self.elapsed_time)
        self.worm.log_bodysegmentcenter(self.elapsed_time,0) # log headpos
        self.target.log_avg_position(self.elapsed_time)

        self.worm.sensor_step(self.elapsed_time)

        self.worm.actuate_joints_by_pos(positions=pos)

        self.man.sim_fluid_dynamics()

        # fix the target to XZ plane
        #targetforce_x,targetforce_y,targetforce_z = self.target.getforce()
        #self.target.setforce((targetforce_x,-targetforce_y,targetforce_z))
        targetinitpos = self.target.get_initposition()
        targetcurpos = self.target.get_position()
        diffypos = targetcurpos[1] - targetinitpos[1]

        diffspeed = diffypos / self.dt

        targetvel_x,targetvel_y,targetvel_z = self.target.getvelocity()
        self.target.setvelocity((targetvel_x,-diffspeed,targetvel_z))

        for bodyidx in range(self.worm._num_bodies):
            init_ypos = 1.
            curpos = self.man.get_body_position(bodyidx)
            diffypos = curpos[1] - init_ypos
            diffspeed = diffypos/self.dt

            wormvel_x,wormvel_y,wormvel_z = self.worm.getvelocity(bodyidx)
            self.worm.setvelocity(bodyidx,(wormvel_x,-diffspeed,wormvel_z))



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
            # log the joint controller
            #jointctrlerstr = self.jointctrler.serialize()
            #with open("jointctrler.out", "w") as text_file:
            #    text_file.write(jointctrlerstr)
            

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

        startpos = self.worm.get_avg_position()
        # Conduct the evaluation
        endpos = self.physics_only_simulation() 

        fit = [endpos[0]-startpos[0], endpos[1]-startpos[1], endpos[2]-startpos[2]]

        return fit