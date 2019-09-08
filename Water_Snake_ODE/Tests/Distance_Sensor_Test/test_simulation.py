"""
    Wrapper to conduct the actual simulation of the sensor test.  
    Access through methods: validator.
"""

import sys, os, random
import itertools
import math

sys.path.insert(0, '../../')

from ODESystem import ODEManager
from ODESystem import Placement

from Robot import Sensors

import ode

man = 0
test_setup = 0
elapse_time = 0

class Test_Setup(object):
    """ Represent the components of the simulation. """

    def __init__(self,man):
        """ Initialize the simulation objects. """

        self.man = man

        # Sensors for robot.
        self.sensor = Sensors(man)

        # Create the main box.
        self.__create_main_box()

        # Create the sphere.
        self.__create_sphere()

    def __create_main_box(self):
        """ Create the main box and attach it by a rotating hinge to the ground. """

        self.man.create_box(0,[1.,1.,1.],[0.,0.5,0.],density=5.0)
        self.man.create_ray(1,[0.5,0.5,0.0],[0,-90,0])
        self.man.create_hinge(0,[0.5,0.5,0.],[0,1],axis=[0.,1.,0.],lims=[-1.,1.],max_force=100.)

        # Create the distance sensor.
        self.sensor.add_dist_sensor(self.man.geoms[1],5.0)

    def __create_sphere(self):
        """ Create a sphere """
        self.man.create_sphere(3,4.0,0.5,[3.0,0.5,0.0])
        self.man.create_sphere(4,4.0,0.5,[4.0,0.5,0.0])

    def get_ray_distance(self):
        return self.sensor.get_raw_distance(self.man.geoms[1])

    def set_ray_distance(self,dist):
        self.sensor.set_distance(self.man.geoms[1],dist)

    def reset_ray(self):
        self.sensor.reset_distance_sensor(self.man.geoms[1])

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

    # Check to see if one of the two objects is a ray.
    if(type(geom1) == ode.GeomRay or type(geom2) == ode.GeomRay):
        ray = geom1 if type(geom1) == ode.GeomRay else geom2
        min_dist = test_setup.get_ray_distance()
        for c in contacts:
            min_dist = euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) if \
                euclidean_distance(ray.getPosition(),c.getContactGeomParams()[0]) < min_dist else min_dist
        test_setup.set_ray_distance(min_dist)
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

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.002, n=4):
        """ Initialize the simulation class. """
        global simulate

        man = ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, run_num=run_num)

        # Settings for the simulation.
        self.log_frames = log_frames
        self.run_num = run_num
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        # Quadruped Specific Setup
        test_setup = Test_Setup(man=man)

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """

        # Print the sim time and the distance of the collision sensor.
        print(str(self.elapsed_time)+","+str(test_setup.get_ray_distance()))
        test_setup.reset_ray()

        elapsed_time = self.elapsed_time
        man.actuate_hinge(0,math.sin(0.25*math.pi*(self.elapsed_time)-(math.pi/2.)))

    def reset_simulation(self):
        """ Reset the simulation. """
        
        man.delete_joints()
        man.delete_bodies()
        self.elapsed_time = 0.

    def simulate(self):
        """ Perform physics simulation. """

        if self.elapsed_time < self.eval_time: 
            self.update_callback()
        if self.elapsed_time >= self.eval_time:
            self.reset_simulation()
            return False, 1
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

    def validator(self):
        """ Validate a single run. 

        Args:
            NEAT_File: file for the NEAT genome
        """
        global man, test_setup

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, gravity=0,stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num)

        # Initialize the quadruped
        test_setup = Test_Setup(man=man)

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        fit = self.physics_only_simulation_validator()

        print(fit)
