import GlobalVarWorkaround

import math
from ODESystem import ODEManager
import GlobalVarWorkaround
import math


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

class Box(object):
    def __init__(self,man, desnsity, base_pos=[0,0,0]):
        self.SEG_DIMS = [.50    # x -length
                    ,.50                                # y -length
                    ,.50]                               # z -length
        
        self.man = man
        self.body_keys = []
        #density=BODY_MASS
        self.body_keys.append(self.man.create_box(0,self.SEG_DIMS,base_pos,density=desnsity))

    def getposition(self):
        return self.man.get_body_position(0)

    def getlength(self,direction):
        return self.SEG_DIMS[0]


class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, eval_time, dt, n, man, box, applyforcetill):
        """ Initialize the simulation class. """

        # aquatic simulation
        self.aquatic = True

        self.man = man
        self.box = box

        # Settings for the simulation.
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        # Check for explosions
        self.exploded = False

        self.avgpositioncontainer = []
        self.directionjointoffset = 6

        self.nextlogtimestamp = 0.
        self.applyforcetill = applyforcetill

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """     

        if self.elapsed_time <= self.applyforcetill:
            odeworld = self.man.world
            force = odeworld.impulseToForce(self.man.stepsize,[1,0,0])
            self.man.bodies[0].addForce([force[0],force[1],force[2]]) # add force to box

        self.man.sim_fluid_dynamics()

        pos = self.box.getposition()
        with open("bodypos.out", "a") as myfile:
            myfile.write(`self.elapsed_time` + ' ' + `pos` + '\n')

    def reset_simulation(self):
        """ Reset the simulation. """
        
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
            # write body length
            bodylength = self.box.getlength(0)
            with open("body.out", "w") as myfile:
                myfile.write('length:' + `bodylength`)
            what = False
            endpos = self.box.getposition()
            # force worm to swim in one direction
            # multiply the euclidean distance with the -1 if the endpos lies in -x area
            fit = math.copysign(1, endpos[0]) * euclidean_distance(endpos,[0,0,0])
            self.reset_simulation()

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