
import ode
from VisualizationJSON import *
import copy
import math

#
# Simulation parameters for aquatic envirnment
#
TIME_STOP = 20
TIME_STEP = 0.05
GRAVITY = (0, 0, 0)
SIMULATION_ENV = "AQUATIC"
#GRAVITY = (0, -9.81, 0)
# gravity - buoyancy
#GRAVITY = (0, -9.81 + 9.81, 0)

#
# ODE objects
#
world = ode.World()
world.setGravity(GRAVITY)

# Create the collision space and and 'ground' plane
space = ode.Space()
ground_normal = (0., 1., 0.)
ground = ode.GeomPlane(space, ground_normal, 0)
contact_group = ode.JointGroup()

# Containers for rigid bodies, 3D geometries, and hinge joints
bodies = []
geoms = []
joints = []

# List of pairing that do not collide
# - example : [(body1, body2), (body3, body1)]
no_collide = []

# used for fluid dynamics
surfaces_per_obj_map = []

def create_surfaces(bodie):
    side_lens = bodie.boxsize
    x = side_lens[0]
    y = side_lens[1]
    z = side_lens[2]

    amp_adj = 1

    surfaces = []
    surfaces.append({'area':y*z, 'norm':[1,0,0], 'amp_adj':amp_adj})
    surfaces.append({'area':x*z, 'norm':[0,1,0], 'amp_adj':amp_adj})
    surfaces.append({'area':y*z, 'norm':[-1,0,0], 'amp_adj':amp_adj})
    surfaces.append({'area':x*z, 'norm':[0,-1,0], 'amp_adj':amp_adj})
    surfaces.append({'area':y*x, 'norm':[0,0,1], 'amp_adj':amp_adj})
    surfaces.append({'area':y*x, 'norm':[0,0,-1], 'amp_adj':amp_adj})

    return surfaces

def rotate3(m, v):
    """Returns the rotation of 3-vector v by 3x3 (row major) matrix m."""
    return (v[0] * m[0] + v[1] * m[1] + v[2] * m[2],
        v[0] * m[3] + v[1] * m[4] + v[2] * m[5],
        v[0] * m[6] + v[1] * m[7] + v[2] * m[8])

def apply_fluiddymics(simu_stepsize):
        """ Simulate an aquatic environment.

        Only done if the fluid_dynamics flag is on in manager.

        NOTE: Fluid dynamics are only in place for capsules and boxes.
        """

        drag_coefficient = .35
        amp_adj = 1.

        for bodyidx in range(len(bodies)):
            vel = bodies[bodyidx].getLinearVel()
            rot = bodies[bodyidx].getRotation()

            # Iterate through the 6 faces of a body (assuming box x,y,z parameters)
            for surfaceidx in xrange(6):
                area = surfaces_per_obj_map[bodyidx][surfaceidx]['area'] # Area of the surface
                norm = surfaces_per_obj_map[bodyidx][surfaceidx]['norm'] # Norm vector [x,y,z] of the surface
                amp_adj = surfaces_per_obj_map[bodyidx][surfaceidx]['amp_adj'] # Apply an amplitude adjustment for a body generating thrust

                adnorm = rotate3(rot,norm) # Reorient the normal to the bodies rotation in the world.

                component = adnorm[0]*vel[0]+adnorm[1]*vel[1]+adnorm[2]*vel[2]  # Get the component of force perpendicular to surface.
                component *= drag_coefficient*area  # Compensating for the size of the surface.
                if(component < 0):
                    component = 0    # If less than zero, no drag force to apply (Surface is opposite to drag force.)
                
                force = world.impulseToForce(simu_stepsize,[-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])
                #bodies[bodyidx].addForce([force[0],force[1],force[2]])  # Add force to the body from calculated surface.
                bodies[bodyidx].addForce([-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])  # Add force to the body from calculated surface.


def velocity_sinusoidal(cur_angle, cur_time, time_step, max_vel, phi = 0.):
    """calculate velocity to move from the current angle to the expted angle
        within the next timestep.
    """

    a = 1
    #a = 0.1
    
    w = 2. * math.pi #f = 1
    #w *= 10
    #phi = 0
    offset = 0

    exp_angle = a * math.sin(w * cur_time+phi) + offset
    delta_angle = exp_angle - cur_angle

    v = delta_angle / time_step

    #if v > max_vel:
    #    v = max_vel
    #elif v < -max_vel:
    #    v = -max_vel
        
    return v

def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms collide and
    creates contact joints if necessary.
    """

    # Check for restricted collisions
    # and do nothing if geom1 - geom2 is in no_collide container
    for pair in no_collide:
        if geom1.getBody() in pair and geom2.getBody() in pair:
            return

    # if the geom1 - geom2 pair isn't in the no_collide container
    # then handle the collision
             
    # Check for collisions
    contacts = ode.collide(geom1, geom2)

    # Create contact joints
    world, contact_group = args
    for c in contacts:
        c.setBounce(1.)
        c.setMu(5000)
        j = ode.ContactJoint(world, contact_group, c)
        j.attach(geom1.getBody(), geom2.getBody())


def add_cube(size, density, position, quaternion=None, color=0x555555):
    """Create a cube/box object, inlcuding visualization parameters.
    """

    # Create rigid body and set position and rotation
    cube = ode.Body(world)
    cube.setPosition(position)
    if quaternion is not None:
        cube.setQuaternion(quaternion)

    mass = ode.Mass()
    mass.setBox(density, *size)
    cube.setMass(mass)
    cube.boxsize = list(size)

    geom = ode.GeomBox(space, lengths=size)
    geom.setBody(cube)    
    geoms.append(geom)

    vis_prim = create_json_primitive("cube", size, color)
    json_object["primitives"].append(vis_prim)

    bodies.append(cube)
    return cube

def main():

    # self defined init wrapper
    # init the visualization functions
    visualizationJSON_init (TIME_STOP, TIME_STEP)

    cube_size = (1.5, 0.5, 0.5)
    cube_density = .1
    initial_cube_position_middle = list((1., cube_size[2], cube_size[2] * 0.5))

    NUM_OF_SEGMENTS = 8

    initial_cube = add_cube(cube_size, cube_density, initial_cube_position_middle)
    initial_cube_surfaces = create_surfaces(initial_cube)
    surfaces_per_obj_map.append(initial_cube_surfaces)

    prev_cube = initial_cube
    prev_cube_position = copy.deepcopy(initial_cube_position_middle)

    for i in range(1, NUM_OF_SEGMENTS):
        # create a cube
        cur_cube_position = copy.deepcopy(prev_cube_position)
        cur_cube_position[0] += cube_size[0] #+ cube_size[0]/16
        #cur_cube_position = list(cur_cube_position) # todo check if necessary
        cur_cube = add_cube(cube_size, cube_density, cur_cube_position)
        cur_cube_surfaces = create_surfaces(initial_cube)
        surfaces_per_obj_map.append(cur_cube_surfaces)

        joint = ode.HingeJoint(world)
        joint.attach(prev_cube, cur_cube)
        jointpos = copy.deepcopy(prev_cube_position)
        # place the joint exactly in the middle (x-axis) of both cubes
        jointpos[0] = prev_cube_position[0] + (cur_cube_position[0] - prev_cube_position[0])/2
        joint.setAnchor((jointpos))
        joint.setAxis((0,1,0)) # turn arround y- axis
        joints.append(joint)
        joint.setParam(ode.ParamLoStop, -1.6) # where lo < hi (radians)
        joint.setParam(ode.ParamHiStop, +1.6) # where hi > lo (radians)
        joint.setParam(ode.ParamFMax, 10)

        no_collide.append((prev_cube, cur_cube))

        prev_cube = cur_cube
        prev_cube_position = copy.deepcopy(cur_cube_position)

    ###########################
    ## Create a cube
    #cube_size = (1.5, 0.5, 0.5)
    #cube_density = .1
    ## x- position .. some offset value
    ## y- position .. "cube_size[2] / 2" the cube lies on the ground with its full surface
    ## z- position .. some offset value
    #cube_position_middle = (1., cube_size[2], cube_size[2] * 0.5)

    #cube1_position = list(cube_position_middle)
    #box1 = add_cube(cube_size, cube_density, cube1_position)

    #cube2_position = copy.deepcopy(cube1_position)
    #cube2_position[0] += cube_size[0] + cube_size[0]/16
    #cube2_position = list(cube2_position)
    #box2 = add_cube(cube_size, cube_density, cube2_position)

    #joint = ode.HingeJoint(world)
    #joint.attach(box1, box2)
    ## y and z position are the same as the boxes
    #jointpos = copy.deepcopy(cube1_position)
    ## place the joint in the middle of the two boxes
    #jointpos[0] = cube1_position[0] + (cube2_position[0] - cube1_position[0])/2

    #joint.setAnchor((jointpos))
    #joint.setAxis((0,1,0))

    #joints.append(joint)

    ##print joint.getAnchor()
    ##print joint.getAnchor2()


    #joint.setParam(ode.ParamLoStop, -1.6) # where lo < hi (radians)
    #joint.setParam(ode.ParamHiStop, +1.6) # where hi > lo (radians)

    ##joint.setParam(ode.ParamVel, 1)
    #joint.setParam(ode.ParamFMax, 100)

    #no_collide.append((box1, box2))


    #cube3_position = copy.deepcopy(cube2_position)
    #cube3_position[0] += cube_size[0] + cube_size[0]/16
    ##cube3_position = list(cube3_position)
    #box3 = add_cube(cube_size, cube_density, cube3_position)

    #joint2 = ode.HingeJoint(world)
    #joint2.attach(box2, box3)
    ## y and z position are the same as the boxes
    #joint2pos = copy.deepcopy(cube2_position)
    ## place the joint in the middle of the two boxes
    #joint2pos[0] = cube2_position[0] + (cube3_position[0] - cube2_position[0])/2

    #joint2.setAnchor((joint2pos))
    #joint2.setAxis((0,1,0))

    #joints.append(joint2)

    #joint2.setParam(ode.ParamLoStop, -1.6) # where lo < hi (radians)
    #joint2.setParam(ode.ParamHiStop, +1.6) # where hi > lo (radians)

    ##joint.setParam(ode.ParamVel, 1)
    #joint2.setParam(ode.ParamFMax, 100)

    #no_collide.append((box2, box3))

    #################################################################
    # Visualization

    ## Create the json primitive
    ##color=0x772277
    #vis_prim_sphere = create_json_primitive("sphere", sphere_radius, 0x772277)
    #json_object["primitives"].append(vis_prim_sphere)

    #vis_prim_cube = create_json_primitive("cube", cube_size, 0x555555)
    #json_object["primitives"].append(vis_prim_cube)


    print("time\tX\tY\tZ")

    #totalnumofit = int(TIME_STOP / TIME_STEP)
    #scale = int(1000)
    #divisor = int(totalnumofit / scale)

    time = 0.0
    while time < TIME_STOP:

        #joint_vel = velocity_sinusoidal(joint.getAngle(), time, TIME_STEP)
        #joint.setParam(ode.ParamVel, joint_vel)

        #joint2_vel = velocity_sinusoidal(joint2.getAngle(), time, TIME_STEP)
        #joint2.setParam(ode.ParamVel, joint2_vel)

        for i in range(0, NUM_OF_SEGMENTS-1):
            # oscilating snake
            #phi = math.pi if i % 2 == 1 else 0.
            #joint_vel = velocity_sinusoidal(joints[i].getAngle(), time, TIME_STEP, phi)
            joint_vel = velocity_sinusoidal(joints[i].getAngle(), time, TIME_STEP, (2.*math.pi/TIME_STEP) / 10, ((2.*math.pi)/NUM_OF_SEGMENTS) * i)
            joints[i].setParam(ode.ParamVel, joint_vel)
        
        #joint_vel = velocity_sinusoidal(joints[0].getAngle(), time, TIME_STEP,999999)
        #joints[0].setParam(ode.ParamVel, joint_vel)
            
        if SIMULATION_ENV == "AQUATIC":
            apply_fluiddymics(TIME_STEP)

        # Collide all objects
        space.collide((world, contact_group), near_callback)

        # Advance the physics world through time
        world.step(TIME_STEP)
        time += TIME_STEP

        # Reset collisions
        contact_group.empty()

        # Print to command-line (for debugging/plotting)
        #if int(time / TIME_STEP) % divisor == 0:
        #pos = sphere.getPosition()
        #print("s {}\t{}\t{}\t{}".format(time, pos[0], pos[1], pos[2]))
        #pos = box1.getPosition()
        #print("{}\t{}\t{}\t{}".format(time, pos[0], pos[1], pos[2]))

        print joint.getAngle()

        # Update json primitives
        #if int(time / TIME_STEP) % divisor == 0:
        update_json(bodies)

    json_filename = "box_drop.json"
    write_json(json_filename)


if __name__ == '__main__':
    main()
