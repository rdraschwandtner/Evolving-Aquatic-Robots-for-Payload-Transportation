#!/usr/bin/env python
"""
A simple simulation of two constrained boxes dropping onto a sphere.

Notes:
- The two boxes are connected by a hinge joint, and they do not 'collide'
with each other.
- A json file is saved with all of the information needed to generate a
visualization.
"""

import ode
import json

#
# Simulation parameters
#
TIME_STOP = 20
TIME_STEP = 0.05
GRAVITY = (0, -9.81, 0)

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

#
# Visualization filename and object
#
json_filename = "box_drop.json"
json_object = {}
json_object["time_start"] = 0
json_object["time_stop"] = TIME_STOP
json_object["time_step"] = TIME_STEP
json_object["primitives"] = []


def write_json(fname, pretty=True):
    """Write the dynamics data to a json file.
    """

    if pretty:
        # json data with human-readable formatting
        text = json.dumps(json_object, sort_keys=True, indent=4)
    else:
        # The 'separators' keyword argument reduces the filesize by getting
        # rid of the spaces and newlines.
        text = json.dumps(json_object, separators=(',', ':'))

    with open(fname, 'w') as json_file:
        json_file.write(text)


def update_json():
    """Update the JSON data object.

    Order of quaternion coordinates is different for different libraries
        THREE : ( x, y, z, w )
        ODE   : ( w, x, y, z )
        TRANS : ( w, x, y, z)
    """

    # Write the position and rotation (quaternion) of each object.
    for body, bidx in zip(bodies, range(len(bodies))):
        p = body.getPosition()
        q = body.getQuaternion()

        json_object["primitives"][bidx]["dynamics"]["position"].append(p)
        json_object["primitives"][bidx]["dynamics"][
            "quaternion"].append([q[1], q[2], q[3], q[0]])


def create_json_primitive(shape, dims, color):
    """Create a json visualization primitive.
    """

    prim = {}
    prim["geometry"] = {}
    prim["geometry"]["shape"] = shape
    if shape == "sphere":
        prim["geometry"]["radius"] = dims
    else:
        prim["geometry"]["sizeX"] = dims[0]
        prim["geometry"]["sizeY"] = dims[1]
        prim["geometry"]["sizeZ"] = dims[2]
    prim["dynamics"] = {}
    prim["dynamics"]["position"] = []
    prim["dynamics"]["quaternion"] = []
    prim["dynamics"]["visible"] = [[0., TIME_STOP]]
    prim["material"] = {}
    prim["material"]["type"] = "phong"
    prim["material"]["color"] = color
    return prim


def add_sphere(radius, density, position, quaternion=None, color=0x772277):
    """Create a sphere object, inlcuding visualization parameters.
    """

    # Create rigid body and set position and rotation
    sphere = ode.Body(world)
    sphere.setPosition(position)
    if quaternion is not None:
        sphere.setQuaternion(quaternion)

    # Set the mass based on density
    mass = ode.Mass()
    mass.setSphere(density, radius)
    sphere.setMass(mass)

    # Create the collision geometry
    geom = ode.GeomSphere(space, radius)
    geom.setBody(sphere)
    geoms.append(geom)

    # Create the json primitive
    vis_prim = create_json_primitive("sphere", radius, color)
    json_object["primitives"].append(vis_prim)

    bodies.append(sphere)
    return sphere


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

    geom = ode.GeomBox(space, lengths=size)
    geom.setBody(cube)
    geoms.append(geom)

    vis_prim = create_json_primitive("cube", size, color)
    json_object["primitives"].append(vis_prim)

    bodies.append(cube)
    return cube


def add_hingejoint(rb1, rb2, anchor, axis, fmax=10, lo=-4, hi=4):
    """Create a hinge joint between two rigid bodies.
    """

    # Create the joint and attach it to both rigid bodies
    joint = ode.HingeJoint(world)
    joint.attach(rb1, rb2)

    # Set the anchor and axis of the hinge joint
    joint.setAnchor(anchor)
    joint.setAxis(axis)

    # Set hinge parameters
    joint.setParam(ode.ParamFMax, fmax)
    joint.setParam(ode.ParamLoStop, lo)
    joint.setParam(ode.ParamHiStop, hi)

    joints.append(joint)
    return joint


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
        c.setBounce(0.2)
        c.setMu(5000)
        j = ode.ContactJoint(world, contact_group, c)
        j.attach(geom1.getBody(), geom2.getBody())


def main():

    # Create a sphere
    sphere_radius = 50
    sphere_position = (0., sphere_radius * 1.05, 0.)
    sphere_density = 1.
    sphere = add_sphere(sphere_radius, sphere_density, sphere_position)

    # Create two boxes and constrain them with a hinge joint
    box_size = (50, 50, 50)
    box_density = 1.
    box_position_middle = (0., 200., box_size[2] * 0.5)

    box1_position = list(box_position_middle)
    box1_position[2] += box_size[2]
    box1 = add_cube(box_size, box_density, box1_position)

    box2_position = list(box_position_middle)
    box2_position[2] -= box_size[2]
    box2 = add_cube(box_size, box_density, box2_position)

    box_anchor = box_position_middle
    box_axis = (1., 0., 0.)
    box_joint = add_hingejoint(box1, box2, box_anchor, box_axis)

    # The two boxes will not collide if the following line is uncommented
    no_collide.append((box1, box2))

    time = 0.0
    while time < TIME_STOP:

        # Collide all objects
        space.collide((world, contact_group), near_callback)

        # Advance the physics world through time
        world.step(TIME_STEP)
        time += TIME_STEP

        # Reset collisions
        contact_group.empty()

        # Print to command-line (for debugging/plotting)
        pos = box1.getPosition()
        print("{}\t{}\t{}\t{}".format(time, pos[0], pos[1], pos[2]))

        # Update json primitives
        update_json()

    write_json(json_filename)


if __name__ == '__main__':
    main()
