
import json

TIME_STOP = 10
TIME_STEP = 10

# used to upsacel the visualization
VIS_SCALE = 50

#
# Visualization filename and object
#
json_filename = "box_drop.json"
json_object = {}
json_object["time_start"] = 0
json_object["time_stop"] = TIME_STOP
json_object["time_step"] = TIME_STEP
json_object["primitives"] = []


def visualizationJSON_init (time_stop, time_step):
    global TIME_STOP
    TIME_STOP = time_stop
    json_object["time_stop"] = TIME_STOP
    global TIME_STEP
    TIME_STEP = time_step
    json_object["time_step"] = TIME_STEP


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


def update_json(bodies):
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

        p = [c * VIS_SCALE for c in p]

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
        prim["geometry"]["radius"] = dims * VIS_SCALE
    else:
        prim["geometry"]["sizeX"] = dims[0] * VIS_SCALE
        prim["geometry"]["sizeY"] = dims[1] * VIS_SCALE
        prim["geometry"]["sizeZ"] = dims[2] * VIS_SCALE
    prim["dynamics"] = {}
    prim["dynamics"]["position"] = []
    prim["dynamics"]["quaternion"] = []
    prim["dynamics"]["visible"] = [[0., TIME_STOP]]
    prim["material"] = {}
    prim["material"]["type"] = "phong"
    prim["material"]["color"] = color
    return prim