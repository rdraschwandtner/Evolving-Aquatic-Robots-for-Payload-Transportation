
import ode
import json
import math


def collision_CB(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms collide and
    creates contact joints if necessary.
    """

    world, contact_group, robot = args

    if robot.doCollide(geom1, geom2) == False:
        return

    # Check for collisions
    robot.contacts = ode.collide(geom1, geom2)

    # Create contact joints
    for c in robot.contacts:
        c.setBounce(1.)
        c.setMu(5000)
        j = ode.ContactJoint(world, contact_group, c)
        j.attach(geom1.getBody(), geom2.getBody())


def rotateVec3(mat, vec):
    return (
        vec[0] * mat[0] + vec[1] * mat[1] + vec[2] * mat[2],
        vec[0] * mat[3] + vec[1] * mat[4] + vec[2] * mat[5],
        vec[0] * mat[6] + vec[1] * mat[7] + vec[2] * mat[8]
    )


def scalarProd(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]


class Ribbot(object):

    """ODE simualation for a ribbon robot."""

    def __init__(self,
                 time_stop=20,
                 time_step=0.01,
                 segment_size=(1.5, 0.5, 0.5),
                 segment_density=0.1,
                 segment_count=8,
                 joint_lostop=-1.6,
                 joint_histop=1.6,
                 joint_fmax=10,
                 drag_coefficient=0.35,
                 vis_scale=100,
                 json_fname=None,
                 json_step=0.05
                 ):

        super(Ribbot, self).__init__()

        self.world = ode.World()
        self.world.setGravity((0.0, 0.0, 0.0))

        self.space = ode.Space()
        self.contact_group = ode.JointGroup()

        self.bodies = []
        self.geoms = []
        self.joints = []
        self.ignore_collide_pairs = []

        self.time = 0.0
        self.time_step = time_step
        self.time_stop = time_stop

        json_time_step = max(time_step, json_step)
        self.time_step_cnt = 0
        self.json_step_mod = int(round(json_time_step / self.time_step))

        self.drag_coefficient = drag_coefficient

        # All segments share the same surface parameters
        self.segment_surface_parameters = [
            {"area": segment_size[1] * segment_size[2], "norm": (1, 0, 0)},
            {"area": segment_size[0] * segment_size[2], "norm": (0, 1, 0)},
            {"area": segment_size[0] * segment_size[1], "norm": (0, 0, 1)},
            {"area": segment_size[1] * segment_size[2], "norm": (-1, 0, 0)},
            {"area": segment_size[0] * segment_size[2], "norm": (0, -1, 0)},
            {"area": segment_size[0] * segment_size[1], "norm": (0, 0, -1)}
        ]

        #
        # Create the ribbon robot
        #
        segment_mass = ode.Mass()
        segment_mass.setBox(segment_density, *segment_size)
        segment_position = [0.0, segment_size[1] * 1.5, 0.0]

        # Create rigid bodies
        for s in xrange(segment_count):
            seg = ode.Body(self.world)
            seg.setPosition(segment_position)
            seg.setMass(segment_mass)
            self.bodies.append(seg)

            geom = ode.GeomBox(self.space, lengths=segment_size)
            geom.setBody(seg)
            self.geoms.append(geom)

            segment_position[0] += segment_size[0]

        # Create hinge joints
        joint_position = [segment_size[0] / 2., segment_size[1] * 1.5, 0.0]
        for seg1, seg2 in zip(self.bodies, self.bodies[1:]):
            joint = ode.HingeJoint(self.world)
            joint.attach(seg1, seg2)
            joint.setAnchor(joint_position)
            joint.setAxis((0, 1, 0))

            joint.setParam(ode.ParamLoStop, joint_lostop)
            joint.setParam(ode.ParamHiStop, joint_histop)
            joint.setParam(ode.ParamFMax, joint_fmax)

            self.joints.append(joint)

            self.ignore_collide_pairs.append((seg1, seg2))
            joint_position[0] += segment_size[0]

        #
        # Setup Visualization
        #
        self.json_fname = json_fname
        if json_fname is not None:
            self.VIS_SCALE = vis_scale

            self.json_object = {}
            self.json_object["time_start"] = 0
            self.json_object["time_stop"] = time_stop
            self.json_object["time_step"] = json_time_step
            self.json_object["primitives"] = []

            for s in xrange(segment_count):
                seg = {}
                seg["geometry"] = {}
                seg["geometry"]["shape"] = "cube"
                seg["geometry"]["sizeX"] = segment_size[0] * self.VIS_SCALE
                seg["geometry"]["sizeY"] = segment_size[1] * self.VIS_SCALE
                seg["geometry"]["sizeZ"] = segment_size[2] * self.VIS_SCALE
                seg["dynamics"] = {}
                seg["dynamics"]["position"] = []
                seg["dynamics"]["quaternion"] = []
                seg["dynamics"]["visible"] = [[0., time_stop]]
                self.json_object["primitives"].append(seg)
            self.update_json()

    def __del__(self):
        if self.json_fname is not None:
            with open(self.json_fname, 'w') as json_file:
                json_file.write(json.dumps(self.json_object, separators=(',', ':')))

    def update_json(self):
        if self.json_fname is None or not (self.time_step_cnt % self.json_step_mod == 0):
            return

        for body, b in zip(self.bodies, range(len(self.bodies))):
            p = body.getPosition()
            p_scale = [coord * self.VIS_SCALE for coord in p]
            self.json_object["primitives"][b]["dynamics"]["position"].append(p_scale)

            q = body.getQuaternion()
            q_reorder = [q[1], q[2], q[3], q[0]]
            self.json_object["primitives"][b]["dynamics"]["quaternion"].append(q_reorder)

    def doCollide(self, geom1, geom2):
        for pair in self.ignore_collide_pairs:
            if geom1.getBody() in pair and geom2.getBody() in pair:
                return False
        return True

    def apply_hydrodynamics(self):
        C_d = self.drag_coefficient

        for body in self.bodies:
            vel = body.getLinearVel()
            rot = body.getRotation()

            for surface in self.segment_surface_parameters:
                area = surface["area"]
                norm = surface["norm"]

                rotated_norm = rotateVec3(rot, norm)
                project_vel_mag = scalarProd(rotated_norm, vel)

                hydro_force_mag = project_vel_mag * C_d * area
                if (hydro_force_mag < 0):
                    hydro_force_mag = 0

                hydro_force = (
                    -hydro_force_mag * rotated_norm[0],
                    -hydro_force_mag * rotated_norm[1],
                    -hydro_force_mag * rotated_norm[2]
                )
                body.addForce(hydro_force)

    def step(self):
        if self.time > self.time_stop:
            return

        self.apply_hydrodynamics()

        self.space.collide((self.world, self.contact_group, self), collision_CB)
        self.world.step(self.time_step)
        self.time += self.time_step
        self.time_step_cnt += 1

        self.contact_group.empty()

        self.update_json()

        # self.bodies[0].addForce((0, 0, -0.1))
        self.joints[0].setParam(ode.ParamVel, 0.1)


def actuate_robot_sinusoid(robot, amp, freq, bias, phase_offset, MAX=2 * math.pi):
    phase = 0.0
    for joint in robot.joints:
        cur_angle = joint.getAngle()
        tar_angle = amp * math.sin(2 * math.pi * freq * robot.time + phase) + bias
        err_angle = tar_angle - cur_angle

        vel = MAX if err_angle > 0 else -MAX
        if abs(err_angle) <= MAX * robot.time_step:
            vel *= abs(err_angle) / (MAX * robot.time_step)

        joint.setParam(ode.ParamVel, vel)
        phase += phase_offset


def actuate_robot_sinusoids(robot, amps, freqs, biases, phase_offsets, MAX=2 * math.pi):
    for joint, j in zip(robot.joints, xrange(len(robot.joints))):
        A, F, B, P = amps[j], freqs[j], biases[j], phase_offsets[j]

        cur_angle = joint.getAngle()
        tar_angle = A * math.sin(2 * math.pi * F * robot.time + P) + B
        err_angle = tar_angle - cur_angle

        vel = MAX if err_angle > 0 else -MAX
        if abs(err_angle) <= MAX * robot.time_step:
            vel *= abs(err_angle) / (MAX * robot.time_step)

        joint.setParam(ode.ParamVel, vel)


def test_ribbot():
    ribbot_fname = "/Users/msu/Desktop/ribbot.json"
    robot = Ribbot(json_fname=ribbot_fname)

    amplitude = math.radians(40)
    frequency = 2.0
    bias = 0.0
    phase_offset = math.radians(17)

    while robot.time < robot.time_stop:
        actuate_robot_sinusoid(robot, amplitude, frequency, bias, phase_offset)
        robot.step()


if __name__ == '__main__':
    test_ribbot()
