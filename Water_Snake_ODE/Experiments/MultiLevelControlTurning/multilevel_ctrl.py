import argparse
import os
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from ODESystem import ODEManager
import GlobalVarWorkaround
import ribosomalrobot_ctrl_simu_stop
import operator

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
    parser.add_argument("--output_path", type=str, default="./", help="Output path")
    parser.add_argument("--run_num", type=int, default=0, help="Run Number")
    parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
    parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
    parser.add_argument("--validator", action="store_true", help="Validate current results.")
    parser.add_argument("--targetposx", type=float, default=10., help="Set the target x position in the world")
    parser.add_argument("--targetposy", type=float, default=10., help="Set the target y position in the world")
    parser.add_argument("--rdensity", type=float, default=50., help="Robot body density")
    parser.add_argument("--freq", type=float, default=0.6, help="Robot sine frequency")
    parser.add_argument("--numofjoints", type=int, default=7, help="Number of joints in the robot")
    parser.add_argument("--length", type=float, default=10., help="length of the robot")
    args = parser.parse_args()

    print args

    log_frames = args.log_frames
    eval_time = args.eval_time
    run_num = args.run_num
    output_path = args.output_path
    dt = .02
    n = 4
    numofjoints = args.numofjoints
    length = args.length


    GlobalVarWorkaround.man = ODEManager(ribosomalrobot_ctrl_simu_stop.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)
    density = args.rdensity

    GlobalVarWorkaround.worm = ribosomalrobot_ctrl_simu_stop.Worm(man=GlobalVarWorkaround.man, num_joints=numofjoints, morphology={'density':density,'xlength':length},logging=log_frames, log_path=output_path, logfileprefix='rep' + str(run_num))
    
    targetposx = args.targetposx
    targetposy = args.targetposy
    print 'target @ x,y (' + `targetposx` + ',' + `targetposy` + ')'
    GlobalVarWorkaround.target = ribosomalrobot_ctrl_simu_stop.Target(man=GlobalVarWorkaround.man, base_pos=[targetposx,1,targetposy])


    # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
    # Must be placed after creating the quadruped.
    if log_frames:
        GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

    actuationfreq = args.freq
    simulation = ribosomalrobot_ctrl_simu_stop.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm, target=GlobalVarWorkaround.target, actuationfreq=actuationfreq)

    fit = simulation.evaluate_individual()

    print fit

