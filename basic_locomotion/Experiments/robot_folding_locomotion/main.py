import os
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from ODESystem import ODEManager
import GlobalVarWorkaround
import simulation
import operator
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--num_joints", type=int, default=8, help="Number of robot joints.")
    GlobalVarWorkaround.args = parser.parse_args()

    num_joints = GlobalVarWorkaround.args.num_joints

    log_frames = True
    eval_time = 20
    run_num = 0
    dt = .02
    n = 4


    GlobalVarWorkaround.man = ODEManager(simulation.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)

    GlobalVarWorkaround.worm = simulation.Worm(man=GlobalVarWorkaround.man,num_joints=num_joints,logging=log_frames)

    #targetposx = -8
    #targetposy = 1

    targetposx = 8
    targetposy = 8

    print 'target @ x,y (' + `targetposx` + ',' + `targetposy` + ')'
    GlobalVarWorkaround.target = simulation.Target(man=GlobalVarWorkaround.man, base_pos=[targetposx,1,targetposy])
    

    # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
    # Must be placed after creating the quadruped.
    if log_frames:
        GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

    simulation = simulation.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm)

    fit = simulation.evaluate_individual()

    print fit

