import argparse
import os
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from ODESystem import ODEManager
import GlobalVarWorkaround
import box_simu
import operator

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--run_num", type=int, default=0, help="Run Number")
    parser.add_argument("--log_frames",action="store_true",help="Save the frames to a folder.")
    parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
    parser.add_argument("--applyforcetill_time", type=float, default=10., help="Apply force till a certain time")
    args = parser.parse_args()

    print args

    log_frames = args.log_frames
    eval_time = args.eval_time
    run_num = args.run_num
    dt = .02
    n = 4
    forcetill = args.applyforcetill_time

    GlobalVarWorkaround.man = ODEManager(box_simu.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)
    
    box = box_simu.Box(man=GlobalVarWorkaround.man, desnsity=50., base_pos=[.0,1.,.0])

    # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
    # Must be placed after creating the quadruped.
    if log_frames:
        GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

    simulation = box_simu.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, box=box, applyforcetill=forcetill)

    fit = simulation.evaluate_individual()

    print fit

