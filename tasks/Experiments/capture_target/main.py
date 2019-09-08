import os
import sys
sys.path.insert(0, '../../') # must be used in order to include other modules from the main directory!
from ODESystem import ODEManager
import GlobalVarWorkaround
import simulation
import operator
import argparse
import math
import array
import random


################################################################################
# Project specific store and load functions
################################################################################
def logGenome(file_prefix,output_path,genome):
    """ Log a genome to an output file. """
    with open(output_path+"/best_individuals/"+file_prefix+"_morph_genome.dat","w") as f:
        f.write(str(genome)) #+","+str(self.genomes[NEAT_id][1]))

def store_gen_stats(fname, generation, individualindices, fitnesses):
    ''' Write generation statistics to a CSV file:
        generation i; individualindex i; fitness i
        generation i+1; individualindex i+1; fitness i+1
    '''

    f = 0
    if os.path.isfile(fname):
        # file exists - append new conctent
        f = open(fname, 'a')
    else:
        f = open(fname, 'w')
        # write header
        f.write('generation;index;fitness\n')

    # write body
    for idx in individualindices:
        f.write(`generation`+';'+`idx`+';'+`fitnesses[idx]`+'\n')    
    f.close()
################################################################################


################################################################################
# Main entry point
################################################################################
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    #parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
    #parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
    parser.add_argument("--output_path", type=str, default="./", help="Output path")
    #parser.add_argument("--num_joints", type=int, default=8, help="Number of robot joints.")
    #parser.add_argument("--validator", action="store_true", help="Validate current results.")
    parser.add_argument("--run_num", type=int, default=0, help="Run Number")
    #parser.add_argument("--eval_time", type=float, default=20., help="Simulation time for an individual.")
    parser.add_argument("--targetposx", type=float, default=20., help="Set the target x position in the world")
    parser.add_argument("--targetposy", type=float, default=20., help="Set the target y position in the world")
    GlobalVarWorkaround.args = parser.parse_args()

    print GlobalVarWorkaround.args


    log_frames = True

    #num_joints = GlobalVarWorkaround.args.num_joints
    num_joints = 16

    run_num = GlobalVarWorkaround.args.run_num
    #eval_time = GlobalVarWorkaround.args.eval_time
    eval_time = 30
    dt = .02
    n = 4


    GlobalVarWorkaround.man = ODEManager(simulation.near_callback, stepsize=dt/n, log_data=log_frames, fluid_dynamics=1, gravity=0, run_num=run_num)

    GlobalVarWorkaround.worm = simulation.Worm(man=GlobalVarWorkaround.man,num_joints=num_joints,logging=log_frames)
    targetposx = GlobalVarWorkaround.args.targetposx
    targetposz = GlobalVarWorkaround.args.targetposy

    print 'target @ x,z (' + `targetposx` + ',' + `targetposz` + ')'
    GlobalVarWorkaround.target = simulation.Target(man=GlobalVarWorkaround.man, base_pos=[targetposx,1,targetposz])

    GlobalVarWorkaround.man.log_world_setup(eval_time,ind_num=run_num)

    amps = 0.2
    frqs = 2.0
    phss = num_joints/(2*math.pi)
    bias = 0.

    #print '--amps:' + str(amps) + ' frqs:'  + str(frqs) + ' phss:' + str(phss)
    jointctrlers = [simulation.JointActuationController(amplitude=amps, frequency=frqs, phaseshift=phss, offset=bias) for i in range(num_joints-4)]
    simulationobj = simulation.Simulation(eval_time, dt=dt, n=n, man=GlobalVarWorkaround.man, worm=GlobalVarWorkaround.worm, jointctrlers=jointctrlers, target=GlobalVarWorkaround.target)
    
    fit = simulationobj.evaluate_individual()
    print '--fit=' + str(fit)
