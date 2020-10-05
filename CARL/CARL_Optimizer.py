import numpy as np
import sys
sys.path.append('../DeepMimic/')
sys.path.append('scripts/')
from carl_env import CarlEnv
from rl_world import RLWorld
from util.logger import Logger
from util.arg_parser import ArgParser
from CARL_Runner import update_world, update_timestep, build_world
import util.mpi_util as MPIUtil

args = []
world = None
iter = 0

def run(max_iters=None):
    global update_timestep
    global world
    global iter

    done = False
    while not (done):
        update_world(world, update_timestep)
        done = world.isDone()

        iter += 1
        if max_iters: done = iter >= max_iters

    return

def shutdown():
    global world

    Logger.print('Shutting down...')
    world.shutdown()
    return

def main():
    global args
    global world

    # Command line arguments
    args = sys.argv[1:]

    arg_parser = ArgParser()
    arg_parser.load_args(args)
    max_iters = arg_parser.parse_int('max_iters', None)

    world = build_world(args, enable_draw=False)

    run(max_iters)
    shutdown()

    return

if __name__ == '__main__':
    main()
