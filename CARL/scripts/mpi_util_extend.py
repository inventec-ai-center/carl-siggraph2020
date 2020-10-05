import numpy as np
from mpi4py import MPI

ROOT_PROC_RANK = 0

def gather(x):
    return MPI.COMM_WORLD.gather(x, root=ROOT_PROC_RANK)
