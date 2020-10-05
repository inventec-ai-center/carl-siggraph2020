import sys
sys.path.append('../DeepMimic/')
sys.path.append('scripts/')
import subprocess
from util.arg_parser import ArgParser
from util.logger import Logger

def main():
    # Command line arguments
    args = sys.argv[1:]
    arg_parser = ArgParser()
    arg_parser.load_args(args)

    num_workers = arg_parser.parse_int('num_workers', 1)
    assert(num_workers > 0)

    Logger.print('Running with {:d} workers'.format(num_workers))
    cmd = 'mpiexec --allow-run-as-root -n {:d} python CARL_Optimizer.py '.format(num_workers)
    cmd += ' '.join(args)
    Logger.print('cmd: ' + cmd)
    subprocess.call(cmd, shell=True)
    return

if __name__ == '__main__':
    main()
