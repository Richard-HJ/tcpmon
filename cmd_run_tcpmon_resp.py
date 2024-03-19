#!/bin/python3


import sys
import argparse
import os
import pathlib
import datetime
import subprocess



def init_argparse() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTION] ...",
        description="Run n tcpmon_resp 1 for each TCP flow."
    )
    parser.add_argument(
        "-n", action='store', 
        default=0, type=int,
        help='Num of TCP flows'
    )
   
    parser.add_argument(
        "-v", "--version", action="version",
        version=f"{parser.prog} version 1.0.0"
    )
    return parser

def run_command(cmd, ignore_stderr = False):
# ------------------------------------------------------------------------------
# best to use the subprocess module over os.system()
    proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
# get the contents of the pipes
#    try:
    outs, errs = proc.communicate()
#    except subprocess.TimeoutExpired:
#            outs, errs = proc.communicate()

    return [str(outs,'UTF-8'), str(errs,'UTF-8')]

def run_subproc(cmd, ignore_stderr = False):
# ------------------------------------------------------------------------------
# best to use the subprocess module over os.system()
    proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

    return 

tcp_port = 5201
cpu_core = 6
prog = "tcpmon_resp"
#CamDTN
#nic = "ens4f1"
#nic = "ens4f1np1"
# DTNlon
nic = "enp131s0f1"
	
def main() -> None:
    parser = init_argparse()
    args = parser.parse_args()
    # check correct args given
    if args.n == 0:
        parser.print_help()

    # sort out aguments
    num_flows = args.n

    # get the home directoy
    home_directory = os.path.expanduser( '~' )
    # get the current directoy
    log_file_dir = os.getcwd()

    cmd_test =  os.path.join(log_file_dir, prog)
	
    # loop over the number of TCP flows
    index = 0
    while index < num_flows:
        port = tcp_port+ index
        #even number affinity
        #affinity = cpu_core + 2*index
        # consecutive core affinity
        affinity = cpu_core+index

        affinity = 1 << affinity

        # make the command
        cmd = cmd_test+" -u "+str(port)+" -a "+hex(affinity)+" -I "+nic+ " > log"+str(index)+".txt 2>&1 &"
        print(cmd)
        run_subproc(cmd) 
        index = index + 1
	

if __name__ == "__main__":
    main()
