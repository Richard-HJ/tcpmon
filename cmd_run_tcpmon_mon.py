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
        description="Run n tcpmon_mon 1 for each TCP flow."
    )
    parser.add_argument(
        "-d", action='store', 
        help='the destination IP name or IP address a.b.c.d'
    )
    parser.add_argument(
        "-i", action='store', 
        default=2, type=int,
        help='time interval between periodic reports in sec'
    )
    parser.add_argument(
        "-n", action='store', 
        default=0, type=int,
        help='Num of TCP flows'
    )
    parser.add_argument(
        "-o", action='store', 
        default='Test',
        help='file name to prepend'
    )
    parser.add_argument(
        "-p", action='store',
        default=98380, type=int,
        help='length in bytes of the msg to send'
    )
    parser.add_argument(
        "-t", action='store', 
        default=0, type=int,
        help='time to run the test in sec'
    )
    parser.add_argument(
        "-w", action='store', 
        default=0, type=int,
        help='time between msg in usec'
    )
    parser.add_argument(
        "-S", action='store', 
        default=0, type=int,
        help='TCP socket buffer size<0>'
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
prog = "tcpmon_mon -C "

	
def main() -> None:
    parser = init_argparse()
    args = parser.parse_args()
    #check correct args given
    if args.n == 0:
        parser.print_help()

    # sort out aguments
    num_flows = args.n
    interval = ' -i '+ str(args.i)
    if(args.i == 0):
        interval =' '

    # get the home directoy
    home_directory = os.path.expanduser( '~' )
    # get the current directoy
    log_file_dir = os.getcwd()

    cmd_test =  os.path.join(log_file_dir, prog)

    # get the date for the filename  
    curent_date = datetime.datetime.now()
    day = curent_date.strftime("%d")
    month = curent_date.strftime("%b")
    year = curent_date.strftime("%y")

	
    # loop over the number of TCP flows
    index = 0
    while index < num_flows:
        port = tcp_port+ index
        # consecutive
        affinity = cpu_core+index
        affinity = 1 << affinity

        #print('{}, {}, {}'.format(index, buf_size, file_postfix[index] ))
        # make the log file
        log_file = log_file_dir+"/"+args.o+"_tcpmon_mon_"+day+month+year+"_"+str(num_flows)+str(index)+".txt"
        #print (log_file)
        # open file for writing
        outfile = open(log_file,'wt')
        
        # make the command
        # 2>&1 Redirect stderr to "where stdout is currently going i.e.log_file
        cmd = cmd_test+ " -d "+args.d+ " -u "+str(port)+ " -a "+hex(affinity)+ interval+ " -p "+str(args.p)+ " -t "+str(args.t)+ " -w "+str(args.w)+ " -S "+str(args.S)+ " > "+log_file+" 2>&1 &"

        print(cmd)
        run_subproc(cmd) 
        #output, error = run_command(cmd) 
        #outfile.write('{}\n'. format(output))		
        index = index + 1
	

if __name__ == "__main__":
    main()
