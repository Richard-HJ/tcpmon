#!/bin/python3


import sys
import argparse
import os
import pathlib
import datetime
import subprocess
import time


tcp_port = 5201
cpu_core = 6

#
#prog = "/home/richard/tcpmon-2.2.1/tcpmon_mon -C "
# for DTNlon
prog = "/home/richard/tcpmon-2.2.1_raw/tcpmon_mon -C "

proc_list = []
outfile_list = []
cmd_list = []


def init_argparse() -> argparse.ArgumentParser:
# ------------------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTION] ...",
        description="Run a set of parallel TCP flow tests with 1flow then 2flows up to n flows. There is a tcpmon_mon 1 for each TCP flow. --oneTest allows to run just one set of n TCP flows"
    )
    parser.add_argument(
        "--bufsize", action='store',
        default=1048576, type=int,
        help='length in bytes of the send chunk'
    )
    parser.add_argument(
        "--testTime", action='store',
        default=360, type=int,
        help='Time to run the stress test in sec'
    )
    parser.add_argument(
        "--oneTest", action="store_true",
        help="run just one set of n concurrent files"
    )

    parser.add_argument(
        "-A", action='store', 
        default=0, type=int,
        help='Affinity core number for start flow '
    )
    parser.add_argument(
        "-d", action='store', 
        default='a.b.c.d',
        help='the destination IP name or IP address a.b.c.d'
    )
    parser.add_argument(
        "-e", action='store', 
        default=0, type=int,
        help='End flow number s< e <=n'
    )
    parser.add_argument(
        "-i", action='store', 
        default=2, type=int,
        help='time interval between periodic reports in sec'
    )
    parser.add_argument(
        "-n", action='store', 
        default=0, type=int,
        help='Num of concurrent files'
    )
    parser.add_argument(
        "-o", action='store', 
        default='Test',
        help='file name to prepend to output'
    )
    parser.add_argument(
        "-p", action='store',
        default=98380, type=int,
        help='length in bytes of the msg to send'
    )
    parser.add_argument(
        "-s", action='store', 
        default=1, type=int,
        help='Start flow number 1<= s <n'
    )
    parser.add_argument(
        "-t", action='store', 
        default=0, type=int,
        help='Time to run the test in sec'
    )
    parser.add_argument(
        "-w", action='store',
        default=0, type=int,
        help='time between msg in usec'
    )
    parser.add_argument(
        "-S", action='store', 
        default=0, type=int,
        help='TCP socket buffer size <0>m= TCP autotune'
    )
    parser.add_argument(
        "-6", "--ipv6", action="store_true",
        help='Select IPv6 only'
    )
   
    parser.add_argument(
        "-v", "--version", action="version",
        version=f"{parser.prog} version 1.0.0"
    )

    return parser


def setup_tcpmon(args, num_flows, start_flow, end_flow, log_file_prefix, cmd_test):
# ------------------------------------------------------------------------------
#Set up the core affinity, logfile name, dst filename
#make the davix_put command 

    global proc_list 
    global outfile_list
    global cmd_list

    outfile_list.clear()
    cmd_list.clear()

    # loop over the number of TCP flows
    index = 1
    while index <= num_flows:
        port = tcp_port + index - 1
        
        #even number affinity
        #affinity = cpu_core + 2*(index - start_flow)
        #OR
        # consecutive core affinity
        affinity = cpu_core+index - start_flow
        #cpubind = "numactl --physcpubind="+str(affinity)+" "
        affinity = 1 << affinity
        
        # make the log file
        log_file = log_file_prefix+"_"+str(num_flows)+str(index)+".txt"
        # open file for writing
        outfile = open(log_file,'wt')
        outfile_list.append(outfile)
        
        # make the command
        # 2>&1 Redirect not needed as stderr and stdout directed by Popen()
        #cmd = cmd_test+ " -d "+args.d+ " -u "+str(port)+ " -a "+hex(affinity)+ interval+ " -p "+str(args.p)+ " -t "+str(args.t)+ " -w "+str(args.w)+ " -S "+str(args.S) + " > "+log_file+" 2>&1"
        cmd = cmd_test+ " -d "+args.d+ " -u "+str(port)+ " -a "+hex(affinity)+ " -i "+str(args.i)+ " -p "+str(args.p)+ " -t "+str(args.t)+ " -w "+str(args.w)+ " -S "+str(args.S) + " > " + log_file
        print(cmd)
        #print(date_time, file=outfile)
        #print(cmd, file=outfile)
        cmd_list.append(cmd)
        
        #print("Time_startup_s; Time_sending_s; Total_Time_s; Data_rate_GBytes/s; Data_rate_Gbit/s; body_length; bytes_sent; total_retrans; Total_runtime_s", file=outfile)
        
        index = index + 1
        
    return 

def run_tcpmon(num_flows):
# ------------------------------------------------------------------------------
# run the davix_put command as sub-process(es) depending on num_flows
# wait for the subprocess to end
# sort and re-format output and write to log file
# Note: order of creating sub-process matches the order of the log files.


    # loop over the number of TCP flows
    index = 0
    proc_list.clear()
    while index < num_flows:
        # get the command
        cmd = cmd_list[index]
        # create the shell that runs the sub-process
        # *** see https://www.golinuxcloud.com/python-subprocess/
        #
        proc_list.append(subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True) )
        
        index = index + 1


    # wait for the subprocess to end
    print("wait for the subprocess to end")        
    #print(proc_list)
    index = 0
    for proc in proc_list:
        #print(proc)
        print("PID "+ str(proc.pid) )
        #print(proc.returncode)
        outs, errs = proc.communicate(input=None, timeout=None)
        
        # check for errors
        err_lines = errs.splitlines()
        num_errs = len(err_lines)
        print("err: "+str(num_errs))

        if num_errs == 0:
            # select required data from the sub-process output
            lines = outs.splitlines()
        else:
            #print("errs:")
            #print(errs)
            # write to log_file
            print(err_lines[0], file=outfile_list[index])
        
        #print("------")
        
        index = index + 1
                
    return 

	
def main() -> None:
# ------------------------------------------------------------------------------
    global date_time
    
    parser = init_argparse()
    args = parser.parse_args()
    #check correct args given
    if args.n == 0:
        parser.print_help()

    # sort out aguments
    num_flows = args.n
    start_flow = args.s
    end_flow = args.e
    if end_flow == 0:
        end_flow = num_flows
    ipv6 = ' '
    if(args.ipv6):
        ipv6 = " -6 "

    # get the home directoy
    home_directory = os.path.expanduser( '~' )
    # get the current directoy
    log_file_dir = os.getcwd()

    # prog path/app defined above
    #cmd_test =  os.path.join(log_file_dir, prog)
    cmd_test = prog + ipv6

    # get the date for the filename  
    curent_date = datetime.datetime.now()
    day = curent_date.strftime("%d")
    month = curent_date.strftime("%b")
    year = curent_date.strftime("%y")
    date_time = curent_date.strftime("%d-%b-%Y %H:%M:%S")
 
    log_file_prefix = log_file_dir+"/"+args.o+"_tcpmon_multiflow_"+day+month+year
    

    if(args.oneTest):
        # singe set of concurrent file flows
        # create and open log files create commands
        setup_tcpmon(args, num_flows, start_flow, end_flow, log_file_prefix, cmd_test)
        run_tcpmon(num_flows)
    else:
        # run the multi-flow test
        n_flows = start_flow
        while n_flows <= end_flow:
            # create and open log files create commands
            setup_tcpmon(args, n_flows, start_flow, end_flow, log_file_prefix, cmd_test)
            run_tcpmon(n_flows)
            print("-----")
                
            n_flows = n_flows + 1
	

if __name__ == "__main__":
    main()
