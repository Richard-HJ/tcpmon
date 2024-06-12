#!/bin/python3


import sys
import argparse
import os
import pathlib
import datetime
import subprocess
import time

# iperf 5001 iperf3 5201
tcp_port = 5201
# 1st core to use
cpu_core = 6
# wait times 
sleep_time_loop = 4

#
#prog = "/home/richard/tcpmon/tcpmon_mon "
#prog = "/home/richard/tcpmon-2.2.1/tcpmon_mon "
# for DTNlon
#prog = "/home/richard/tcpmon-2.2.1_raw/tcpmon_mon "
prog = "/home/richard/tcpmon-transit_times/tcpmon_transit "

#define lists
proc_list = []
outfile_list = []
cmd_list = []



def init_argparse() -> argparse.ArgumentParser:
# ------------------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTION] ...",
        description="Use tcpmon to send a set of n concurrent transfers over a time --testTime. There is a tcpmon_mon for each flow in the set. --oneTest allows to run just one set of n flows"
    )
    parser.add_argument(
        "--bufsize", action='store',
        default=1048576, type=int,
        help='length in bytes of the send chunk'
    )
    parser.add_argument(
        "--numTrials", action='store',
        default=3600, type=int,
        help='Number of trials in the stress test'
    )
    parser.add_argument(
        "--oneTest", action="store_true",
        help="run just one set of n concurrent files"
    )

    parser.add_argument(
        "-A", action='store', 
        default=6, type=int,
        help='Affinity core number for start flow '
    )
    parser.add_argument(
        "-d", action='store', 
        default='a.b.c.d',
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
        help='Num of concurrent flows'
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
        "-l", action='store', 
        default=0, type=int,
        help='number of messages to send in the test'
    )
    parser.add_argument(
        "-u", action='store',
        default=5201, type=int,
        help='TCP port number'
    )
    parser.add_argument(
        "-w", action='store',
        default=0, type=int,
        help='time between msg in usec'
    )
    parser.add_argument(
        "-N", action='store', 
        default=1, type=int,
        help='mumber of trials on 1 TCP link'
    )
    parser.add_argument(
        "-S", action='store', 
        default=0, type=int,
        help='TCP socket buffer size <0> = TCP autotune'
    )
    parser.add_argument(
        "-W", action='store', 
        default=0, type=int,
        help='Time to wait between trials in sec on 1 TCP link'
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

def setup_tcpmon_mon(args, num_flows, log_file_prefix, cmd_test):
# ------------------------------------------------------------------------------
#Set up the core affinity, logfile name
#make the tcpmon command 
# outputs
#    cmd_list[]
#    outfile_list[]

    cmd_list = []
    outfile_list = []
    cmd_list.clear()
    outfile_list.clear()
   
    # loop over the number of TCP flows
    index = 1
    while index <= num_flows:
        port = tcp_port + index - 1
        #even number affinity
        #affinity = cpu_core + 2*(index - 1)
        #OR
        # consecutive core affinity
        affinity = cpu_core+index - 1
        
        # make the log file
        log_file = log_file_prefix+"_"+str(num_flows)+str(index)+".txt"
        # open file for writing
        outfile = open(log_file,'wt')
        outfile_list.append(outfile)
                
        # make the command
        # 2>&1 Redirect not needed as stderr and stdout directed by Popen()
        # for tcpmon_mon
        #cmd = cmd_test+ " -d "+args.d+ " -u "+str(port)+ " -A "+str(affinity)+ " -i "+str(args.i)+ " -p "+str(args.p)+ " -l "+str(args.l)+ " -w "+str(args.w)+ " -S "+str(args.S)
        # for tcpmon_transit
        cmd = cmd_test+ " -d "+args.d+ " -u "+str(port)+ " -A "+str(affinity)+ " -i "+str(args.i)+ " -p "+str(args.p)+ " -l "+str(args.l)+ " -w "+str(args.w)+ " -S "+str(args.S)+ " -N "+str(args.N)+ " -W "+str(args.W)
        print(cmd)
        print(date_time, file=outfile)
        print(cmd, file=outfile)
        cmd_list.append(cmd)
        
        #print("Time_s; num_errors; Time_startup_s; Time_sending_s; Total_Time_s; Data_rate_GBytes/s; Data_rate_Gbit/s; body_length; bytes_sent; total_retrans; Total_runtime_s", file=outfile)
        
        index = index + 1
        
    return cmd_list, outfile_list

    
def subproc_waitend(num_flows, start_time, trial_num, cmd_list, outfile_list):
# ------------------------------------------------------------------------------
# run the command from cmd_list as sub-process(es) depending on num_flows
# wait for the subprocess to end
# sort and re-format output and write to log file
# trial_num  used to select & format output
#                 >= 1 if no headings
#                 == 0 if print headings
# Note: order of creating sub-process matches the order of the log files.

    outs_list = []
    errs_list = []
    # loop over creating the sub-process each with its own command
    index = 0
    proc_list.clear()
    outs_list.clear()
    errs_list.clear()
    trial_start_time = time.perf_counter()
    while index < num_flows:
        # get the command
        cmd = cmd_list[index]
        if trial_num != 0:
            cmd = cmd + " -q "
        
        # create the shell that runs the sub-process
        # *** see https://www.golinuxcloud.com/python-subprocess/
        #
        proc_list.append(subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True) )
        
        index = index + 1

    # wait for the subprocess to end
    for proc in proc_list:
        #print(proc)
        print("PID "+ str(proc.pid) )
        #print(proc.returncode)
        outs, errs = proc.communicate(input=None, timeout=None)
        #outs, errs = proc.communicate()
        #print(outs)
        outs_list.append(outs)
        errs_list.append(errs)
    trial_end_time = time.perf_counter()
    trial_time = trial_end_time - trial_start_time

    #add info to the program output
    index = 0
    for proc in proc_list:
        errs = errs_list[index]
        outs = outs_list[index]
        
        # check for errors
        err_lines = errs.splitlines()
        num_errs = len(err_lines)

        if num_errs == 0:
            # select required data from the sub-process output
            lines = outs.splitlines()
            for line in lines:            
                # add time and error count
                full_line = str(start_time)+ "; " + str(trial_num) + "; " + str(num_errs) + "; " + str(trial_time) + "; " + line
                # write to log_file
                print(full_line, file=outfile_list[index])
        # have error
        else:
            # add time and error count
            full_line = str(start_time)+ "; " + str(trial_num) + "; " + str(num_errs) + "; " + str(trial_time) + "; " + err_lines[0]
            # write to log_file
            print(full_line, file=outfile_list[index])
                
        index = index + 1
        
    print("------")

    return




def main() -> None:
    global interval
    global date_time
    global start_time
    global cpu_core

    print(time.get_clock_info('perf_counter'))
    
    parser = init_argparse()
    args = parser.parse_args()
    #check correct args given
    if args.n == 0:
        parser.print_help()

    # sort out aguments
    num_flows = args.n
    cpu_core = args.A
    ipv6 = ' '
    if(args.ipv6):
        ipv6 = " -6 "

    # get the home directoy
    home_directory = os.path.expanduser( '~' )
    # get the current directoy
    log_file_dir = os.getcwd()

    # prog path/app defined above
    #cmd_test = prog
    cmd_test = prog + ipv6

    # get the date for the filename  
    curent_date = datetime.datetime.now()
    day = curent_date.strftime("%d")
    month = curent_date.strftime("%b")
    year = curent_date.strftime("%y")
    date_time = curent_date.strftime("%d-%b-%Y %H:%M:%S")

    log_file_prefix = log_file_dir+"/"+args.o+"_tcpmon_"+day+month+year

    # create and open log files create commands
    cmd_list, outfile_list = setup_tcpmon_mon(args, num_flows, log_file_prefix, cmd_test)

    # record time the test run started
    start_time = time.time()

    if(args.oneTest):
        # run tcpmon_mon for a singe set of concurrent flows
#                 >= 1 if no headings  == 0 if print headings
        subproc_waitend(num_flows, start_time, 0, cmd_list, outfile_list)
        #print (time.time() - start_time)
    else:
        # run the stress test
        index = 0
        run_time = 0
        while index < args.numTrials:
            #run tcpmon_mon
            print("Trial " + str(index) + " run_time " + str(run_time) )
            subproc_waitend(num_flows, run_time, index, cmd_list, outfile_list)
            # wait after sending flow. Then loop sending
            time.sleep(sleep_time_loop)

            index = index + 1
            run_time = int(time.time() - start_time)


if __name__ == "__main__":
    main()
