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
        description="Use tcpmon to measure TCP throughput as a function of TCP buffer size."
    )
    parser.add_argument(
        "-a", action='store', 
        default=' ',
        help='Set affinity <cpu_mask set bitwise cpu_no 3 2 1 0 in hex>'
    )
    parser.add_argument(
        "-d", action='store', 
        help='the destination IP name or IP address a.b.c.d'
    )
    parser.add_argument(
        "-i", action='store', 
        default=0, type=int,
        help='time interval between periodic reports in sec'
    )
    parser.add_argument(
        "-l", action='store', 
        default=100000, type=int,
        help='Num of msg to send'
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

# DTNlon - JBO
#tcp_buf_size = ["10000000", "20000000", "30000000", "40000000", "0" ]
#file_postfix = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11",  "12",  "13", "14", "15", "16", "17", "18"
#file_postfix = ["08", "09", "10", "11",  "12",  "13", "14", "15", "16", "17", "18"]

#DTNlon-RAL
tcp_buf_size = ["2500000", "5000000", "7000000", "10000000", "20000000", "30000000", "40000000", "0" ]
file_postfix = ["05", "06", "07", "08", "09", "10", "11",  "12",  "13", "14", "15", "16", "17", "18"]

#AARnetCBR
#tcp_buf_size = ["100000000", "600000000", "900000000", "1023000000", "0" ]
#file_postfix = ["08", "09", "10", "11",  "12",  "13", "14", "15", "16", "17", "18"]

prog = "tcpmon_mon"
#param_test = " -u 5201 -w0 -C -H -G 2000 "
#DTNlon-RAL ceph-gw8
#param_test = " -u 2811 -w0 -C -H -G 2000 "
#DTNlon-RAL ceph-svc03
param_test = " -u 1094 -w0 -C -H -G 2000 "

def main() -> None:
    parser = init_argparse()
    args = parser.parse_args()
    
    # sort out aguments
    affinity = ' -a '+args.a
    if(args.a == ' '):
        affinity =' '
    interval = ' -i '+ str(args.i)
    if(args.i == 0):
        interval =' '
    loops = " -l "+str(args.l)
    run_time = ' '
    if(args.t > 0):
        run_time = " -t "+str(args.t)
        loops = ' '
    
    # get the home directoy
    home_directory = os.path.expanduser( '~' )
    # get the current directoy
    log_file_dir = os.getcwd()
    
    # workout where the program is
    path = os.path.join( home_directory, 'bin', prog)
    file_exists = os.path.exists(path)
    cmd_test =  os.path.join(log_file_dir, prog) + param_test
    if(file_exists):
        cmd_test = path + param_test
    
    # get the date for the filename  
    curent_date = datetime.datetime.now()
    day = curent_date.strftime("%d")
    month = curent_date.strftime("%b")
    year = curent_date.strftime("%y")
    
    # loop over the TCP buffer sizes
    index = 0
    for buf_size in tcp_buf_size:
        #print('{}, {}, {}'.format(index, buf_size, file_postfix[index] ))
        # make the log file
        log_file = log_file_dir+"/"+args.o+"_tcpmon_mon_"+day+month+year+"_"+file_postfix[index]+".txt"
        #print (log_file)
        # open file for writing
        outfile = open(log_file,'wt')
        # add 1st line
        outfile.write('TCP buffer size; ; ; {}\n'. format(buf_size))
        # make the command
        cmd = cmd_test+" -d "+args.d+" -p "+str(args.p)+loops+run_time+affinity+interval+" -S "+buf_size
        print(cmd)
        output, error = run_command(cmd) 
        outfile.write('{}\n'. format(output))
        
        # close file
        outfile.close()
        index +=1


if __name__ == "__main__":
    main()


