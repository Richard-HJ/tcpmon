#!/bin/python3


import sys
import argparse
import os
import pathlib
import datetime
import subprocess



# OC VMs
# 10G paris 7 ms 10 Gbit/s DBP 7MB
# OCpar-OCfra 16.6 ms 10Gbit/s DBP 20.75 MBytes
#tcp_buf_sizes = ["0", "1M", "2.0M", "3.0M", "3.5M", "4.0M", "4.5M", "5.0M", "5.5M", "6.0M", "6.5M", "7.0M", "7.5M", "8M", "8.5M", "9M", "10M", "20M" ]

#CoralES rtt 50ms 10Gbit/s
#tcp_buf_sizes = ["0", "1M", "2.0M", "5M", "10M", "15M", "20M", "25M", "30M", "35M", "40M", "45M", "50M", "55M", "60M" ]

# 100G Cam 4 ms 30 Gbit/s DBP 15MB 
# 100G RAL 3 ms 30 Gbit/s DBP 11MB
# 100G paris 7 ms 30 Gbit/s DBP  26 MB
tcp_buf_sizes = ["0", "1M", "2.0M", "5M", "7M", "10M", "12M", "15M", "17M", "20M", "22M", "25M", "27M", "30M", "35M", "40M", "45M", "50M" ]

# NII rtt 170ms 10Gbit/s DPB 212 MB  30Gbit/s DPB 638 MB
#tcp_buf_sizes = ["0", "10M", "30M", "50M", "70M", "100M", "150M", "170M", "200M", "220M", "250M", "300M", "350M", "400M", "420M", "450M", "500M", "600M" ]

# Lon-AARNet 100G
#tcp_buf_sizes = [ "0", "10M", "50M", "100M", "200M", "300M", "400M", "500M", "600M", "700M", "750M", "800M", "850M", "900M", "950M", "980M", "1023M" ]

file_postfix = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11",  "12",  "13", "14", "15", "16", "17", "18" ]
#file_postfix = ["08", "09", "10", "11", "12", "13", "14", "15", "16", "17",  "18"]


msg_spacing = "0"


prog = "tcpmon_mon"
#DTNlon-DTNpar
param_test = " -u 5201 -C "
#DTNlon-RAL
#param_test = " -u 2811 -w0 -C -H -G 2000 "
#DTNlon-CoralES
param_test = " -C "

def init_argparse() -> argparse.ArgumentParser:
# ------------------------------------------------------------------------------
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
        "-u", action='store',
        default=5201, type=int,
        help='tcp port number'
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

def scale_units(txt):
# ------------------------------------------------------------------------------
# input is of the form nnnx or nn.nx where x is the scale factor one of kKmMgG
    KILO=1000
    MEGA=1000000
    GIGA=1000000000
    
    # find the scale factor letter
    scale = " "
    for a in txt:
        if a.isalpha():
            scale = a

    num_txt = txt.strip('kKmMgG')
    num = float(num_txt)

    if scale == "k":
        num = num * KILO
    elif scale == "K":
        num = num * KILO
    elif scale == "m":
        num = num * MEGA
    elif scale == "M":
        num = num * MEGA
    elif scale == "g":
        num = num * GIGA
    elif scale == "G":
        num = num * GIGA
    return int(num)
    

def main() -> None:
# ------------------------------------------------------------------------------
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
    ipv6 = ' '
    if(args.ipv6):
        ipv6 = " -6 "
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
    
    # loop over the TCP buffer size    
    index = 0
    #for msg_spacing in msg_spacings:
    for tcp_buf_size in tcp_buf_sizes:
        # make the log file
        log_file = log_file_dir+"/"+args.o+"_tcpmon_bufscan_"+day+month+year+"_"+file_postfix[index]+".txt"
        #print (log_file)
        # open file for writing
        outfile = open(log_file,'wt')
        # sort TCP buffer scaling
        tcp_buffer = scale_units(tcp_buf_size)
        
        # make the command
        cmd = cmd_test+" -d "+args.d+" -u "+str(args.u)+ipv6+" -p "+str(args.p) +loops+run_time+affinity+interval+ " -S " +str(tcp_buffer) + " -w "+msg_spacing
        print(cmd)
        output, error = run_command(cmd) 
        outfile.write('{}\n'. format(output))
        
        # close file
        outfile.close()
        index +=1


if __name__ == "__main__":
    main()


