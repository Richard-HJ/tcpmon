TCP Measurement Scripts for tcpmon
==================================
This note describes how to run tcpmon and the associated scripts to make the following measurements:
•	Achievable TCP throughput as a Function of the TCP Buffer Size
•	The Performance of a Single TCP Flow
•	Multiple Concurrent TCP Flows using different core pairs
As described, the measurement tool runs as a tcpmon_mon – tcpmon_resp client-server pair with each process using affinity to run on a specific CPU core. However by not specifying the affinity parameter the tcpmon processes will run on any available core as determined by the linux scheduler.

The examples use IPv4 but the tool supports IPv6 when the -6 parameter is given.

Together with the performance measurements it is useful to record the path of the flow using
```
traceroute <remote host>
```
and the kernel parameters and the OS configuration with
```
./cmd_get_sysinfo.pl
```

# Achievable TCP throughput as a Function of the TCP Buffer Size 

## On the Sending Host
### Check the cmd_tcpmon_bufscan.py Script
Edit the python script to check that the range of TCP buffers used matches the Delay Bandwidth Product for the test being made. For example for 100 Gigabit link, with an RTT of 10 ms, and an expected throughput of 30 Gbit/s the DBP is 37.5 MBytes and following range would be suitable:
```
tcp_buf_sizes = ["0", "1M", "2.0M", "5M", "7M", "10M", "12M", "15M", "17M", "20M", "22M", "25M", "27M", "30M", "35M", "40M", "45M", "50M" ]
```
Up to 18 TCP buffer sizes are supported.

### To run:
```
./cmd_tcpmon_bufscan.py -d 62.40.123.242 -u 5201 -p 98380 -t 60 -i2  -a 0x40 -o DTNlon-IDIA
```
Where 
-d is the name or IP address of the remote host
-u is the TCP port number to use
-p is the length in bytes of the message to send
-t  is the time to run the test in sec
-i is time interval between periodic reports in sec
-a denotes the cpu_mask with selected cores set bitwise in hex, in this case 0x40 is core 6
-o is the name to prepend to the files with the measurements 

## Files Produced
The text file names contain a date followed by a number e.g.
```
RNP-DTNpar_100G_EllaLink_TCPbuf_09Aug21_02.txt
```
## Using Excel to Plot the Results
The excel file is tcpmon_bufscan_Mar24.xlsm

You will have to press the “Enable Content” button to allow the macros to work.
To load the text files select View / Macros / View Macros / Load_all
Then use the open window to select one of the files you wish to load, the macro will match the _nn number of the file to that of the tab.
Do not worry if there is a message from Excel suggesting a problem with one or more formula references in the worksheet.

In the tab “Plots” press the “Read Sheet Names” button to correctly fill in column A. 

The numbers in rows 2-8 columns A-M give the line and column numbers of the data items in the data tabs. The most important are:
msg spacing line
tcpmon  BW line
Flow start line
Flow last line
rx_out_of_buffer column  (given as the letters of the column)

# The Performance of a Single TCP Flow
## On the Receiving Host
Run tcpmon_resp:
```
./tcpmon_resp -A 6 -u 5201 -I enp131s0f1
```

Where 
-A denotes the core you wish to use, in this case core 6
-u is the TCP port number to use
-I is the name of the ethernet interface on the receiving host

## On the Sending Host
Run tcpmon_mon:
```
./tcpmon_mon -d 62.40.123.242 -u 5201 -p 98380 -i 2 -t 300 -w 0 -A 6 -C -S 0 > IDIA-DTNlon_tcpmon_tseries_04Jun24_11.txt 2>&1
```

Where 
-d is the name or IP address of the remote host
-u is the TCP port number to use
-p is the length in bytes of the message to send
-i is time interval between periodic reports in sec
-t  is the time to run the test in sec
-w is the time between sending messages in usec
-A denotes the core you wish to use, in this case core 6
-C indicated that the CPU core information is given as a table
-S is the spacing in micro seconds between sending the messages
## File Produced
The text file name has `_11` appended to enable it to be easily plotted using the same excel file as for multiple concurrent files. Please see the next section.

# Multiple Concurrent TCP Flows using different core pairs
The python script will run a set of parallel TCP flow tests starting with 1flow then 2 flows up to n flows. There is a tcpmon_mon – tcpmon_resp pair for each TCP flow.
`--oneTest` as a parameter to cmd_run_tcpmon_multiflow.py allows to run just one set of n TCP flows not the sequence of 1 flow then 2 up to n flows.
## On the Receiving Host
### Check the cmd_run_tcpmon_resp.py Script
Edit the python script cmd_run_tcpmon_resp.py to check:
•	The first TCP port to use, default `5201`
•	The name of the local NIC e.g. `nic = "enp3s0"` 
•	The number of the first core that will be used e.g. `cpu_core = 0`
•	The sequence of core affinity to use to run tcpmon_resp; 
even core number affinity, or consecutive core affinity.
•	Also check the number of cores required are physically available and adjust the calculation of affinity as required.
### To run:
```
./cmd_run_tcpmon_resp.py -n 5
```

Where 
-n is the number of concurrent TCP flows that will be used
## On the Sending Host
### Check the cmd_tcpmon_multiflow.py Script
Edit the python script cmd_tcpmon_multiflow.py to check:
•	The first TCP port to use, default `tcp_port = 5201` 
•	The number of the first core that will be used e.g. `cpu_core = 0` 
•	The sequence of core affinity to use to run tcpmon_resp; 
even core number affinity, or consecutive core affinity.
•	Also check the number of cores required are physically available and adjust the calculation of affinity as required.
### To run:
```
./cmd_run_tcpmon_multiflow.py -d 192.84.5.1 -t 300 -w 0 -n 5 -o AARnetCBR-CamDTN1
```
Where 
-d is the name or IP address of the remote host
-t  is the time to run the test in sec
-w is the time between sending messages in usec
-n is the maximum number of concurrent TCP flows that will be used
-o is the name to prepend to the files with the measurements 
## Files Produced
The text file names contain a date followed by a number, the first digit being the number of concurrent flows and the second the flow number in that test e.g.
`AARnetCBR-RALxrootd01_tcpmon_multiflow_28Feb24_11.txt
AARnetCBR-RALxrootd01_tcpmon_multiflow_28Feb24_21.txt
AARnetCBR-RALxrootd01_tcpmon_multiflow_28Feb24_22.txt`
## Using Excel to Plot the Results
The excel file is tcpmon_multiflow_tseries_Feb24.xlsm

You will have to press the “Enable Content” button to allow the macros to work.
To load the text files select View / Macros / View Macros / Load_all
Then use the open window to select one of the files you wish to load, the macro will match the _nn number of the file to that of the tab.
Do not worry if there is a message from Excel suggesting a problem with one or more formula references in the worksheet.

In the tab “Plots” press the “Read Sheet Names” button to correctly fill in column A. 

The numbers in rows 2-8 columns A-H give the line and column numbers of the data items in the data tabs. The most important are:
Flow start line
Flow last line
Summary start line
Summary last line
and
Data_rate column
re-trans column
elapsed_time

Columns N-P give the names of the axes in the plots.
