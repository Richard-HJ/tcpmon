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

#Achievable TCP throughput as a Function of the TCP Buffer Size 

##On the Sending Host
###Check the cmd_tcpmon_bufscan.py Script
Edit the python script to check that the range of TCP buffers used matches the Delay Bandwidth Product for the test being made. For example for 100 Gigabit link, with an RTT of 10 ms, and an expected throughput of 30 Gbit/s the DBP is 37.5 MBytes and following range would be suitable:
```
tcp_buf_sizes = ["0", "1M", "2.0M", "5M", "7M", "10M", "12M", "15M", "17M", "20M", "22M", "25M", "27M", "30M", "35M", "40M", "45M", "50M" ]
```
Up to 18 TCP buffer sizes are supported.

###To run:
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

##Files Produced
The text file names contain a date followed by a number e.g.
```
RNP-DTNpar_100G_EllaLink_TCPbuf_09Aug21_02.txt
```
##Using Excel to Plot the Results
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

