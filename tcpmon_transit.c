/*
     tcpmon_transit.c     R. Hughes-Jones  The University of Manchester

     Aim is to send a stream of TCP messages to measure the network transit times
     Use TCP socket to:
	   send a series (-l) of n byte (-p) "messages" to remote node with a specified interpacket interval (-w)
     Print local stats

*/

/*
   Copyright (c) 2015,2016,2017,2018,2019,2020,2021,2022,2023,2024 Richard Hughes-Jones, University of Manchester
   All rights reserved.

   Redistribution and use in source and binary forms, with or
   without modification, are permitted provided that the following
   conditions are met:

     o Redistributions of source code must retain the above
       copyright notice, this list of conditions and the following
       disclaimer. 
     o Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials
       provided with the distribution. 

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
   TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/


#define INSTANTIATE true

#include "net_test.h"                            /* common inlcude file */
#include "version.h"                             /* common inlcude file */


#define TCP_DATA_MAX 16000000
    unsigned char tcp_data[TCP_DATA_MAX];        /* TCP frame to send */
    char tcp_data_recv[TCP_DATA_MAX];   /* TCP frame received */
    char tcp_hist_recv[TCP_DATA_MAX];   /* TCP frame received for remote histograms*/
    char tcp_snmp_recv[TCP_DATA_MAX];   /* TCP frame received for remote snmp data */
    char tcp_cpuload_recv[TCP_DATA_MAX];   /* TCP frame received for remote cpu data */
    int *int_data_ptr;                           /* int pointer to data */
      
#define ERROR_MSG_SIZE 256
    char error_msg[ERROR_MSG_SIZE];                         /* buffer for error messages */

/* for command line options */
    extern char *optarg;

/* parameters */
    int dest_tcp_port = 0x3799;                 /* 14233 base10 */
    char dest_ip_address[HOSTNAME_MAXLEN];
    int pkt_len = 10000;                		/* length of request packet */
    int wait_time_max = 0;       	            /* max length of wait time used for looping over inter packet wait */
    int interval_stats_sec =0;      	        /* time interval between reading TCP stats */ 
    int get_hist =0;            	            /* set to 1 for histograms */
    int get_info =0;            	            /* set to 1 for information = relative packet arrival times */
    int bin_width =1;                           /* bin width of interframe time histo */
    int low_lim =0;                             /* low limit of interframe time histo */
    int wait_time_int=0;                        /* wait time between sending packets */
    int gap_time_int=0;                         /* time to wait time between sending busts of packets spaced by wait_time_int */
    int burst_mode = 0;                         /* =1 if send loop_count packets then wait gap_time_int 1/10 us */
    int soc_buf_size =0;                        /* send & recv buffer size bytes 0= do not set */
    int precidence_bits=0;                      /* precidence bits for the TOS field of the IP header IPTOS_TOS_MASK = 0x1E */
    int tos_bits=0;                             /* tos bits for the TOS field of the IP header IPTOS_PREC_MASK = 0xE0 */
    int tos_set = 0;                            /* flag =1 if precidence or tos bits set */
    int dscp_bits=0;                            /* difserv code point bits for the TOS field of the IP header */
    int dscp_set = 0;                           /* flag =1 if dscp bits set - alternative to precidence or tos bits set */
    int quiet = 0;                              /* set =1 for just printout of results - monitor mode */
    int loop_count = 2;              	        /* no. times to loop over the message loop */
    int burst_count = 1;              	        /* no. bursts of packet to send in Burst mode */
    int response_len = 64;           	        /* length of response message */
    int verbose =0;                  		    /* set to 1 for printout (-v) */
    int use_IPv6 =0;                            /* set to 1 to use the IPv6 address family */
    int send_ack = 0;           	            /* set to n to tell remote end to send ACK aster n packets */
    int run_time_sec =0;                        /* no of sec to run test */
    int extended_output =1;                     /* set to 1 for more printout (CPUStats */
    int info_lines = 0;                         /* number of InfoStore lines -G option */
    int n_to_skip=0;                            /* number of packets to skip before recording data for -G option */
    int log_lost=0;                             /* =1 to log LOST packets only -L option */
    long cpu_affinity_mask;                     /* cpu affinity mask set bitwise cpu_no 3 2 1 0 in hex */
    float run_rate =0.0;                        /* user data rate in Mbit/s */
    int rate_set = 0;                           /* flag =1 if a data rate is given - calc  wait_time_int */
	int print_cpu_table=0;                      /* =1 to print the CPU useage as a table */
	int force_set_mss=0;                        /* >0 value to use to set MSS */
	int num_trials=1;                           /* number of trials - num times to run the sending loop */

    struct param *params;

/* control */
    int loop_max = 0;                           /* loop control - number of times to loop over the sending loop - allows run to time */
    int64 frame_num;                            /*  frame number valid prior to sendto() */

/* timing */
    struct timeval before;           	        /* time before measurements */
    struct timeval after;            	        /* time after measurements */
    struct timeval start;
	StopWatch stopwatch_elapsed;                // data transfer time & elapsed time through the transfer
    double delta_t_elapsed = -1;
    double delta_t_elapsed_last = 0;

/* statistics */
    struct HIST hist[10];
    int num_sent=0;                             /* total no. of packets sent */
    int num_bursts = 0;                         /* number of packet bursts sent */

	int64 bytes_sent_last = 0;

    NET_SNMPStat net_snmp_stats;
    NETIFinfo net_if_info[NET_SNMP_MAX_IF];
    NETIFinfo net_if_info_recv[NET_SNMP_MAX_IF];
    SNMPinfo snmp_info;
    SNMPinfo snmp_info_recv;

    NICinfo nic_info[2];
    int64 rx_out_of_buffer;
	int tcpi_total_retrans_last = 0;
	int infostore_total_retrans_last = 0;
 
/* parameters for socket etc */
    int loops_done=0;
    int tcp_soc =0;                         	/* handle for socket */

#define LOCK_SLEEP_TIME    30

/* forward declarations */
static void parse_command_line (int argc, char **argv);
static void sig_alrm(int signo);
static void cntlc_handler(int signo);
static void cntlz_handler(int signo);
static void close_link_error( int tcp_soc );
int tcp_send_cmd(int tcp_soc, char *send_data, int req_len, char *data_recv, int resp_len, char *name); 

int tcp_read(int soc, char *ptr, int len_to_read, int *recv_loop)
  /* --------------------------------------------------------------------- */
{
	/* read len_to_read bytes from the TCP soc */
	/* TCP is a stream so must check we read the correct no. of bytes AND allow for caught signals */

	int nbytes_left;
	int flags =0;
	int msg_len_read;
	int nread;
	int errno_save;
	int n_loop;
	
	nbytes_left = len_to_read;
	msg_len_read = 0;
	n_loop = *recv_loop;
	while(nbytes_left>0){
		nread = recvfrom(soc, ptr, nbytes_left, flags,  NULL, NULL );
		/* check for error or link closed ( nread =0) */
		if(nread <=0){
			if(errno == EINTR ) {
				printf("EINTR seen nread %d\n", nread);
				nread = 0;	/* signal caught */
			}
			else {
				errno_save =errno;
				if(verbose) {
					perror("tcp_read Error on recvfrom:" );
					printf(" read %d bytes nbytes_left %d errno %d\n", nread, nbytes_left, errno_save);
				}
				if(nread ==0 && errno_save ==0) return(0); // link closed
				return(-1);
			}
		}
		nbytes_left -= nread;
		ptr+= nread;
		msg_len_read += nread;
		n_loop++;
	}	/* end of tcp-read while() */
	*recv_loop = n_loop;	
	return (msg_len_read);
}

int tcp_write(int tcp_soc, char *ptr, int len_to_write, int *send_loop)
  /* --------------------------------------------------------------------- */
{
	/* write len_to_write bytes from the TCP soc */
	/* TCP is a stream so must check we read the correct no. of bytes AND allow for caught signals */

	int nbytes_left;
	int flags =0;
	int nsent;
	int msg_len_write;
	int errno_save;
	int n_loop;

		/* need to loop until sent all req_len bytes (in case send interrupted) */
		nbytes_left = len_to_write;
		msg_len_write = 0;
		n_loop =0;
		do {
			nsent = sendto(tcp_soc, ptr, nbytes_left, flags, NULL, 0);
			/* check for error or link closed ( nsent <= 0) */
			if (nsent <=0) {
				if(errno == EINTR ) {
					printf("EINTR seen nsent %d\n", nsent);
					nsent = 0;	/* signal caught before any data was transmitted - try again */
				}
				else {
					errno_save =errno;
					perror("tcp_write Error on sendto:");
					printf(" wrote %d bytes nbytes_left %d errno %d\n", nsent, nbytes_left, errno_save);
					return (-1);
				}
			}
			nbytes_left -= nsent;
			ptr += nsent;
			msg_len_write += nsent;
			n_loop++;
		} while ( nbytes_left > 0 );


	*send_loop = n_loop;	
	return (msg_len_write);
}



int tcp_send_cmd(int soc, char *send_data, int req_len, char *data_recv, int resp_len, char *name)
/* --------------------------------------------------------------------- */
{
	int nwrite;
	int nread;
	int send_loop = 0;
	int recv_loop = 0;

	if(verbose) printf("tcp_send_cmd: %s req_len %d resp_len %d \n", name, req_len, resp_len);     

/* send the request */	
	nwrite = tcp_write(soc, send_data, req_len, &send_loop);
	if(nwrite != req_len) {
		sprintf(error_msg, 
			"Error: on sending %s: sent %d bytes not %d ", 
			name, nwrite, req_len );
		perror(error_msg );
		return (-1);
	}		

/* receive the response  */
	nread = tcp_read(soc, data_recv, resp_len, &recv_loop);
	if(nread != resp_len) {
		sprintf(error_msg, 
			"Error: on receiving %s: read %d bytes not %d ", 
			name, nread, resp_len );
		perror(error_msg );
		return (-1);
	}
	return (0);
}

int main (int argc, char **argv)
/* --------------------------------------------------------------------- */
{
/* 
Ethernet frame for IP:
           | MAC link | IP | UDP/TCP | data             | CRC |
len bytes:         14   20    8 / 20   46->(1500-28/40)    4      
   ethernet packet for IP:
                      |                                 |
Ethernet MTU 1500 bytes
Ethernet type 0-1500     packet length
              5101-65535 packet type
              2048       = IP
*/
    struct sockaddr *soc_address;
    SOC_INFO soc_info;
    unsigned int flags = 0;          	        /* flags for sendto() recvfrom() select() */
    int ipfamily_addr_len;                      /* length of soc_recv_address - depends on IP family IPv4, IPv6*/
    struct addrinfo hint_options, *result;      /* for getaddrinfo() */
    struct addrinfo *rp;                        /* for debug listing */

/* statistics */
    CPUStat cpu_stats;
    CPUinfo cpuinfo[NET_SNMP_MAX_CPU+1];
    CPUinfo *cpuload_ptr = NULL;
    Interrupt_info  inter_info[NET_SNMP_MAX_IF];  

    NETIFinfo net_if_info[NET_SNMP_MAX_IF];
    SNMPinfo snmp_info;

    struct InfoStore infostore;
	int info_items = 12;

    int recv_bufSize, send_bufSize;
    struct tcp_info tcpInfo;
	int delta_total_retrans = 0;
	//int64 TCPInSegs;                            /* from local snmp */
	int64 TCPOutSegs;                           /* from local snmp */
	
/* timing */
    long delay;                      		    /* time for one message loop */
    double time_per_frame;
    double wait_time=0;                         /* time to wait between packets */
    double gap_time=0;                          /* time to wait between sending bursts of packets */
	double trial_time=0;

/* timers */
    StopWatch wait_sw;                          /* time to wait between sending packets */
    StopWatch gap_sw;                           /* time to wait between sending bursts */

    StopWatch stopwatch_putTime;
    double delta_t;

    struct itimerval timer_value;               /* for the interval timer */
    struct itimerval timer_old_value;  

/* local variables */
    char* ptr = NULL;
    int nbytes_left = 0;	                    /* no of bytes left to read from TCP stream */
    int nsent = 0;		                        /* no of bytes sent to TCP socket */
	int send_loop = 0;                          /* num times called sendto() to send the block of data */
    int error;
    int errno_save;
    double data_rate;   
	double tcp_retrans_rate;
    int i;
    int ret;
    char port_str[128];
    char* remote_addr_text;
    int pkt_len_requested=0;
	int mss;                                    /* TCP max segment size from getsockopt() */
    int len;
	int req_len;
	int resp_len;
	int trial;

/* Set the default IP Ethernet */
/* IP protocol number ICMP=1 IGMP=2 TCP=6 UDP=17 */

/* set the signal handler for SIGALRM */
    signal (SIGALRM, sig_alrm);
/* define signal handler for cntl_c */
    signal(SIGINT, cntlc_handler);
/* define signal handler for cntl_z */
    signal(SIGTSTP, cntlz_handler);

    h_book(&hist[0], 0, 0, 10, 100, "network put time us");

/* get the input parameters */
    parse_command_line ( argc, argv);

    /* check message length >= sizeof(struct param) */
    if( pkt_len < sizeof(struct param)) { 
        pkt_len_requested = pkt_len;
		pkt_len = sizeof(struct param);
    }
	
/* set up InfoStore */
	if(get_info){
		InfoStore_Init(&infostore, "network_time_us; write_blksize_bytes; send_loop; soc_fd; elapsed_time_us; recv_bufSize; send_bufSize; tcpi_snd_ssthresh; tcpi_snd_cwnd_octets; tcpi_rtt; tcpi_rttvar; tcpi_retrans ", 
						info_items, info_lines );
	}
	
/* set histogram parameters */
    hist[0].bin_width = bin_width;
    hist[0].low_lim = low_lim;

/* set the CPU affinity of this process*/
    set_cpu_affinity (cpu_affinity_mask, quiet);

/* initalise and calibrate the time measurement system */
    ret = RealTime_Initialise(quiet);
    if (ret) exit(EXIT_FAILURE);
    ret = StopWatch_Initialise(quiet);
    if (ret) exit(EXIT_FAILURE);

/* initalise CPUStats */
    CPUStat_Init();

/* test system timer */	
    gettimeofday(&before, NULL);
    sleep(1);	
    gettimeofday(&after, NULL);
    delay = ((after.tv_sec - before.tv_sec) * 1000000) + (after.tv_usec - before.tv_usec);
    if(!quiet) printf("clock ticks for 1 sec = %ld us\n", delay);

/* create the socket address with the correct IP family for sending TCP packets */
    sprintf(port_str, "%d", dest_tcp_port);
    /* clear then load the hints */
    bzero(&hint_options, sizeof(struct addrinfo) );
    hint_options.ai_family = AF_INET;
#ifdef	IPV6
   if(use_IPv6 == 1) hint_options.ai_family = AF_INET6;
#endif
    hint_options.ai_socktype = SOCK_STREAM;
    hint_options.ai_flags = 0;
    hint_options.ai_protocol = 0 ;          /* Any protocol */
    /* no flags as not a server but will use connect()*/
    if(verbose) printf("HINTS  family %d \n socktype %d \n protocol %d \n", 
		       hint_options.ai_family, hint_options.ai_socktype, hint_options.ai_protocol);

    error = getaddrinfo(dest_ip_address, port_str, &hint_options, &result);   
    if(error){
        snprintf(error_msg, ERROR_MSG_SIZE,
		 "Error: Could not use address family %s", gai_strerror(error) );
	perror(error_msg );
        exit(EXIT_FAILURE);
    }
    if(verbose){
		printf(" Socket address IP family info found:\n");
		for (rp = result; rp != NULL; rp = rp->ai_next) {
			printf(" family [2=IPv4 10=ipv6] %d \n socktype [1= STREAM 2=DGRAM] %d \n protocol [0=any 6=TCP 17=UDP] %d ipfamily_addr_len %d\n", 
					rp->ai_family, rp->ai_socktype, rp->ai_protocol, rp->ai_addrlen);
		}
    } 
		 
/* loop over returned socket address info - make the socket and try to connect */
    do{ 
		if(verbose){
			printf(" family [2=IPv4 10=ipv6] %d \n socktype [1= STREAM 2=DGRAM] %d \n protocol [0=any 6=TCP 17=UDP] %d ipfamily_addr_len %d\n", 
					result->ai_family, result->ai_socktype, result->ai_protocol, result->ai_addrlen);
		}
		/* get the length of the sock address struct */
		ipfamily_addr_len = result->ai_addrlen;
		soc_address = result->ai_addr;
		/* convert remote IP address to text */
		remote_addr_text = sock_ntop (soc_address );

		/* Open the TCP IP socket. */
		soc_info.soc_buf_size = soc_buf_size;
		soc_info.precidence_bits = precidence_bits;
		soc_info.tos_bits = tos_bits;
		soc_info.tos_set = tos_set;
		soc_info.dscp_bits = dscp_bits;
		soc_info.dscp_set = dscp_set;
		soc_info.enable_nodelay = 1; /* turn off the Nagle algorithm */
		soc_info.quiet = quiet;
		sock_create_tcp_socket(&tcp_soc, &soc_info, result->ai_family);

       /* we dont need to bind() as we dont care what the local port number is */
	   
	   /* set the TCP MSS for the connected socket  RHJ hack if MTU discovery not enabled */
	   /* Set MSS to 1460 Bytes 1500 -20IP -20TCP. You get 1448 as the TCP stack allows for an extra 12-bytes for the TCP Timestamp option to protect against wrapped TCP sequence numbers */
		if(force_set_mss > 0){
			len = sizeof( force_set_mss );
			error = setsockopt( tcp_soc, IPPROTO_TCP, TCP_MAXSEG, &force_set_mss,  len );
			if (error) {
				perror("setsockopt( TCP_MAXSEG) on TCP socket failed :" );
				exit(EXIT_FAILURE);
			}
		}

       /* make a link to the remote TCP port */
		error = connect(tcp_soc, result->ai_addr, ipfamily_addr_len );
		if (error == 0) break; /* connected */

		/* close unused socket */
		close (tcp_soc);
    } while(  (result = result->ai_next) != NULL); /* end of loop over socket address info */

/* check that TCP connected OK */
   if (result == NULL) {
		errno_save = errno;
		sprintf(error_msg, 
		"Error: Connect to remote TCP IP host %s port %d failed:", 
		dest_ip_address, dest_tcp_port);
        errno = errno_save;
		perror(error_msg );
		printf("error %d\n", error);
		exit(EXIT_FAILURE);
    }

	/* find the TCP MSS for the connected socket */
	len = sizeof( mss );
	ret = getsockopt( tcp_soc, IPPROTO_TCP, TCP_MAXSEG, (char*) &mss, (socklen_t *) &len );
	
	if(!quiet){
		printf(" The destination IP name: %s IP address: %s\n", dest_ip_address, remote_addr_text);
		printf(" The destination TCP port is   %d %x\n", dest_tcp_port, dest_tcp_port);
		printf(" The TCP maximum segment size is   %d\n", mss);
	}

/* Here we build the TCP message, ready for sending.
   Fill with random values to prevent any compression on the network 
*/
    for(int_data_ptr= (int *)&tcp_data; 
        int_data_ptr < (int *)&tcp_data + TCP_DATA_MAX/4; int_data_ptr++){
        *int_data_ptr =rand();
    }
    for(i=0; i<20; i++){
        tcp_data[i] = i;
    }

    gettimeofday(&start, NULL);

//loop over the number of trials   
 for(trial =0; trial < num_trials; trial++){  
    if(trial > 0) quiet=1;

/* point params struct at the tcp data buffer */
	params = (struct param *)&tcp_data;

    params->mss = mss;
	params->frame_num = 0;
	params->low_lim   = i4swap(low_lim);
	params->bin_width = i4swap(bin_width);

    gap_time = (double) gap_time_int / (double) 10.0;
    /* calc wait_time_int in 1/10 us from the given run_rate in Mbit/s */
    if(rate_set ==1){
        wait_time_int = (int)(10.0*(float)pkt_len*8.0/run_rate);
    }  
	wait_time = (double) wait_time_int/ (double) 10.0;
      
/* clear the local stats */
	delay = 0;
	frame_num = 0;
	loops_done = 0;

	loop_max = loop_count;
/* set the alarm to determine when to read the TCP stats.
   If a time is set for the length of the test the sig_alrm() handler sets loop_max to 0 to stop */
	if(verbose)printf("run_time_sec %d interval_stats_sec %d\n", run_time_sec, interval_stats_sec);
	if(interval_stats_sec >0) {
		alarm(interval_stats_sec);
		printf("Time;\t Bytes sent;\t Rate Gbit/s;\t Retrans;\t Cwnd MBytes;\t TCPInSegs;\t TCPOutSegs \n"); 
	}
	else if(run_time_sec >0) {
		alarm(run_time_sec);
	}
/* check if need to set the timer to exit the program and print stats and histograms */
    if(interval_stats_sec >0){
		timer_value.it_interval.tv_sec = interval_stats_sec;         /* Value to reset the timer when the it_value time elapses:*/
		timer_value.it_interval.tv_usec = 0;                         /*  (in us) */
		timer_value.it_value.tv_sec = interval_stats_sec;            /* Time to the next timer expiration: 1 seconds */
		timer_value.it_value.tv_usec = 0;                            /*  (in us) */
		/* set the interval timer to be decremented in real time */
		
		ret = setitimer( ITIMER_REAL, &timer_value, &timer_old_value );
		if(ret){
			perror("set interval timer failed :" );
			exit(EXIT_FAILURE);
		}
   }

/* record initial interface & snmp info and put in params */
	net_snmp_Start(  &net_snmp_stats);

/* record initial CPU and interrupt info */
	CPUStat_Start(  &cpu_stats);

/* loop over sending mock data  */
DATA_LOOP_START:
	gettimeofday(&before, NULL);
	/* get Time-zero for stamping the packets */
	StopWatch_Start(&stopwatch_elapsed);

	for (i = 0; i < loop_max; i++) {
		/* set the stopwatch going for waiting between sending packets */
		StopWatch_Start(&wait_sw);

		/* allow tests for a given length of time */
		if(run_time_sec >0) i=0;

		params->cmd = i4swap(CMD_DATA);
		params->frame_num = i8swap(frame_num);
		params->msg_len = i4swap(pkt_len);       
		/* timestamp the packet to send relative to the time started to loop */
		StopWatch_Stop(&stopwatch_elapsed); // time through the transfer
		delta_t_elapsed = StopWatch_TimeDiff(&stopwatch_elapsed);
		params->send_time = i8swap( (int64)(delta_t_elapsed*(double)10.0) );

		if(verbose)printf("----++ frame_num %"LONG_FORMAT"d pkt_len %d loops_done %d\n ", params->frame_num, pkt_len, loops_done);
		if(get_info){
			sock_get_tcp_bufsize(tcp_soc , &recv_bufSize, &send_bufSize);
			sock_get_tcpinfo(tcp_soc , &tcpInfo);
		}
		
		// time the network send
		StopWatch_Start(&stopwatch_putTime);		
		/* send the mock data */
		/* need to loop until sent all req_len bytes (in case send interrupted) */
		nbytes_left = pkt_len;
		ptr = (char*)&tcp_data;
		send_loop =0;
		do {
			nsent = sendto(tcp_soc, ptr, nbytes_left, flags, NULL, 0);
			if(verbose)printf("sendto() Data nsent %d nbytes_left %d pkt_len %d\n", nsent, nbytes_left, pkt_len);
			/* check for error or link closed ( nsent <= 0) */
			if (nsent <=0) {
				if (errno == EINTR ) nsent = 0;	/* signal caught before any data was transmitted - try again */
				else {
					perror("Error on sendto; exiting program");
					close_link_error( tcp_soc );
				}
			}
			nbytes_left -= nsent;
			ptr += nsent;
			send_loop++;
		} while ( nbytes_left > 0 );
		StopWatch_Stop(&stopwatch_putTime);
		delta_t = StopWatch_TimeDiff(&stopwatch_putTime);
		h_fill1(&hist[0], (int64_t)delta_t);
		if(get_info){
				delta_total_retrans = tcpInfo.tcpi_total_retrans - infostore_total_retrans_last;
				infostore_total_retrans_last = tcpInfo.tcpi_total_retrans;
				InfoStore_Store(&infostore, (int64_t)delta_t);
				InfoStore_Store(&infostore, (int64_t)pkt_len);
				InfoStore_Store(&infostore, (int64_t)send_loop);	
				InfoStore_Store(&infostore, (int64_t)tcp_soc);  
				InfoStore_Store(&infostore, (int64_t)delta_t_elapsed);
				InfoStore_Store(&infostore, (int64_t)recv_bufSize);
				InfoStore_Store(&infostore, (int64_t)send_bufSize);
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_snd_ssthresh);
				InfoStore_Store(&infostore, (int64_t)(tcpInfo.tcpi_snd_cwnd * tcpInfo.tcpi_snd_mss));  // snd_cwnd in octets.
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_rtt);
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_rttvar);
				InfoStore_Store(&infostore, (int64_t)delta_total_retrans);
		}
		
		frame_num++;
		loops_done++;
		
		/* wait the required time */
		StopWatch_Delay(&wait_sw, wait_time);
		
		if(verbose)printf("------\n");	
	}    /* end of loop sending frames */

/* record the time */
	gettimeofday(&after, NULL);

	if(burst_mode){
		/* wait the required time */
		StopWatch_Start(&gap_sw);
		StopWatch_Delay(&gap_sw, gap_time);
		num_bursts ++;
		if(num_bursts < burst_count) goto DATA_LOOP_START;
	}

/* record final CPU and interrupt info */
   CPUStat_Stop( &cpu_stats);

/* record final interface & snmp info */
	sleep(1);   // make sure the counters have been updated
	net_snmp_Stop(  &net_snmp_stats);
	net_snmp_Info(  &net_snmp_stats, net_if_info, &snmp_info);
	//TCPInSegs  = snmp_info.TCPInSegs;
	TCPOutSegs = snmp_info.TCPOutSegs;
	
/* record final TCP info */
	sock_get_tcpinfo(tcp_soc , &tcpInfo);

	tcp_retrans_rate = ((double) tcpInfo.tcpi_total_retrans *100.0)/(double)TCPOutSegs;

/* calculate the time per packet  */
   time_per_frame =  (double)delta_t_elapsed  / (double)loops_done;
   data_rate = ( (double)pkt_len * 8.0 * (double)loops_done ) / (double)delta_t_elapsed;
 
/* Get information from remote host */
	/* Write the request to the remote host to return the  stats */
	params = (struct param *)&tcp_data;
	params->cmd = i4swap(CMD_GETSTATS);
	req_len = sizeof(struct param);
	params->msg_len = i4swap(req_len);       
	resp_len = sizeof(struct param);       
	params->resp_len = i4swap(resp_len );  /* used as length of the return message */
	ret = tcp_send_cmd(tcp_soc, (char *)params, req_len, tcp_data_recv, resp_len, "getstats"); 
	/* check for no response / error */
	if(ret < 0) exit(EXIT_FAILURE);
	if(verbose) {
		params = (struct param *)&tcp_data_recv;
	    printf("From tcp_data_recv :\n");
	    printf("num_recv: %d\n", i4swap(params->num_recv) );
	    printf("bytes_recv: %"LONG_FORMAT"d\n", i8swap(params->bytes_recv) );
	    printf("soc_buf_size: %d\n", i4swap(params->soc_buf_size) );
	    printf("first_last_time 0.1us: %"LONG_FORMAT"d\n", i8swap(params->first_last_time) );
	}

	/* send command to get remote network & snmp stats */
	params = (struct param *)&tcp_data;
	params->cmd = i4swap(CMD_GETNETSNMP);
	req_len = sizeof(struct param);
	params->msg_len = i4swap(req_len);       
	resp_len = sizeof(NET_SNMPStat);       
	params->resp_len = i4swap(resp_len );  /* used as length of the return message */
	ret = tcp_send_cmd(tcp_soc, (char *)params, req_len, tcp_snmp_recv, resp_len, "getnetsnmp"); 
	/* check for no response / error */
	if(ret < 0) exit(EXIT_FAILURE);
	/* extract remote interface & snmp info */
	net_snmp_Info(  ( NET_SNMPStat *)tcp_snmp_recv, net_if_info_recv, &snmp_info_recv);

	/* send request for NIC stats */
	params = (struct param *)&tcp_data;
	params->cmd = i4swap(CMD_GETNICSTATS);
	params->index = 0;
	req_len = sizeof(struct param);
	params->msg_len = i4swap(req_len);       
	resp_len = sizeof(NICinfo);       
	params->resp_len = i4swap(resp_len );  /* used as length of the return message */
	ret = tcp_send_cmd(tcp_soc, (char *)params, req_len, (char *)&nic_info[0], resp_len, "getnicstats0"); 
	/* check for no response / error */
	if(ret < 0) exit(EXIT_FAILURE);

	/* send request for NIC stats */
	params = (struct param *)&tcp_data;
	params->cmd = i4swap(CMD_GETNICSTATS);
	params->index = 1;
	req_len = sizeof(struct param);
	params->msg_len = i4swap(req_len);       
	resp_len = sizeof(NICinfo);       
	params->resp_len = i4swap(resp_len );  /* used as length of the return message */
	ret = tcp_send_cmd(tcp_soc, (char *)params, req_len, (char *)&nic_info[1], resp_len, "getnicstats1"); 
	/* check for no response / error */
	if(ret < 0) exit(EXIT_FAILURE);

	//rx_out_of_buffer = nic_stats_getValue( &nic_info, "rx_out_of_buffer");
	/* check for counter not implemented */
	//if( rx_out_of_buffer ==-1) rx_out_of_buffer=0;
		
	/* Write the request to the remote host to return the cpu load */
	params = (struct param *)&tcp_data;
	params->cmd = i4swap(CMD_GETCPULOAD);
	req_len = sizeof(struct param);
	params->msg_len = i4swap(req_len);       
	resp_len = sizeof(cpuinfo);       
	params->resp_len = i4swap(resp_len );  /* used as length of the return message */
	ret = tcp_send_cmd(tcp_soc, (char *)params, req_len, tcp_cpuload_recv, resp_len, "getcpuload"); 
	/* check for no response / error */
	if(ret < 0) exit(EXIT_FAILURE);
	cpuload_ptr = (struct _CPUinfo *)&tcp_cpuload_recv;

  
/* Do Printout */
	/* titles */
	if(!quiet){
		/* print titles  */
		printf(" Trial num; Trial time; 0;");
		
        printf(" %d bytes; ",pkt_len);
		if(pkt_len_requested > 0) printf(" length requested too small at %d bytes using %d bytes; ", pkt_len_requested, pkt_len);
		printf("\n");

		printf(" pkt len; soc_buf_size; num_sent;");
		printf(" wait_time; Time/frame us; Send_time; send_data_rate Mbit;" );

		printf(" TCPOutSegs;");
		printf(" tcpi_total_retrans;");
		printf(" tcp_retrans_rate%%;");
		printf("  ;");

		printf(" Recv soc_buf_size;");
		printf(" num_recv;");
		printf(" bytes_recv;");
		printf(" Recv time/Frame; recv_time; recv_data_rate Mbit;");
		printf("  ;");

		net_snmp_print_info( net_if_info, &snmp_info, 3, 'L');
		net_snmp_print_info( net_if_info, &snmp_info, 3, 'R');
		
		nic_stats_print_info(&nic_info[0], 1, 'R');
		nic_stats_print_info(&nic_info[1], 1, 'R');

		CPUStat_Info(  &cpu_stats, cpuinfo, inter_info);
		if(!print_cpu_table){
			CPUStat_print_cpu_info( cpuinfo, 1, 'L', extended_output);
			printf("  ;");
			CPUStat_print_cpu_info( cpuload_ptr, 1, 'R', extended_output);
		}
		
		printf("\n");
	}

	/* print local stats */
	printf(" %d; ", trial);
	trial_time = (double)(before.tv_sec - start.tv_sec)  + (double)(before.tv_usec - start.tv_usec)/1e6;
	printf("  %g; ", trial_time);
	printf(" 0; ");
	
	printf(" %d; ", pkt_len);
	printf(" %d; ", soc_buf_size);
	printf(" %d; ", loops_done);
	printf("  %g; ", wait_time);
	printf("  %g; ", time_per_frame );
	printf("  %g; ", delta_t_elapsed);      // Send time
	printf("  %g; ", data_rate);
	printf("  %"LONG_FORMAT"d; ", TCPOutSegs); 
	printf(" %d; ", tcpInfo.tcpi_total_retrans);
	printf("  %g; ", tcp_retrans_rate);
	printf("  ;");

	/* print remote stats */
	params = (struct param *)&tcp_data_recv;
	time_per_frame = (double)i8swap(params->first_last_time)/(double)i4swap(params->num_recv);
	data_rate = ( (double)i8swap(params->bytes_recv) * 8.0 ) / (double)i8swap(params->first_last_time);

	printf(" %d; ", i4swap(params->soc_buf_size));
	printf(" %d; ", i4swap(params->num_recv));
	printf(" %"LONG_FORMAT"d; ", i8swap(params->bytes_recv) );
	printf("  %g; ", time_per_frame );
	printf("  %"LONG_FORMAT"d; ", i8swap(params->first_last_time));
	printf("  %g; ", data_rate);
	printf("  ;");
	
	/* print local interface & snmp info */
	net_snmp_print_info( net_if_info, &snmp_info, 4, 'L');    
	/* print remote interface & snmp info */
	net_snmp_print_info( net_if_info_recv, &snmp_info_recv, 4, 'R');

	/* print remote NIC info */
	nic_stats_print_info(&nic_info[0], 2, 'R');
	nic_stats_print_info(&nic_info[1], 2, 'R');

	if(!print_cpu_table){
		/* print total local CPU info */
		CPUStat_print_cpu_info( cpuinfo, 2, 'L', extended_output);

		/* print total remote CPU info */
		CPUStat_print_cpu_info( cpuload_ptr, 2, 'R', extended_output);
	}

	printf("  \n" );
	fflush(stdout);

	if(print_cpu_table){
		/* print total local CPU info */
		CPUStat_print_cpu_info( cpuinfo, 3, 'L', extended_output);
		CPUStat_print_cpu_info( cpuinfo, 4, 'L', extended_output);
		/* print total remote CPU info */
		CPUStat_print_cpu_info( cpuload_ptr, 3, 'R', extended_output);
		CPUStat_print_cpu_info( cpuload_ptr, 4, 'R', extended_output);
	}

	if(get_hist){
		h_output( &hist[0]);
		h_output(&hist[0]);  // do twice to keep excel layout
		//		h_output( &hist[1]);
	} /* end of if(get_hist) */ 

	if(get_info){
		InfoStore_Print(&infostore );
	}


 } // end of loop over trials
 
   close(tcp_soc);
   
   return(0);
}

static void close_link_error( int tcp_soc )
/* --------------------------------------------------------------------- */
{
	close(tcp_soc);
	exit(EXIT_FAILURE);
}

static void parse_command_line (int argc, char **argv)
/* --------------------------------------------------------------------- */
{
/* local variables */
    char c;
    int error;
    int i;
    time_t date;
    char *date_str;
    char cmd_text[128];
    char *str_loc;
    float value;
	int core;

    char *help ={
"Usage: udpmon_bw_mon -option<parameter> [...]\n\
options:\n\
	-6 = Use IPv6\n\
	-A = <cpu core to use - start from 0 >\n\
	-B = <bin width of remote histo in us>\n\
	-C = Print CPU use in a table\n\
	-F = Force MSS to this value\n\
	-G = <[number of packets to skip:]number of packets on which to return information>\n\
	-H = Print histograms\n\
	-L = <[number of packets to skip:]number of LOST packets on which to return information>\n\
	-M = <min (low limit) of remote histo in us>\n\
	-N = <num of trials>\n\
	-P = <precidence bits set - in hex - will be shifted left by 9>\n\
	-Q = <DSCP QoS bits set - in hex >\n\
	-S = <size of send and receive socket buffers in bytes>\n\
	-T = <tos bits set - in hex - will be shifted left by 1>\n\
	-V = print version number\n\
	-a = <cpu_mask set bitwise cpu_no 3 2 1 0 in hex>\n\
	-d = <the destination IP name or IP address a.b.c.d>\n\
	-e = <end value of wait time in us>\n\
	-g = <gap time to wait between bursts in us>\n\
	-h = print this message\n\
	-i = <time interval between reading TCP stats sec [10s]>\n\
	-l = <no. of frames to send>\n\
	-n = <no. of bursts to send in Burst Mode>\n\
	-p = <length in bytes of mock data packet>\n\
	-r = send data rate Mbit/s\n\
	-t = <no. of seconds to run the test - calculates no. of frames to send >\n\
	-q = quiet - only print results\n\
	-v = turn on debug printout\n\
	-u = <tcp port no - default 0x3799 ie 14233 decimal>\n\
	-w = <wait time tt.t in us> "};

    error=0;
    
#ifdef IPv6
    while ((c = getopt(argc, argv, "a:d:e:g:i:l:n:p:r:t:u:w:A:B:F:G:L:M:N:P:Q:S:T:hqv6CHV")) != (char) EOF) {
#else
      while ((c = getopt(argc, argv, "a:d:e:g:i:l:n:p:r:t:u:w:A:B:F:G:L:M:N:P:Q:S:T:hqvCHV")) != (char) EOF) {
#endif	
	switch(c) {

	    case 'a':
		if (optarg != NULL) {
		    sscanf(optarg, "%lx", &cpu_affinity_mask);
		} else {
		    error = 1;
		}
		break;

	    case 'd':
		if (optarg != NULL) {
		    memset(dest_ip_address, 0, HOSTNAME_MAXLEN);
		    strncpy(dest_ip_address,  optarg, HOSTNAME_MAXLEN-1);
		} else {
		    error = 1;
		}
		break;

	    case 'e':
		if (optarg != NULL) {
		    wait_time_max = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'g':
		if (optarg != NULL) {
		    sscanf(optarg, "%f", &value);
		    gap_time_int = (int)(10.0*value);
		    burst_mode =1;
		} else {
		    error = 1;
		}
		break;

	    case 'h':
            fprintf (stdout, "%s \n", help);
	        exit(EXIT_SUCCESS);
		break;

	    case 'i':
		if (optarg != NULL) {
		    interval_stats_sec = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'l':
		if (optarg != NULL) {
		   loop_count = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'n':
		if (optarg != NULL) {
		   burst_count = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'u':
		if (optarg != NULL) {
		    dest_tcp_port =  atoi(optarg); 
		} else {
		    error = 1;
		}
		break;

	    case 'p':
		if (optarg != NULL) {
		    pkt_len = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'q':
	        quiet = 1;
		break;

	    case 'r':
		if (optarg != NULL) {
		    sscanf(optarg, "%f", &run_rate);
		    rate_set =1;
		} else {
		    error = 1;
		}
		break;

	    case 't':
		if (optarg != NULL) {
		    run_time_sec = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'v':
	        verbose = 1;
		break;

	    case 'w':
		if (optarg != NULL) {
		    sscanf(optarg, "%f", &value);
		    wait_time_int = (int)(10.0*value);
		} else {
		    error = 1;
		}
		break;

	    case '6':
	        use_IPv6 = 1;
		break;
		
	    case 'A':
		if (optarg != NULL) {
		   core = atoi(optarg);
		   cpu_affinity_mask = 1<<core;
		} else {
		    error = 1;
		}
		break;

	    case 'B':
		if (optarg != NULL) {
		   bin_width = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

 	    case 'C':
	        print_cpu_table = 1;
		break;

 	    case 'F':
	        force_set_mss = atoi(optarg);
		break;

	    case 'G':
			if (optarg != NULL) {
				memset(cmd_text, 0, strlen(cmd_text));
				strcpy(cmd_text,  optarg);
				str_loc = strstr(cmd_text, ":");
				if (str_loc) {
					*str_loc=' ';
					sscanf(cmd_text, "%d %d", &n_to_skip, &info_lines);
				}
				else {
					n_to_skip =0;
					sscanf(cmd_text, "%d", &info_lines);
				}
			}
	        get_info = 1;
		break;

	    case 'H':
	        get_hist = 1;
		break;

	    case 'M':
		if (optarg != NULL) {
		   low_lim =  atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'N':
		if (optarg != NULL) {
		   num_trials = atoi(optarg);
		} else {
		    error = 1;
		}
		break;

	    case 'P':
		if (optarg != NULL) {
		    sscanf(optarg, "%x", &precidence_bits);
		    tos_set = 1;
		} else {
		    error = 1;
		}
		break;

	    case 'Q':
		if (optarg != NULL) {
		    sscanf(optarg, "%x", &dscp_bits);
		    dscp_set = 1;
		} else {
		    error = 1;
		}
		break;

	    case 'S':
		if (optarg != NULL) {
		    soc_buf_size = (int)atoi(optarg);
		} else {
		    error = 1;
		}

		break;

	    case 'T':
		if (optarg != NULL) {
		    sscanf(optarg, "%x", &tos_bits);
		    tos_set = 1;
		} else {
		    error = 1;
		}
		break;

	    case 'V':
	        printf(" %s \n", TCPMON_VERSION);
	        exit(EXIT_SUCCESS);
		break;

	    default:
		break;
	}   /* end of switch */
    }       /* end of while() */
    if ((pkt_len == 0) || (loop_count == 0)) {
	error = 1;
    }
    if (error ) {
	fprintf (stderr, "%s \n", help);
	exit	(EXIT_FAILURE);
    }

    if(!quiet){
        date = time(NULL);
		date_str = ctime(&date);
        date_str[strlen(date_str)-1]=0;
        printf(" %s :", date_str );
        printf(" %s CPUs", TCPMON_VERSION);
        printf(" Command line: ");
	for(i=0; i<argc; i++){
            printf(" %s", argv[i]);
	}
	printf(" \n");
    }

    return;
}

static void cntlc_handler( int signo)
/* --------------------------------------------------------------------- */
{
/* called on cntl_C */
	fflush(stdout);
        printf("Done  %d loops out of %d. \n", 
	       loops_done, loop_count );
	fflush(stdout);

    return;
}
 
static void cntlz_handler( int signo)
/* --------------------------------------------------------------------- */
{
/* called on cntl_Z */
	fflush(stdout);
        printf("Done  %d loops out of %d. \n", 
	       loops_done, loop_count );
	printf("cntl-Z received : Process ended\n");
	fflush(stdout);
        exit(EXIT_SUCCESS);

    return;
}

static void sig_alrm( int signo)
/* --------------------------------------------------------------------- */
{
	int time_alive_sec;
    double data_rate;   
	struct tcp_info tcp_info;
	int64 bytes_sent;
	int64 del_bytes_sent;
	double del_t_elapsed;
	double tcpi_snd_cwnd;
	int del_tcpi_total_retrans;

/* get the elapsed time now */
	StopWatch_Stop(&stopwatch_elapsed); // time through the transfer
	delta_t_elapsed = StopWatch_TimeDiff(&stopwatch_elapsed);
    time_alive_sec = delta_t_elapsed/1000000 + 0.01 ;  // allow for rounding errors
	//printf("SIGALRM caught time_alive %d run_time_sec %d delta_t_elapsed %g\n", time_alive_sec, run_time_sec, delta_t_elapsed);
/* check if we terminate the program or just print snapshot of statistics */
	if(time_alive_sec < run_time_sec || (run_time_sec ==0)){
		
		bytes_sent = (int64)pkt_len * (int64)loops_done;
		del_bytes_sent = bytes_sent - bytes_sent_last;
		bytes_sent_last = bytes_sent;
		del_t_elapsed = delta_t_elapsed - delta_t_elapsed_last;
		delta_t_elapsed_last = delta_t_elapsed;
		// data_rate Gbit/s
		data_rate = ( (double)del_bytes_sent * (double)8.0 ) / ((double)del_t_elapsed * 1e3);

		/* Read TCP stats snmp info - has to be cumulative as dont know when receiver prints data */
		net_snmp_Stop(  &net_snmp_stats );
		net_snmp_Info(  &net_snmp_stats, net_if_info, &snmp_info);

		/* get the tcp_info information about this socket */
		sock_get_tcpinfo(tcp_soc , &tcp_info);
		// Cwnd MBytes
		tcpi_snd_cwnd = ((double)tcp_info.tcpi_snd_cwnd * (double)tcp_info.tcpi_snd_mss)/1e6;
		del_tcpi_total_retrans = tcp_info.tcpi_total_retrans - tcpi_total_retrans_last;
		tcpi_total_retrans_last = tcp_info.tcpi_total_retrans;
		//printf("Time\t Bytes sent\t Rate Gbit/s\t TCPRetrans\t Cwnd MBytes\t TCPInSegs\t TCPOutSegs \n"); 
		printf("%d", time_alive_sec);
		printf("\t%"LONG_FORMAT"d", del_bytes_sent);
		printf("\t%.2f", data_rate);
	    printf("\t\t%d", del_tcpi_total_retrans ); 
	    printf("\t\t%.2f", tcpi_snd_cwnd ); 
	    printf("\t\t%"LONG_FORMAT"d", snmp_info.TCPInSegs ); 
	    printf("\t\t%"LONG_FORMAT"d", snmp_info.TCPOutSegs ); 
	    //printf("%"LONG_FORMAT"d ", snmp_info.TCPRetransSegs ); 
		
	    printf("\n");
	
    }
    else {
        /* run_time_sec timer determining the length of the test has expired */
        loop_max = 0;
    }
    return;
}

