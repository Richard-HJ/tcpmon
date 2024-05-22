/*
     tcpmon_recv.c     R. Hughes-Jones  The University of Manchester

     Aim is to receive a stream of TCP messages
     Use TCP socket to:
	   receive a series of packets from a remote node
     on cntl-C Print local stats

*/

/*
   Copyright (c) 2015,2016,2017,2018,2019,2020 Richard Hughes-Jones, University of Manchester
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
    unsigned char tcp_data_recv[TCP_DATA_MAX];   /* TCP data  received */
    unsigned char tcp_params_start[sizeof(struct param)];         /* TCP parameter data received at start of message */
	struct tcp_info  tcp_info_data;              /* tcp_info from sender  as differences */

//    unsigned char tcp_params_end[sizeof(struct param)];           /* TCP parameter data received at end of message */
    int *int_data_ptr;                           /* int pointer to data */

#define ERROR_MSG_SIZE 256
    char error_msg[ERROR_MSG_SIZE];              /* buffer for error messages */

#define LISTENQ 1204					         /* defines the maximum length the queue of pending connections can be */

#define ITEMS_PER_G_RECV 2

/* for print_stats_hist( int mode) */
#define MODE_TABLE   1
#define MODE_HEADER  2
#define MODE_LINE    4
#define MODE_DATA    8

/* parameters */
    char dest_ip_address[HOSTNAME_MAXLEN];       /* actually the IP adddress that sends the packets */
    int dest_tcp_port;
    int pkt_len = 64;                            /* length of request packet */
    int resp_len;                                /* length of response frame */
    int soc_buf_size = 0;                        /* send & recv buffer size bytes */
    int precidence_bits=0;                       /* precidence bits for the TOS field of the IP header IPTOS_TOS_MASK = 0x1E */
    int tos_bits=0;                              /* tos bits for the TOS field of the IP header IPTOS_PREC_MASK = 0xE0 */
    int tos_set = 0;                             /* flag =1 if precidence or tos bits set */
    int dscp_bits=0;                             /* difserv code point bits for the TOS field of the IP header */
    int dscp_set = 0;                            /* flag =1 if dscp bits set - alternative to precidence or tos bits set */
    int verbose =0;                  		     /* set to 1 for printout (-v) */
    int use_IPv6 =0;                             /* set to 1 to use the IPv6 address family */
    int get_hist =0;            	             /* set to 1 for histograms */
    int get_info =0;            	             /* set to 1 for information = relative packet arrival times */
    int bin_width =1;                            /* bin width of interframe time histo */
    int low_lim =0;                              /* low limit of interframe time histo */
    int info_data_len = 0;                       /* length in bytes of info data to be returned by -G option */
    int interval_stats_sec =0;      	         /* time interval between reading TCP stats */ 
    int timer_prog_lifetime =0;                  /* num sec to run the program recving data */
    int quiet = 0;                               /* set =1 for just printout of results - monitor mode */
    int64 n_to_skip=0;                           /* number of packets to skip before recording data for -G option */
    long cpu_affinity_mask;                      /* cpu affinity mask set bitwise cpu_no 3 2 1 0 in hex */
    char *interface_name[2];                     /* name of the interface e.g. eth0 */
	int interface_index = 0;                     /* which NIC interface to use */
    int chunk_size = 65536;                      /*read chunk size bytes */
	int force_set_mss=0;                         /* =1 set MSS to 1460 Bytes */
	
    struct param *params;
    struct param *params_tcpinfo;

/* control */
    int first=1;                                 /* flag to indicate that the next frame read will be the first in test */
    int timer_first =1;                          /* flag to indicate that this is the first time the timer fired */
 
/* for command line options */
extern char *optarg;

/* timing */
    struct timeval start;           	        /* time before measurements */
    struct timeval now;            	            /* time after measurements */
    int now_sec_first;                          /* seconds value for the first  gettimeofday(now) */

/* timers */
    StopWatch relative_sw;                      /* to measure total time to send data */
    StopWatch relative_last_sw;                 /* to measure time to send last set of data */
    StopWatch first_last_sw;                    /* time between first and last packet seen */

/* statistics */
    struct HIST hist[10];
    int64 num_recv=0;                           /* total no. of packets sent */
    int64 num_recv_last=0;                      /* previous no. of packets sent */
    int64 bytes_recv;                           /* total bytes received */
    int64 bytes_recv_last;                      /* previous no. of bytes received */

    int64 TCPInSegs_last=0;                     /* counts for sender - needed as send cumulative counts - dont know when it is sampled */
    int64 TCPOutSegs_last=0;
    int64 TCPRetransSegs_last=0;                

    int64 recv_TCPInSegs_sum =0;                /* counts for receiver - needed as measure  incremental counts - snap */
    int64 recv_TCPOutSegs_sum =0;

    int num_output=0;                           /* line number for output - num times timer fired */

    CPUStat cpu_stats;
    CPUinfo cpuinfo[NET_SNMP_MAX_CPU+1];
    Interrupt_info  inter_info[NET_SNMP_MAX_IF];  
    NET_SNMPStat net_snmp_stats;
    NETIFinfo net_if_info[NET_SNMP_MAX_IF];
    SNMPinfo snmp_info;

    NIC_Stat nic_stats[2];
    NICinfo  nic_info[2];

    struct InfoStore infostore;

/* forward declarations */
static void parse_command_line (int argc, char **argv);
static void sig_alarm(int signo);
static void cntlc_handler(int signo);
static void cntlz_handler(int signo);

static void print_stats_hist( int mode);

void print_msg(unsigned char *buf, int len)
  /* --------------------------------------------------------------------- */
{
	int j;
	printf("msg:\n");
	for(j=0; j<len; j++){
		printf(" %x", buf[j]);
	}
	printf("\n");
}

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
				if(nread ==0 && errno_save ==0) return(-1); // link closed
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

int main (int argc, char **argv)
/* --------------------------------------------------------------------- */
{
/* 
thernet frame for IP:
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
    struct sockaddr *soc_recv_address;
    struct addrinfo hint_options, *result;      /* for getaddrinfo() */
    struct addrinfo *rp;                        /* for debug listing */

    SOC_INFO soc_info;
    int ipfamily_addr_len;                      /* length of soc_recv_address - depends on IP family IPv4, IPv6*/

    int listen_soc = 0;		                    /* handle for socket used to listen for connection requests */
    int tcp_soc = 0;		                    /* handle for TCP socket used for data */
    int udp_soc = 0;		                    /* handle for UDP socket used for NIC_Stats */
 
/* timing */
    double ipg_time;
    long hist_time;
    long delay;                      		    /* time for one message loop */
    int64 int64_time;

    struct itimerval timer_value;               /* for the interval timer */
    struct itimerval timer_old_value;  
    int timer_val =0;                           /* value to set the interval timer */

/* timers */
    StopWatch ipg_sw;                           /* time between packets */
    StopWatch stopwatch_getTime;
	StopWatch stopwatch_elapsed;                // data transfer time & elapsed time through the transfer
    double delta_t;
    double delta_t_elapsed = -1;                /* time between curent packet and first seen - StopWatch us */
/* statistics */
	int info_lines = 2000;                      /* for InfoStore */
	int info_items = 10;

    int recv_bufSize, send_bufSize;
    struct tcp_info tcpInfo;

/* local variables */
	int i;
    int error=0;
    int errno_save;
    char *ptr = NULL;
    int nbytes_left = 0;		                /* no of bytes left of message to read from TCP stream */
    int nread = 0;			                    /* no of bytes read from TCP stream */
	int recv_loop = 0;                          /* num times called recvfrom() to receive the block of data */
    int msg_len_read;                           /* length of message read from TCP stream */
	int read_len = 0;                           /* no of bytes for recvfrom() to read from TCP stream */
    int len = 0;
    int value = 0;			                    /* parameter for set soc options */
	int mss;                                    /* TCP max segment size from getsockopt() */
    int cmd;
	int send_loop = 0;
	int extended_output = 1;
	int index;

    int64 inc;                                  /* number by which the frame number increments */
    int64 frame_num;                            /* number of frame just received */ 
    int64 old_frame_num = -1;                   /* number of previous frame received */  
//    int64 old_frame_recv_time =0;               /* time previous frame was received 0.1 us */
//    int64 old_frame_send_time =0;               /* time previous frame was sent 0.1 us */
    int ret;
    char port_str[128];
 
	double time_per_frame;
    double data_rate;   

/* set the signal handler for SIGALRM */
    signal (SIGALRM, sig_alarm);
/* define signal handler for cntl_c */
    signal(SIGINT, cntlc_handler);
/* define signal handler for cntl_z */
    signal(SIGTSTP, cntlz_handler);

/* book histograms */ 
/*     h_book(struct HIST *hist_ptr, int32 id, int32 low_lim, int32 bin_width, 
		int32 n_bins, char *char_ptr)  */ 
    h_book( &hist[0], 0, 0, 1, 150, "Time between messages us");
    h_book( &hist[1], 0, 0, 20, 150, "Message read time us");

    InfoStore_Init( &infostore, "network_time_us; read blksize_bytes; recv_loop; tcp_soc; elapsed_time_us; recv_bufSize; send_bufSize; tcpi_rcv_ssthresh; tcpi_rcv_rtt; tcpi_rcv_space ",
					info_items, info_lines);

/* setup defaults for parameters */
    resp_len = sizeof(struct param);
	interface_name[0]= NULL;	
	interface_name[1]= NULL;	
	delay = 0.0;

    /* these are initialised on CMD_ZEROSTATS */
    relative_sw.t1 = 0; 
    relative_sw.t2 = 0;
    ipg_sw.t1 = 0;
    ipg_sw.t2 = 0;
    first_last_sw.t1 = 0;
    first_last_sw.t2 = 0;

/* Set the default IP Ethernet */
/* IP protocol number ICMP=1 IGMP=2 TCP=6 UDP=17 */
    dest_ip_address[0]=0;
    dest_tcp_port = 0x3799;           		    /* The default RAW/UDP port number (14233 base10) */

/* get the input parameters */
    parse_command_line ( argc, argv);

/* set histogram parameters */
    hist[0].bin_width = bin_width;
    hist[0].low_lim = low_lim;

/* set the CPU affinity of this process*/
    set_cpu_affinity (cpu_affinity_mask, quiet);

/* initalise and calibrate the time measurement system */
    ret = RealTime_Initialise(1);
    if (ret) exit(EXIT_FAILURE);
    ret = StopWatch_Initialise(1);
    if (ret) exit(EXIT_FAILURE);

/* initalise CPUStats */
    CPUStat_Init();

/* create the socket address with the correct IP family and port for listening to TCP packets */
	sprintf(port_str, "%d", dest_tcp_port);
    /* clear then load the hints */
    bzero(&hint_options, sizeof(struct addrinfo) );
    hint_options.ai_family = AF_INET;
#ifdef	IPV6
    if(use_IPv6 == 1) hint_options.ai_family = AF_INET6;
#endif
    hint_options.ai_socktype = SOCK_STREAM;
    hint_options.ai_flags = AI_PASSIVE; /* for server */
    error = getaddrinfo(NULL, port_str, &hint_options, &result);   
    if(error){
		snprintf(error_msg, ERROR_MSG_SIZE,
				"Error: Could not use address family %s", gai_strerror(error) );
		perror(error_msg );
        exit(EXIT_FAILURE);
    }

    if(verbose){
		printf(" Socket address IP family info found:\n");
		for (rp = result; rp != NULL; rp = rp->ai_next) {
			printf(" family %d \n socktype %d \n protocol %d ipfamily_addr_len %d\n", 
				rp->ai_family, rp->ai_socktype, rp->ai_protocol, rp->ai_addrlen);
		}
    } 

    /* loop over returned socket address info */
    do{ 
		/* get the length of the sock address struct */
        ipfamily_addr_len = result->ai_addrlen;
		soc_address = result->ai_addr;

		/* Open the TCP IP socket. */
		soc_info.soc_buf_size = soc_buf_size;
		soc_info.precidence_bits = precidence_bits;
		soc_info.tos_bits = tos_bits;
		soc_info.tos_set = tos_set;
		soc_info.dscp_bits = dscp_bits;
		soc_info.dscp_set = dscp_set;
		soc_info.quiet = quiet;
		sock_create_tcp_socket(&listen_soc, &soc_info, result->ai_family);

		/* bind TCP port to listen socket */
		error = bind(listen_soc, soc_address, ipfamily_addr_len );
		if (error == 0) break; /* OK*/

		/* close unused socket - try next one */
		close (listen_soc);
    } while(  (result = result->ai_next) != NULL);

    if (result == NULL) {
        errno_save = errno;
        sprintf(error_msg, 
		"Error: Bind of port %d to TCP socket failed:", dest_tcp_port);
        errno = errno_save;
		perror(error_msg );
        exit(EXIT_FAILURE);
    }

    if(!quiet) printf(" The TCP port is   %d 0x%x\n", dest_tcp_port, dest_tcp_port);

/* set socket options SO_REUSEADDR - so can re-listen if an old link is still in TIME_WAIT state*/
	len = sizeof(value);
	value = 1;
	error = setsockopt(listen_soc, SOL_SOCKET, SO_REUSEADDR, &value, len);
	if (error) {
		perror("setsockopt( SO_REUSEADDR) on Listen socket failed :" );
		exit(EXIT_FAILURE);
	}
	/* set the TCP MSS for the connected socket  RHJ hack if MTU discovery not enabled */
	/* Set MSS to 1460 Bytes 1500 -20IP -20TCP. You get 1448 as the TCP stack allows for an extra 12-bytes for the TCP Timestamp option to protect against wrapped TCP sequence numbers */
	if(force_set_mss ==1){
		mss = 1460;
		len = sizeof( mss );
		error = setsockopt( listen_soc, IPPROTO_TCP, TCP_MAXSEG, &mss,  len );
		if (error) {
			perror("setsockopt( TCP_MAXSEG) on Listen socket failed :" );
			exit(EXIT_FAILURE);
		}
	}
/* Open the UDP IP socket. - for NIC stats */
    soc_info.soc_buf_size = 0;
    soc_info.precidence_bits = 0;
    soc_info.tos_bits = 0;
    soc_info.tos_set = 0;
    soc_info.dscp_bits = 0;
    soc_info.dscp_set = 0;
    soc_info.quiet = 1;       // do not need to print settings
    sock_create_udp_socket(&udp_soc, &soc_info, AF_INET);
	
/* initalise NIC Stats */
    nic_stats_Init( &nic_stats[0], udp_soc, interface_name[0]);
    nic_stats_Init( &nic_stats[1], udp_soc, interface_name[1]);

/* allocate space for recv address - size depends on IPv4 or IPv6 */
   soc_recv_address = malloc(ipfamily_addr_len);

/* convert socket to listen for an incoming TCP connection request */
	error = listen(listen_soc, LISTENQ );
	if (error) {
		perror("Listen on TCP IP socket failed :" );
		exit(EXIT_FAILURE);
	}

/* check which timer to set: timer_prog_lifetime to exit the program and print stats and hitograms */
	if(timer_prog_lifetime >0) timer_val = timer_prog_lifetime;
// ** check re if((timer_interval >0) && (cpu_affinity_mask_stats <=0) ) timer_val = timer_interval;
	if(interval_stats_sec >0) timer_val = interval_stats_sec;

/* check if need to set the timer to exit the program and print stats and histograms */
	if(timer_val >0){
		timer_value.it_interval.tv_sec = timer_val;              /* Value to reset the timer when the it_value time elapses:*/
		timer_value.it_interval.tv_usec = 0;                     /*  (in us) */
		timer_value.it_value.tv_sec = timer_val;                 /* Time to the next timer expiration: 1 seconds */
		timer_value.it_value.tv_usec = 0;                        /*  (in us) */
		/* set the interval timer to be decremented in real time */
		ret = setitimer( ITIMER_REAL, &timer_value, &timer_old_value );
		if(ret){
			perror("set interval timer failed :" );
			exit(EXIT_FAILURE);
		}
    }
/* dont need a stats Output thread as print at the end of each message */

/* clear the local stats */
	num_recv = 0;
    bytes_recv = 0;

/* point params struct to the data buffer */
    params = (struct param *) tcp_params_start;


/* loop for ever waiting for a connection request */
    for(;;){
		if(verbose) {
			printf("Wait for connection  \n");
		}      
        /* wait for an incoming TCP connection request and accept it */
		tcp_soc = accept(listen_soc,  (struct  sockaddr*)&soc_recv_address, (socklen_t *)&ipfamily_addr_len );
		if (tcp_soc < 0) {
			if(verbose) {
				perror("Connection accept on TCP IP socket failed :" );
			}
			goto CLOSE_LINK;
		}	
		/* set the socket TCP options - turn off Nagel & delayed ack 
		see also /proc/sys/net/ipv4/tcp_delack_min */
		len = sizeof(value);
		value = 1;
		error = setsockopt(tcp_soc, IPPROTO_TCP, TCP_NODELAY, &value, len);

		/* find the TCP MSS for the connected socket */
		len = sizeof( mss );
		ret = getsockopt( tcp_soc, IPPROTO_TCP, TCP_MAXSEG, (char*) &mss, (socklen_t *) &len );
	
		if(!quiet){
		printf(" The TCP maximum segment size is   %d\n", mss);
		}
		if(verbose) {
			printf("Connection accepted \n");
		}
		/* zero variables */
		frame_num = 0;                        /* number of frame just received */
		old_frame_num = -1;                   /* number of previous frame received */
		first = 1;
		num_recv = 0;                         /* the number of packets seen */
		bytes_recv = 0;
		
		h_clear( &hist[0] ); 
		h_clear( &hist[1] ); 

		/* record initial interface & snmp info */
		net_snmp_Start(  &net_snmp_stats);
		/* record initial info from NIC */
		nic_stats_Start( &nic_stats[0]);
		nic_stats_Start( &nic_stats[1]);
		/* record initial CPU and interrupt info */
		CPUStat_Start(  &cpu_stats);

		/* set a time zero */
		gettimeofday(&start, NULL);
		StopWatch_Start(&ipg_sw);		
		StopWatch_Start(&stopwatch_elapsed);

		/* loop for ever over reading from the TCP socket*/
		for(;;){
			sock_get_tcp_bufsize(tcp_soc , &recv_bufSize, &send_bufSize);
			sock_get_tcpinfo(tcp_soc , &tcpInfo);
			StopWatch_Stop(&stopwatch_elapsed); // time through the transfer
			delta_t_elapsed = StopWatch_TimeDiff(&stopwatch_elapsed);

			/* time the network receive */
			StopWatch_Start(&stopwatch_getTime);
			/* First do the minimum required - read enough data to get params so can find the length of the entire message */
			ptr = (char*) &tcp_params_start;
			read_len= sizeof( struct param );
			recv_loop = 0;
			nread = tcp_read(tcp_soc, ptr, read_len, &recv_loop);
			if(nread == -1) goto CLOSE_LINK;
			if(verbose) {
				printf("----++\n");	
				printf("Read params length: %d bytes\n", nread);
				printf(" cmd %d Frame num %" LONG_FORMAT "d \n",  params->cmd, params->frame_num);
			}
			msg_len_read = nread ;
			
			//StopWatch_Start(&msg_sw);				/* start after params read - avoid waiting for the sender inter-message gap */

			/* check for a request of > sizeof( struct param ) bytes and read the rest of the message*/
			nbytes_left = i4swap(params->msg_len) - sizeof( struct param ); // ** check dont overflow tcp_data_recv
			if(nbytes_left >0) {
				ptr = (char*) &tcp_data_recv;
				recv_loop = 0;
				nread = tcp_read(tcp_soc, ptr, nbytes_left, &recv_loop);
				if(nread == -1) goto CLOSE_LINK;
				if(verbose) {
					printf("Read message length: %d bytes\n", nread);
				}
				msg_len_read = nread + sizeof( struct param );
			}
			

			/* what do we have to do */
			cmd = i4swap(params->cmd);
			frame_num = i8swap(params->frame_num);
			if(verbose) {
				printf("Message length: %d bytes\n", msg_len_read);
				print_msg(tcp_params_start, 64);
				printf("Command : %d Frame num %" LONG_FORMAT "d old frame num %" LONG_FORMAT "d\n", cmd, frame_num, old_frame_num);
			}

			switch (cmd){

			case CMD_DATA:
				/* record the time for receiving the message */
				StopWatch_Stop(&stopwatch_getTime);
				delta_t = StopWatch_TimeDiff(&stopwatch_getTime);
				h_fill1(&hist[1], (int64_t)delta_t);

				/* record the time between the frame arrivals */
				StopWatch_Stop(&ipg_sw);
				ipg_time = StopWatch_TimeDiff(&ipg_sw);
				relative_sw.t2 = ipg_sw.t2;
				StopWatch_Start(&ipg_sw);
				/* histogram with 1us  bins*/
				hist_time = (long) (ipg_time);
				h_fill1( &hist[0], hist_time);

				InfoStore_Store(&infostore, (int64_t)delta_t);
				InfoStore_Store(&infostore, (int64_t)msg_len_read);
				InfoStore_Store(&infostore, (int64_t)recv_loop);
				InfoStore_Store(&infostore, (int64_t)tcp_soc);
				InfoStore_Store(&infostore, (int64_t)delta_t_elapsed);
				InfoStore_Store(&infostore, (int64_t)recv_bufSize);
				InfoStore_Store(&infostore, (int64_t)send_bufSize);
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_rcv_ssthresh);
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_rcv_rtt);
				InfoStore_Store(&infostore, (int64_t)tcpInfo.tcpi_rcv_space);

				num_recv++;  /* the number of packets seen */
				bytes_recv = bytes_recv + (int64)msg_len_read;

				if(first ==1){
					/* set a time zero for average throughput */
					/* record time of first frame seen */
					relative_sw.t1 = ipg_sw.t2;
					relative_last_sw.t1 = ipg_sw.t2;
					first_last_sw.t1 =ipg_sw.t2;
					first =0;
				}
				else if(first ==2){
					/* set the time zero for throughput for the _last data */
					relative_last_sw.t1 = ipg_sw.t2;
					first =0;
				}

				/* check increment of msg number */
				inc = frame_num - old_frame_num;
				if(inc == 1){
				}

				if(inc >0){
					old_frame_num = frame_num;
					/* record the time this frame was seen relative to the time of the zerostats command - in us */
				}
			break;

			case CMD_ZEROSTATS:
				if(verbose)	printf("zerostats\n");
				/* clear the local stats */
				num_recv = 0;
				bytes_recv = 0;
				net_snmp_Start(  &net_snmp_stats);
				/* record initial info from NIC */
				nic_stats_Start( &nic_stats[0]);
				nic_stats_Start( &nic_stats[1]);
				/* record initial CPU and interrupt info */
				CPUStat_Start(  &cpu_stats);
				/* record time of first frame seen */
				first =1;
				//StopWatch_Start(&ipg_sw);		
				h_clear( &hist[0] ); 
				h_clear( &hist[1] ); 
			
				/* send the OK */
				resp_len= sizeof( struct param );
				params->msg_len = resp_len;
				error = tcp_write(tcp_soc, (char *)params, resp_len, &send_loop);
				if(error != resp_len) {
					sprintf(error_msg, 
						"Error: on data sent GETSTATS: sent %d bytes not %d ", 
						error, resp_len );
					perror(error_msg );
				}		
			break;

			case CMD_GETSTATS:
				if(verbose) {
					printf("get stats resp_len %d\n", params->resp_len);
				}
				/* record time of first frame seen */
				first_last_sw.t2 =ipg_sw.t2;
                delay = StopWatch_TimeDiff(&first_last_sw);
				/* record final CPU and interrupt info */
				CPUStat_Stop( &cpu_stats);
				/* record final interface & snmp info */
				net_snmp_Stop(  &net_snmp_stats);
				/* record final info from the NIC */
				nic_stats_Stop( &nic_stats[0]);
				nic_stats_Info( &nic_stats[0], &nic_info[0]);
				nic_stats_Stop( &nic_stats[1]);
				nic_stats_Info( &nic_stats[1], &nic_info[1]);

				
				/* sort byte swapping */
				params->num_recv = i4swap(num_recv);
				params->bytes_recv = i8swap(bytes_recv);
				params->soc_buf_size = i4swap(soc_buf_size);
				int64_time = (int64)(delay);
				params->first_last_time = i8swap(int64_time);
				resp_len= sizeof( struct param );
				params->msg_len = resp_len;
				
				if(verbose){
					printf("num_recv       : %"LONG_FORMAT"d\n", num_recv);
					printf("bytes_recv     : %"LONG_FORMAT"d\n", bytes_recv);
					printf("soc_buf_size   : %d\n", soc_buf_size);
					printf("first_last_time: %"LONG_FORMAT"d\n", int64_time);
					
					net_snmp_Info(  &net_snmp_stats, net_if_info, &snmp_info);

					printf("L if;");
					for (i=0; i<5; i++){
						if(net_if_info[i].name[0] != 0){
							printf( "-%d-%s: ; %" LONG_FORMAT "d; %" LONG_FORMAT "d; %" LONG_FORMAT "d; %" LONG_FORMAT "d;", 
									i,&net_if_info[i].name[0], net_if_info[i].pktsin, net_if_info[i].bytesin,
									net_if_info[i].pktsout, net_if_info[i].bytesout);
						}
					}	
					printf(" L snmp;");
					printf( " %" LONG_FORMAT "d; %" LONG_FORMAT "d; %" LONG_FORMAT "d; %" LONG_FORMAT "d; ", 
							snmp_info.InReceives, snmp_info.InDiscards,
							snmp_info.OutRequests, snmp_info.OutDiscards);
					printf("\n");
				}
				error = tcp_write(tcp_soc, (char *)params, resp_len, &send_loop);
				if(error != resp_len) {
					sprintf(error_msg, 
						"Error: on data sent GETSTATS: sent %d bytes not %d ", 
						error, resp_len );
					perror(error_msg );
				}		
			break;

			case CMD_GETCPULOAD:
				resp_len = i4swap(params->resp_len);
				if(verbose) {
					printf("get cpuload resp_len %d\n", resp_len);
				}
				CPUStat_Info(  &cpu_stats, cpuinfo, inter_info);
				/* should allow for byte swapping */

				/* send the response frame(s)  */		
				error = tcp_write(tcp_soc, (char *)&cpuinfo, sizeof(cpuinfo), &send_loop);
				if(error != resp_len) {
					sprintf(error_msg, 
						"Error: on data sent GETCPULOAD: sent %d bytes not %d ", 
						error, resp_len );
					perror(error_msg );
				}
			break;
			
			case CMD_GETNETSNMP:
				resp_len = i4swap(params->resp_len);
				if(verbose) {
					printf("get net_snmp resp_len %d\n", resp_len);
				}
				/* should byte swap - but only once */

				/* send the response frame(s)  */		
				error = tcp_write(tcp_soc, (char *)&net_snmp_stats, sizeof(net_snmp_stats), &send_loop);
				if(error != resp_len) {
					sprintf(error_msg, 
						"Error: on data sent GETNETSNMP: sent %d bytes not %d ", 
						error, resp_len );
					perror(error_msg );
				}
			break;

			case CMD_GETNICSTATS:
				resp_len = i4swap(params->resp_len);
				index = i4swap(params->index);
				if(verbose) {
					printf("get nic_stats resp_len %d index %d\n", resp_len, index);
					nic_stats_print_info(&nic_info[index], 1, 'L');
					printf("\n");
					nic_stats_print_info(&nic_info[index], 2, 'L');
					printf("\n");
				}
				/* should byte swap - but only once */

				/* send the response frame(s)  */
				error = tcp_write(tcp_soc, (char *)&nic_info[index], sizeof(NICinfo), &send_loop);
				if(error != resp_len) {
					sprintf(error_msg,
						"Error: on data sent GETNICSTATA: sent %d bytes not %d ",
						error, resp_len );
					perror(error_msg );
				}
			break;
			
			default:
			break;

			}   /* end of switch() */
			if(verbose) printf("------\n");	
	
		}    /* end of loop receiving messages from tcp socket */

CLOSE_LINK:
		close(tcp_soc);	
		if(verbose) {
			printf("TCP data link closed \n");		

			// print results
			// titles
			printf(" Recv soc_buf_size;");
			printf(" num_recv;");
			printf(" bytes_recv;");
			printf(" Recv time/Frame; recv_time; recv_data_rate Mbit;");

			CPUStat_Info(  &cpu_stats, cpuinfo, inter_info);
			CPUStat_print_cpu_info( cpuinfo, 1, 'L', extended_output);

			net_snmp_Info(  &net_snmp_stats, net_if_info, &snmp_info);
			net_snmp_print_info( net_if_info, &snmp_info, 3, 'L');

			nic_stats_print_info(&nic_info[0], 1, 'L');
			nic_stats_print_info(&nic_info[0], 1, 'L');
			printf("\n");
		
			// data
			time_per_frame = (double)delay/(double)num_recv;
			data_rate = ( (double)bytes_recv * 8.0 ) / (double)delay;

			printf(" %d; ", soc_buf_size);
			printf(" %"LONG_FORMAT"d; ", num_recv);
			printf(" %"LONG_FORMAT"d ", bytes_recv );
			printf(" %g; ", time_per_frame );
			printf(" %"LONG_FORMAT"d; ", delay);
			printf(" %g; ", data_rate);

			/* print total local CPU info */
			CPUStat_print_cpu_info( cpuinfo, 2, 'L', extended_output);
			/* print local interface & snmp info */
			net_snmp_print_info( net_if_info, &snmp_info, 4, 'L');
			/* print NIC info */
			nic_stats_print_info(&nic_info[0], 2, 'L');
			nic_stats_print_info(&nic_info[1], 2, 'L');
		
			printf("\n");

			if(get_hist){
				h_output( &hist[0]);
				h_output( &hist[1]);
			} /* end of if(get_hist) */ 

			if(get_info){
				InfoStore_Print(&infostore );
				close(listen_soc);
				close(tcp_soc);
				close(udp_soc);
				exit(0);		// exit as the InfoStore only applies to one run
			}
		}
		
    }	/* end of for ever looping waiting for tcp connections */
    
    close(listen_soc);
    close(tcp_soc);
    close(udp_soc);

    return(0);    
}

static void parse_command_line (int argc, char **argv)
/* --------------------------------------------------------------------- */
{
/* local variables */
    char c;
    int error;
    time_t date;
    char *date_str;
    int i;
    char cmd_text[128];
    char *str_loc;
	int core;

    error =0;
    char *help ={
"Usage: udpmon_bw_mon -option<parameter> [...]\n\
options:\n\
	 -6 = Use IPv6\n\
	 -A = <cpu core to use - start from 0 >\n\
	 -B = <bin width of remote histo in us>\n\
	 -F = Force MSS to 1460 Bytes\n\
	 -G = <number of messages on which to return information>\n\
     -H = Print histograms\n\
     -I = <interface name for NIC information e.g. enp131s0f1 [NULL]>\n\
	 -L = <[number of messages to skip:]number of LOST messages on which to return information>\n\
	 -M = <min (low limit) of remote histo in us>\n\
	 -Q = <DSCP QoS bits set - in hex >\n\
	 -S = <size of send and receive socket buffers in bytes>\n\
     -V = print version number\n\
	 -a = <cpu_mask set bitwise cpu_no 3 2 1 0 in hex>\n\
	 -c = read chunk size bytes \n\
	 -d = <the IP name or IP address to print a.b.c.d>\n\
	 -h = print this message\n\
	 -i = time in sec between printing statistic snapshots [0=never]\n\
	 -q = quiet - only print results\n\
     -t = time in sec before program ends [0=never]\n\
	 -u = <tcp port no - default 0x3799 ie 14233 decimal>\n\
     -v = turn on debug printout"};


#ifdef IPv6
    while ((c = getopt(argc, argv, "a:c:d:i:t:u:A:B:G:I:L:M:Q:S:hqv6FHV")) != (char) EOF) {
#else
    while ((c = getopt(argc, argv, "a:c:d:i:t:u:A:B:G:I:L:M:Q:S:hqvFHV")) != (char) EOF) {
#endif	
	switch(c) {

	    case 'a':
		if (optarg != NULL) {
		    sscanf(optarg, "%lx", &cpu_affinity_mask);
		} else {
		    error = 1;
		}
		break;

	    case 'c':
		if (optarg != NULL) {
		   chunk_size = atoi(optarg);
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

	    case 'q':
	        quiet = 1;
		break;

	    case 't':
		if (optarg != NULL) {
		   timer_prog_lifetime = atoi(optarg);
		}
		break;

	    case 'u':
		if (optarg != NULL) {
		    dest_tcp_port =  atoi(optarg); 
		} else {
		    error = 1;
		}
		break;

	    case 'v':
	        verbose = 1;
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

 	    case 'F':
	        force_set_mss = 1;
		break;

	    case 'H':
	        get_hist = 1;
		break;

		case 'I':
		if (optarg != NULL) {
			interface_name[interface_index] = optarg;
			interface_index++;
		} else {
			error = 1;
		}
		break;

	    case 'G':
		if (optarg != NULL) {
			memset(cmd_text, 0, strlen(cmd_text));
			strcpy(cmd_text,  optarg);
			str_loc = strstr(cmd_text, ":");
			if (str_loc) {
				*str_loc=' ';
				sscanf(cmd_text, "%"LONG_FORMAT"d %d", &n_to_skip, &info_data_len);
			}
			else {
				n_to_skip =0;
				sscanf(cmd_text, "%d", &info_data_len);
			}
			info_data_len = info_data_len*sizeof(int64)*ITEMS_PER_G_RECV;  /* *8 for bytes *2 as 2 words recorded per frame */
		}
	        get_info = 1;
		break;

	    case 'M':
		if (optarg != NULL) {
			low_lim =  atoi(optarg);
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

	    case 'V':
	        printf(" %s \n", TCPMON_VERSION);
	        exit(EXIT_SUCCESS);
		break;

	    default:
		break;
	}   /* end of switch */
    }       /* end of while() */

    if (error) {
	fprintf (stderr, "%s \n", help);
	exit	(EXIT_FAILURE);
    }

    date = time(NULL);
    date_str = ctime(&date);
    date_str[strlen(date_str)-1]=0;
    printf(" %s :", date_str );
    printf(" %s ", TCPMON_VERSION);
    printf(" Command line: ");
    for(i=0; i<argc; i++){
		printf(" %s", argv[i]);
    }
    printf(" \n");
}

static void print_stats_hist( int mode)
/* --------------------------------------------------------------------- */
{
/*
    mode 
    = MODE_TABLE    print the table
	= MODE_HEADER   print text header line and then line of results
	= MODE_LINE     just print line of results
	= MODE_DATA     print hist + info + lost

	* * * * 
	Note the order of testing mode and doing the printout is required to give the desired output
	* * * * 
*/

  int64 rx_out_of_buffer;

/* for receiver ie local */

  double data_rate = 0.0;
  double data_rate_last = 0.0;
  double wire_rate = 0.0;
  double wire_rate_last = 0.0;
  double elapsed_time;
  double elapsed_time_last;
  int bytes_per_frame;
  int bytes_per_frame_last;
  int extended_output =1;
  double recv_time_packet;
  double recv_time_packet_last;
  double TCPRetransSegs_pcent;
  double TCPRetransSegs_pcent_last;
  int update_last_counters =0;

  int delta_sec;
  double delta_hr;
  double excel_time;
  int win_scale;
  int64 tcp_window;


  //  printf(" -------------- TCPInSegs_last %"LONG_FORMAT"d TCPOutSegs_last %"LONG_FORMAT"d\n", TCPInSegs_last, TCPOutSegs_last);

	bytes_per_frame = (num_recv >0)? bytes_recv/num_recv : 0;
	bytes_per_frame_last = ((num_recv - num_recv_last) >0)? (bytes_recv - bytes_recv_last)/(num_recv - num_recv_last) : 0;

	/* calc the data rate seen */
	elapsed_time = StopWatch_TimeDiff(&relative_sw);

	relative_last_sw.t2 = relative_sw.t2;
	elapsed_time_last = StopWatch_TimeDiff(&relative_last_sw);
	recv_time_packet = ((num_recv )>0)?
						elapsed_time/(double)(num_recv ) : 0;
	recv_time_packet_last = ((num_recv - num_recv_last )>0)?
						elapsed_time_last/(double)(num_recv-num_recv_last ) : 0;

	first = 2;  // next packet gives the start time for the _last data

	/* check we have received packets then elapsed_time >0.0 */
	if(elapsed_time > 0.0) {
	    data_rate =(double)8.0*(double)bytes_recv/elapsed_time;
	    /* IPG 12  Preamble+start 8  eth header 14 eth CRC 4  IP 20 UDP 8 = 66 */
	    wire_rate =(double)8.0*(double)(bytes_recv+ num_recv*(66))/elapsed_time;

	    data_rate_last =(double)8.0*(double)(bytes_recv - bytes_recv_last)/elapsed_time_last;
	    /* IPG 12  Preamble+start 8  eth header 14 eth CRC 4  IP 20 UDP 8 = 66 */
	    wire_rate_last =(double)8.0*(double)((bytes_recv - bytes_recv_last)+ (num_recv-num_recv_last)*66)/elapsed_time_last;
	} else {
	    elapsed_time = 0.0; 
	    elapsed_time_last = 0.0; 
	}

	/* avoid snapping unnecessarily */
	if((mode & (MODE_TABLE | MODE_LINE)) != 0){
		/* get local CPU & interupt info */
		CPUStat_Snap(  &cpu_stats, cpuinfo, inter_info);

		/* get interface & snmp info for the receiver */
		net_snmp_Snap(  &net_snmp_stats, net_if_info, &snmp_info);

	    /* get NIC stats */
	    nic_stats_Snap( &nic_stats[0], &nic_info[0] );
	}

	rx_out_of_buffer = nic_stats_getValue( &nic_info[0], "rx_out_of_buffer");
	/* check for counter not implemented */
	if( rx_out_of_buffer ==-1) rx_out_of_buffer=0;

	/* calculate the culmulative counts */
	recv_TCPInSegs_sum += snmp_info.TCPInSegs;
	recv_TCPOutSegs_sum += snmp_info.TCPOutSegs;

	TCPRetransSegs_pcent = ((params->TCPOutSegs )>0)?
				params->TCPRetransSegs*(double)100.0/(params->TCPOutSegs) : 0;
	TCPRetransSegs_pcent_last = ((params->TCPOutSegs - TCPOutSegs_last)>0)? 
				(double)100.0*(params->TCPRetransSegs - TCPRetransSegs_last)/(double)(params->TCPOutSegs - TCPOutSegs_last) : 0;

	win_scale = params->tcpi_snd_wscale; 
	tcp_window = ((int64)params->tcpi_snd_cwnd )<< win_scale;

/* do printout as selected */
	if((mode & MODE_TABLE) == MODE_TABLE){
	    printf(" \n");
	    printf("Frames recv           : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   num_recv, (num_recv - num_recv_last) );
	    printf("Bytes received        : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   bytes_recv, (bytes_recv - bytes_recv_last) );
	    printf("Bytes/Frame           : %d \t delta: %d\n", bytes_per_frame, bytes_per_frame_last );
	    
	    printf("Elapsed_time us       : %g \t delta: %g\n", elapsed_time, elapsed_time_last);
	    printf("Recv time/pkt us      : %g \t delta: %g\n", recv_time_packet, recv_time_packet_last);
	    printf("User data rate Mbit/s : %g \t delta: %g\n", data_rate, data_rate_last);
	    printf("Wire rate Mbit/s      : %g \t delta: %g\n", wire_rate, wire_rate_last);


	    printf("Send   TCPInSeg       : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   params->TCPInSegs,  (params->TCPInSegs - TCPInSegs_last) );
	    printf("Send   TCPOutSeg      : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   params->TCPOutSegs,  (params->TCPOutSegs - TCPOutSegs_last) );
	    printf("Send TCPReTransmitSeg : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   params->TCPRetransSegs,  (params->TCPRetransSegs - TCPRetransSegs_last) );
	    printf("Send %% TCPReTransmit  : %g \t delta: %g\n", TCPRetransSegs_pcent, TCPRetransSegs_pcent_last);

	    printf("Recv   TCPInSeg       : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   recv_TCPInSegs_sum,  snmp_info.TCPInSegs );
	    printf("Recv   TCPOutSeg      : %"LONG_FORMAT"d \t delta: %"LONG_FORMAT"d\n", 
		   recv_TCPOutSegs_sum ,  snmp_info.TCPOutSegs );

	    printf("snd MSS               : %d \n", params->tcpi_snd_mss);
	    printf("snd RTT               : %d \n", params->tcpi_rtt ); 
	    printf("snd win scale         : %d \n", params->tcpi_snd_wscale );
	    printf("snd cwnd              : %d \n", params->tcpi_snd_cwnd );
	    printf("snd cwnd scaled       : %"LONG_FORMAT"d\n", tcp_window );
	    printf("snd ssthresh          : %d \n", params->tcpi_snd_ssthresh ); 
		printf("snd retrans           : %d \n", params->tcpi_retrans ); 
		printf("snd total retrans     : %d \n", params->tcpi_total_retrans ); 
		printf("snd inital MSS        : %d \n", params->mss ); 

	    update_last_counters = 1;
	}  /* end of MODE_TABLE */

	if((mode & MODE_HEADER) == MODE_HEADER){
	    /* print titles  */
	    printf("num; linux time; excel time; hour;");
	    printf(" num_recv; ");
	    printf(" Bytes Recv; Bytes/frame;");
	    printf(" elapsed time us; time/recv pkt;");
	    printf(" recv_user_data_rate Mbit; recv_wire_rate Mbit;");

	    printf(" R TCP;");
	    printf(" TCPInSegs; TCPOutSegs;");
	    printf(" TCPRetransSegs; %%TCPRetransSegs;");
	    printf(" snd MSS; RTT; win_scale;");
	    printf(" snd cwnd; cwnd_scale; ssthresh;");
	    printf(" retrans; total_retrans;");
		printf(" snd inital MSS;" ); 
	    
	    printf(" L TCP;");
	    printf(" TCPInSegs; TCPOutSegs;");
	    CPUStat_print_cpu_info( cpuinfo, 1, 'L', extended_output);
	    //CPUStat_print_inter_info( inter_info, 1, 'L');
	    net_snmp_print_info( net_if_info, &snmp_info, 3, 'L');
	    nic_stats_print_info(  &nic_info[0], 1, 'L');
	    nic_stats_print_info(  &nic_info[0], 1, 'L');
	    printf(" \n");
	  } /* end of MODE_HEADER */

	if((mode & MODE_LINE) == MODE_LINE){
	    delta_sec = (int) now.tv_sec - now_sec_first;
	    /* calc excel time 
	       Excel stores dates and times as a number representing the number of days since 1900-Jan-0, 
	       plus a fractional portion of a 24 hour day:   ddddd.tttttt
	       SO
	       excel_time = (linux_time now )/(num sec in day) + time(1 jan 1970)*/
	    delta_hr = (double)delta_sec / 3600.0; 
	    excel_time = (double)now.tv_sec/(double)86400.0 + (double)25569.0;
	    printf(" %d; %d; %f; %g;",num_output, (int) now.tv_sec, excel_time, delta_hr);
	    printf(" %"LONG_FORMAT"d;", 
		   (num_recv - num_recv_last) );
	    
	    printf(" %"LONG_FORMAT"d;", (bytes_recv - bytes_recv_last) );
	    printf(" %d;", bytes_per_frame_last );
	    printf(" %g;", elapsed_time_last);
	    printf(" %g;", recv_time_packet_last);
	    printf(" %g; %g;", data_rate_last, wire_rate_last);
	    
	    printf(" R TCP;");
	    printf(" %"LONG_FORMAT"d;", 
		   (params->TCPInSegs - TCPInSegs_last) ); 
	    printf(" %"LONG_FORMAT"d;", 
		   (params->TCPOutSegs - TCPOutSegs_last) ); 
	    printf(" %"LONG_FORMAT"d;", 
		   (params->TCPRetransSegs - TCPRetransSegs_last) ); 
	    printf(" %.2g;", TCPRetransSegs_pcent_last);

	    printf(" %d;", params->tcpi_snd_mss ); 
	    printf(" %d;", params->tcpi_rtt ); 
	    printf(" %d;", params->tcpi_snd_wscale ); 
	    printf(" %d;", params->tcpi_snd_cwnd ); 
	    printf(" %"LONG_FORMAT"d;", tcp_window ); 
	    printf(" %d;", params->tcpi_snd_ssthresh ); 
	    printf(" %d;", params->tcpi_retrans ); 
	    printf(" %d;", params->tcpi_total_retrans ); 
            printf(" %d;", params->mss ); 

	    printf(" L TCP;");
	    printf(" %"LONG_FORMAT"d;", 
		   snmp_info.TCPInSegs ); 
	    printf(" %"LONG_FORMAT"d;", 
		   snmp_info.TCPOutSegs ); 

	    /* print total local CPU info */
	    CPUStat_print_cpu_info( cpuinfo, 2, 'L', extended_output);
	    
	    /* print total local interupt info */
	    //CPUStat_print_inter_info( inter_info, 2, 'L');
	    
	    /* print local interface & snmp info */
	    net_snmp_print_info( net_if_info, &snmp_info, 4, 'L');
		/* print NIC info */
		nic_stats_print_info(  &nic_info[0], 2, 'L');		
		nic_stats_print_info(  &nic_info[1], 2, 'L');		
	    printf(" \n");

	    num_output++;
	    update_last_counters =1;
	}  /* end of MODE_LINE */

	if((mode & MODE_DATA) == MODE_DATA){
		/* Print the histogram data */
		if(get_hist == 1){
			h_output( &hist[0]);
			h_output( &hist[1]);
		} /* end of if(get_hist) */ 


		fflush(stdout);

	} /* end of mode == MODE_DATA */


/* check if have to update the last counters */
	if(update_last_counters == 1){
	    /* update the _last counters */
	    num_recv_last = num_recv;
	    bytes_recv_last = bytes_recv;

	    /* params->TCP* are cumulative */
	    TCPInSegs_last = params->TCPInSegs;
	    TCPOutSegs_last = params->TCPOutSegs;
	    TCPRetransSegs_last = params->TCPRetransSegs;
	}
	//	printf(" -----update counters %d --------- TCPInSegs_last %"LONG_FORMAT"d TCPOutSegs_last %"LONG_FORMAT"d\n", update_last_counters, TCPInSegs_last, TCPOutSegs_last);

}


static void cntlc_handler( int signo)
/* --------------------------------------------------------------------- */
{
/* called on cntl_C */
	int extended_output =1;
	
	printf("cntl-C received \n");

	fflush(stdout);
	printf("\n");
        printf("Frames recv        : %"LONG_FORMAT"d \n", num_recv );
        printf("Bytes recv         : %"LONG_FORMAT"d \n", bytes_recv );

	    CPUStat_print_cpu_info( cpuinfo, 1, 'L', extended_output);
	    net_snmp_print_info( net_if_info, &snmp_info, 3, 'L');
	    nic_stats_print_info(  &nic_info[0], 1, 'L');
	    printf(" \n");

	    /* print total local CPU info */
	    CPUStat_print_cpu_info( cpuinfo, 2, 'L', extended_output);	    
	    /* print local interface & snmp info */
	    net_snmp_print_info( net_if_info, &snmp_info, 4, 'L');
		/* print NIC info */
		nic_stats_print_info(  &nic_info[0], 2, 'L');		
	    printf(" \n");

	exit(0);
	
    return;
}
 
static void cntlz_handler( int signo)
/* --------------------------------------------------------------------- */
{
/* called on cntl_Z */
	fflush(stdout);
	printf("\n");
    printf("Frames recv        : %"LONG_FORMAT"d \n", num_recv );
    printf("Bytes recv         : %"LONG_FORMAT"d \n", bytes_recv );

	printf("cntl-Z received : Process ended\n");
	fflush(stdout);
	exit(0);

    return;
}

static void sig_alarm( int signo)
/* --------------------------------------------------------------------- */
{
  int delay_sec;

/* get the time now */
    gettimeofday(&now, NULL);
    delay_sec = (now.tv_sec - start.tv_sec);

    //    printf("SIGALRM caught start %d now %d time_alive %d\n", start.tv_sec, now.tv_sec, delay_sec);

/* check if we terminate the program or just print snapshot of statistics */
    if(delay_sec < timer_prog_lifetime || (timer_prog_lifetime ==0)){
		/* just print snapshot of statistics, interface & snmp info */
		if(timer_first){
			timer_first =0;
			now_sec_first = (int)now.tv_sec;
			print_stats_hist(MODE_HEADER | MODE_LINE);
		}
		else{
			print_stats_hist(MODE_LINE);
		}
    }

    else{
		/* record final interface & snmp info and exit */
		sleep(1);   // make sure the counters have been updated
		if(timer_first){
			timer_first =0;
			print_stats_hist(MODE_HEADER | MODE_LINE | MODE_DATA);
		}
		else{
			print_stats_hist(MODE_LINE | MODE_DATA);
		}
		exit(0);
    }
	return;
}


