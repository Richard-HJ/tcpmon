bin_PROGRAMS = tcpmon_mon tcpmon_resp tcpmon_transit

ARCH=@ARCH@
HJ_LOCATION=@HJ_LOCATION@

tcpmon_mon_CPPFLAGS = -D$(ARCH) -Wall

tcpmon_resp_CPPFLAGS = -D$(ARCH) -Wall

tcpmon_transit_CPPFLAGS = -D$(ARCH) -Wall

INCLUDES = -I$(HJ_LOCATION)/include/libhj

tcpmon_mon_SOURCES  = tcpmon_mon.c net_test.h 

tcpmon_resp_SOURCES  = tcpmon_resp.c net_test.h 

tcpmon_transit_SOURCES  = tcpmon_transit.c net_test.h 

EXTRA_DIST = tcpmon.spec.in tcpmon.spec bootstrap README cmd_get_sysinfo.pl \
			cmd_tcpmon_bufscan.py cmd_tcpmon_BW_scan.py \
			cmd_run_tcpmon_mon.py cmd_run_tcpmon_multiflow.py cmd_run_tcpmon_stress.py cmd_run_tcpmon_resp.py cmd_tcpmon_send_infoStorescan.py
