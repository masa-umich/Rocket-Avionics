/*
 * log_errors.h
 *
 *  Created on: Jul 28, 2025
 *      Author: felix
 */

// All error/status messages go in this file
// Every message MUST start with the 3 digit code, the first digit that refers to the board is added later.
// Keep all messages as short as possible, details can be added to the error lookup table

#ifndef INC_LOG_ERRORS_H_
#define INC_LOG_ERRORS_H_

// General error messages (all boards could have this happen)
// To keep track of messages, I'm starting these at 200. This is not required, but it's easier to keep track of in this file
#define ERR_FLASH_FULL							"201 Flash full"
#define ERR_FLASH_INIT							"202 EEPROM init failed"
#define ERR_LINK_INIT_DOWN						"203 Ethernet link down"
#define ERR_UDP_INIT_MUTEX						"204 Failed to take UDP mutex during init"
#define ERR_UDP_INIT_NNETCONN					"205 NULL netconn during UDP logging init"
#define ERR_ADS_INIT_THREAD_ERR					"206 ADS thread did not start"
#define ERR_IMU_INIT							"207 IMU init failed, IMU "
#define ERR_BAR_INIT							"208 BAR init failed, BAR "

// Flight Computer error messages
#define FC_ERR_TCP_SERV_SOCK_CREAT_NOBUF		"001 No buffer space - creating server listen socket. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_CREAT_NOSOCK		"002 No available sockets - creating server listen socket. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_CREAT_UNKNOWN		"003 Unknown errno - creating server listen socket. Exiting thread, errno "

#define FC_ERR_TCP_SERV_SOCK_BIND_NSOCK			"004 NULL socket - binding server listen socket. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_BIND_BADPCB		"005 Bad PCB - binding server listen socket. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_BIND_USEDADDR		"006 Address already used - binding server listen socket. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_BIND_UNKNOWN		"007 Unknown errno - binding server listen socket. Exiting thread, errno "

#define FC_ERR_TCP_SERV_SOCK_LISTEN_NSOCK		"008 NULL socket - setting server listen socket to listening. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NNETCONN	"009 NULL netconn - setting server listen socket to listening. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NPCB		"010 NULL PCB - setting server listen socket to listening. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_BADPCB		"011 Bad PCB - setting server listen socket to listening. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NOMEM		"012 No memory - setting server listen socket to listening. Exiting thread"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_UNKNOWN		"013 Unknown errno - setting server listen socket to listening. Exiting thread, errno "

#define FC_ERR_TCP_SERV_SOCK_ACCEPT_NSOCK		"014 NULL socket - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_NNETCONN	"015 NULL netconn - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_NO_SOCK		"016 No socket available - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_NOMEM		"017 No memory - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_BUF_ERR		"018 Buffer error - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_LSOCK_CLSD	"019 Listening socket closed - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_LSOCK_ERR	"020 Listening socket error - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_ASOCK_ERR	"021 Accepted conn error - waiting/accepting TCP connection"
#define FC_ERR_TCP_SERV_SOCK_ACCEPT_UNKNOWN		"022 Unknown errno - waiting/accepting TCP connection, errno "

#define FC_ERR_TCP_SERV_LISTEN_THREAD_CLOSE		"023 TCP server listener thread closing"

#define FC_ERR_TCP_SERV_RECV_WAIT_NSOCK			"024 NULL socket - TCP server waiting for data"
#define FC_ERR_TCP_SERV_RECV_WAIT_NOMEM			"025 Out of memory - TCP server waiting for data"
#define FC_ERR_TCP_SERV_RECV_WAIT_BUSY_SOCK		"026 Busy socket - TCP server waiting for data"
#define FC_ERR_TCP_SERV_RECV_WAIT_UNKNOWN		"027 Unknown errno - TCP server waiting for data, errno "

#define FC_ERR_TCP_SERV_RECV_READ_NSOCK			"028 NULL socket - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_TIMEOUT		"029 Read timeout - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_CONN_RST		"030 Reset by peer - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_SOCK_NCONN	"031 Not connected - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_NOMEM			"032 No memory - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_NOBUF			"033 No buffer space - TCP server reading from conn, fd "
#define FC_ERR_TCP_SERV_RECV_READ_UNKNOWN		"034 Unknown errno - TCP server reading from conn, fd/errno "

#define FC_ERR_TCP_SERV_RECV_STORE_NOMEM		"035 Out of memory - TCP server storing packet, fd "
#define FC_ERR_TCP_SERV_RECV_THREAD_CLOSE		"036 TCP server reader thread closing"

#define FC_ERR_TCP_SERV_WRITE_SEND_NSOCK		"037 NULL socket - TCP server sending packet, fd "
#define FC_ERR_TCP_SERV_WRITE_SEND_SOCK_BUSY	"038 Netconn busy - TCP server sending packet, fd "
#define FC_ERR_TCP_SERV_WRITE_SEND_NPCB			"039 NULL PCB - TCP server sending packet, fd "
#define FC_ERR_TCP_SERV_WRITE_SEND_NNETCONN		"040 NULL netconn - TCP server sending packet, fd "
#define FC_ERR_TCP_SERV_WRITE_SEND_MEM_ERR		"041 Internal memory error - TCP server sending packet, fd "
#define FC_ERR_TCP_SERV_WRITE_SEND_UNKNOWN		"042 Unknown errno - TCP server sending packet, fd/errno "

#define FC_ERR_TCP_SERV_WRITE_THREAD_CLOSE		"043 TCP server writer thread closing"

#define FC_ERR_TCP_SERV_FD_SCAN_NSOCK			"044 NULL socket - TCP server fd scanning, fd "
#define FC_ERR_TCP_SERV_FD_SCAN_UNKNOWN			"045 Unknown errno - TCP server fd scanning, fd/errno "

#define FC_ERR_EEPROM_INIT						"046 EEPROM init failed, loading defaults"
#define FC_ERR_EEPROM_LOAD_COMM_ERR				"047 EEPROM config loading failed, loading defaults"
#define FC_ERR_EEPROM_LOAD_TC_ERR				"048 EEPROM config invalid TC gains, loading default gains"
#define FC_ERR_EEPROM_LOAD_VLV_ERR				"049 EEPROM config invalid valve config, loading default settings"

#define FC_ERR_CREAT_TCP_MEM_ERR				"050 Memory error creating TCP server"
#define FC_ERR_INIT_TCP_THREAD_ERR				"051 TCP server thread not started during code init"

// General status messages
#define STAT_EXAMPLE							"000 This is a general status message"
#define STAT_NETWORK_LOG_ONLINE					"701 UDP logging online, IP "
#define STAT_STARTUP_DONE						"702 Program initialization complete"

// Flight Computer status messages
#define FC_STAT_EXAMPLE							"000 This is a status message unique to the FC (e.g. radio thread started)"
#define FC_STAT_TCP_SERV_NEW_CONN				"501 TCP server accepting new connection, fd/addr "
#define FC_STAT_TCP_CONN_CLOSED					"502 TCP connection closed gracefully, fd "
#define FC_STAT_EEPROM_LOADED					"503 EEPROM config loaded successfully"
#define FC_STAT_EEPROM_DEFAULT_LOADED			"504 EEPROM defaults loaded, overridden by macro"
#define FC_STAT_TCP_SERV_RUNNING				"505 TCP server initialized and running"

// Flight Computer error message types - used for throttling
#define FC_ERR_TYPE_TCP_SERV_LISTEN				0
#define FC_ERR_TYPE_TCP_SERV_RECV_SELECT		1
#define FC_ERR_TYPE_TCP_SERV_RECV_READ			2
#define FC_ERR_TYPE_TCP_SERV_WRITE				3
#define FC_ERR_TYPE_TCP_FD_SCAN					4

#endif /* INC_LOG_ERRORS_H_ */
