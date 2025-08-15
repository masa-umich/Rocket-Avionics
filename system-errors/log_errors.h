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
#define ERR_IMU_READ							"209 IMU read failed, IMU "
#define ERR_BAR_READ							"210 BAR read failed, BAR "
#define ERR_ADS_READ							"211 ADS read failed"
#define ERR_TELEM_NOT_UPDATED					"212 Telemetry not updated"
#define ERR_TELEM_MEM_ERR						"213 Memory error sending telemetry"
#define ERR_TELEM_OVERTIME						"214 Telemetry loop ran overtime"
#define ERR_SAVE_INCOMING_TELEM					"215 Could not save incoming telemetry"
#define ERR_PROCESS_VLV_CMD_BADID				"216 Invalid valve id - valve command"
#define ERR_PROCESS_VLV_STATE_BADID				"217 Invalid valve id - valve state"
#define ERR_UNKNOWN_LMP_PACKET					"218 Unknown LMP message type"
#define ERR_TFTP_EERPOM_TOO_SHORT				"219 Written EEPROM data too short"
#define ERR_TFTP_EEPROM_READ_ERR				"220 EEPROM read error"
#define ERR_TFTP_EEPROM_WRITE_ERR				"221 EEPROM write error"
#define ERR_TFTP_EEPROM_BAD_CRC					"222 Invalid CRC during EEPROM write"
#define ERR_UDP_REINIT							"223 Error trying to reinit UDP logging"
#define ERR_RTC_NOT_SET							"224 Error trying to set the RTC"


// Flight Computer error messages
#define FC_ERR_TCP_SERV_SOCK_CREAT_NOBUF		"001 No buffer space - creating server listen socket"
#define FC_ERR_TCP_SERV_SOCK_CREAT_NOSOCK		"002 No available sockets - creating server listen socket"
#define FC_ERR_TCP_SERV_SOCK_CREAT_UNKNOWN		"003 Unknown errno - creating server listen socket, errno "

#define FC_ERR_TCP_SERV_SOCK_BIND_NSOCK			"004 NULL socket - binding server listen socket"
#define FC_ERR_TCP_SERV_SOCK_BIND_BADPCB		"005 Bad PCB - binding server listen socket"
#define FC_ERR_TCP_SERV_SOCK_BIND_USEDADDR		"006 Address already used - binding server listen socket"
#define FC_ERR_TCP_SERV_SOCK_BIND_UNKNOWN		"007 Unknown errno - binding server listen socket, errno "

#define FC_ERR_TCP_SERV_SOCK_LISTEN_NSOCK		"008 NULL socket - setting server listen socket to listening"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NNETCONN	"009 NULL netconn - setting server listen socket to listening"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NPCB		"010 NULL PCB - setting server listen socket to listening"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_BADPCB		"011 Bad PCB - setting server listen socket to listening"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_NOMEM		"012 No memory - setting server listen socket to listening"
#define FC_ERR_TCP_SERV_SOCK_LISTEN_UNKNOWN		"013 Unknown errno - setting server listen socket to listening, errno "

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

#define FC_ERR_SING_ADC_INIT					"052 ADC init failed"
#define FC_ERR_SING_ADC_READ					"053 ADC read failed"

#define FC_ERR_REINIT_TCP_THREAD_ERR			"054 TCP server thread not started during reinit"

#define FC_ERR_TCP_SERV_RECV_ERROR_RESET		"055 Reset by peer - TCP server socket flagged as error, fd "
#define FC_ERR_TCP_SERV_RECV_ERROR_TIMEO		"056 Unresponsive peer - TCP server socket flagged as error, fd "
#define FC_ERR_TCP_SERV_RECV_ERROR_CLOSED		"057 Connection closed - TCP server socket flagged as error, fd "
#define FC_ERR_TCP_SERV_RECV_ERROR_NOTCONN		"058 Socket not connected - TCP server socket flagged as error, fd "
#define FC_ERR_TCP_SERV_RECV_ERROR_UNKNOWN		"059 Unknown errno - TCP server socket flagged as error, fd/errno "

// Bay Board error messages
#define BB_ERR_TCP_CLIENT_SOCK_CREAT_NOBUF		"001 No buffer space - creating client socket"
#define BB_ERR_TCP_CLIENT_SOCK_CREAT_NOSOCK		"002 No available sockets - creating client socket"
#define BB_ERR_TCP_CLIENT_SOCK_CREAT_UNKNOWN	"003 Unknown errno - creating client socket, errno "

#define BB_ERR_TCP_CLIENT_CONNECT_NSOCK			"004 NULL socket - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_ERR			"005 Error connecting - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_TIMEO			"006 Connection timed out - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_NRTE			"007 No route - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_ABORT			"008 Internally aborted - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_NBUF			"009 Buffer error - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_NMEM			"010 Memory error - client connecting"
#define BB_ERR_TCP_CLIENT_CONNECT_UNKNOWN		"011 Unknown errno - client connecting, errno "

#define BB_ERR_TCP_CLIENT_WAIT_NSOCK			"012 NULL socket - client waiting for packet"
#define BB_ERR_TCP_CLIENT_WAIT_NOMEM			"013 Memory error - client waiting for packet"
#define BB_ERR_TCP_CLIENT_WAIT_BUSY				"014 Socket busy - client waiting for packet"
#define BB_ERR_TCP_CLIENT_WAIT_UNKNOWN			"015 Unknown errno - client waiting for packet, errno "

#define BB_ERR_TCP_CLIENT_READ_NSOCK			"016 NULL socket - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_TIMEOUT			"017 Read timeout - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_CONN_RST			"018 Reset by peer - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_SOCK_NCONN		"019 Not connected - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_NOMEM			"020 No memory - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_NOBUF			"021 No buffer space - client receiving packet"
#define BB_ERR_TCP_CLIENT_READ_UNKNOWN			"022 Unknown errno - client receiving packet, errno "

#define BB_ERR_TCP_CLIENT_ERROR_RESET			"023 Reset by peer - client socket flagged as error"
#define BB_ERR_TCP_CLIENT_ERROR_TIMEO			"024 Unresponsive peer - client socket flagged as error"
#define BB_ERR_TCP_CLIENT_ERROR_CLOSED			"025 Connection closed - client socket flagged as error"
#define BB_ERR_TCP_CLIENT_ERROR_NOTCONN			"026 Socket not connected - client socket flagged as error"
#define BB_ERR_TCP_CLIENT_ERROR_UNKNOWN			"027 Unknown errno - client socket flagged as error, errno "

#define BB_ERR_TCP_CLIENT_SAVE_BUFFULL			"028 rxbuffer full - client saving packet"
#define BB_ERR_TCP_CLIENT_SAVE_NOMEM			"029 No memory - client saving packet"

// General status messages
#define STAT_EXAMPLE							"000 This is a general status message"
#define STAT_NETWORK_LOG_ONLINE					"701 UDP logging online, IP "
#define STAT_STARTUP_DONE						"702 Program initialization complete"
#define STAT_TELEM_TASK_STARTED					"703 Telemetry task started"
#define STAT_PACKET_TASK_STARTED				"704 Packet processing task started"
#define STAT_EEPROM_CONFIG_CHANGED				"705 New EEPROM config saved"
#define STAT_LINK_UP							"706 Ethernet link back online"
#define STAT_LINK_DOWN							"707 Ethernet link down"
#define STAT_AVAILABLE_FLASH					"708 Used flash space: "
#define STAT_CLEAR_FLASH						"709 Flash cleared"

// Flight Computer status messages
#define FC_STAT_EXAMPLE							"000 This is a status message unique to the FC (e.g. radio thread started)"
#define FC_STAT_TCP_SERV_NEW_CONN				"501 TCP server accepting new connection, fd/addr "
#define FC_STAT_TCP_CONN_CLOSED					"502 TCP connection closed gracefully, fd "
#define FC_STAT_EEPROM_LOADED					"503 EEPROM config loaded successfully"
#define FC_STAT_EEPROM_DEFAULT_LOADED			"504 EEPROM defaults loaded, overridden by macro"
#define FC_STAT_TCP_SERV_RUNNING				"505 TCP server initialized and running"
#define FC_STAT_TCP_SERV_REINIT					"506 TCP server reinitialized after link down"

// Bay Board status messages
#define BB_STAT_TCP_CLIENT_CONNECTED			"501 TCP client connected"
#define BB_STAT_TCP_CLIENT_CLOSED				"502 TCP client disconnected"

// Flight Computer error message types - used for throttling
#define FC_ERR_TYPE_TCP_SERV_LISTEN				0
#define FC_ERR_TYPE_TCP_SERV_RECV_SELECT		1
#define FC_ERR_TYPE_TCP_SERV_RECV_READ			2
#define FC_ERR_TYPE_TCP_SERV_WRITE				3
#define FC_ERR_TYPE_TCP_FD_SCAN					4
#define FC_ERR_TYPE_TELEM_NUPDATED				5
#define FC_ERR_TYPE_TELEM_MEM_ERR				6
#define FC_ERR_TYPE_TELEM_OVERTIME				7
#define FC_ERR_TYPE_INCOMING_TELEM				8
#define FC_ERR_TYPE_BAD_VLVID					9
#define FC_ERR_TYPE_UNKNOWN_LMP					10
#define FC_ERR_TYPE_TFTP_EEPROM_READ			11
#define FC_ERR_TYPE_TFTP_EEPROM_WRITE			12

// Flight Computer peripheral error message types
#define FC_ERR_PERI_TYPE_ADC					0
#define FC_ERR_PERI_TYPE_IMU1					1
#define FC_ERR_PERI_TYPE_IMU2					2
#define FC_ERR_PERI_TYPE_BAR1					3
#define FC_ERR_PERI_TYPE_BAR2					4
#define FC_ERR_PERI_TYPE_ADS					5

// Bay Board error message types
#define BB_ERR_TYPE_TCP_CLIENT_CONNECT			0
#define BB_ERR_TYPE_TCP_CLIENT_RECV				1

#endif /* INC_LOG_ERRORS_H_ */
