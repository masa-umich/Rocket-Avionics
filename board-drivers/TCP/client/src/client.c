/*
 * client.c
 *
 *  Created on: Aug 13, 2025
 *      Author: felix
 */

#include "client.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "lwip/sockets.h"
#include "log_errors.h"

QueueHandle_t txbuffer = NULL;
QueueHandle_t rxbuffer = NULL;

int conn_fd = -1;
SemaphoreHandle_t netconn_use = NULL;
SemaphoreHandle_t netconn_hold = NULL;
SemaphoreHandle_t netconn_free = NULL;

uint8_t running = 0;
SemaphoreHandle_t runningMutex = NULL;

uint8_t connected = 0;
SemaphoreHandle_t connectedMutex = NULL;

SemaphoreHandle_t client_stop = NULL;
SemaphoreHandle_t client_stopped = NULL;

struct sockaddr_in fc_addr;

int client_init(ip4_addr_t *fcaddr, int start) {
	fc_addr.sin_family = AF_INET;
	fc_addr.sin_port = htons(TCP_PORT);
	inet_addr_from_ip4addr(&fc_addr.sin_addr, fcaddr);

	txbuffer = xQueueCreate(100, sizeof(RawMessage));
	rxbuffer = xQueueCreate(100, sizeof(RawMessage));

	netconn_use = xSemaphoreCreateBinary();
	netconn_hold = xSemaphoreCreateBinary();
	netconn_free = xSemaphoreCreateBinary();
	runningMutex = xSemaphoreCreateMutex();
	connectedMutex = xSemaphoreCreateMutex();
	client_stop = xSemaphoreCreateBinary();
	client_stopped = xSemaphoreCreateBinary();

	if(txbuffer == NULL || rxbuffer == NULL || netconn_use == NULL || netconn_hold == NULL || netconn_free == NULL) {
		return 1;
	}

	xSemaphoreTake(runningMutex, portMAX_DELAY);
	running = start;
	xSemaphoreGive(runningMutex);

    osThreadId_t receiveTaskHandle, sendTaskHandle;

    const osThreadAttr_t receiveTask_attributes = {
      .name = "receiveTask",
      .stack_size = 512 * 6,
      .priority = (osPriority_t) osPriorityNormal,
    };

    const osThreadAttr_t sendTask_attributes = {
      .name = "sendTask",
      .stack_size = 512 * 6,
      .priority = (osPriority_t) osPriorityNormal,
    };

    receiveTaskHandle = osThreadNew(client_receive_thread, NULL, &receiveTask_attributes);
    sendTaskHandle = osThreadNew(client_send_thread, NULL, &sendTaskHandle);

    if(receiveTaskHandle == NULL || sendTaskHandle == NULL) {
    	return 2;
    }

    return 0;
}

void client_receive_thread(void *arg) {
	for(;;) {
		uint8_t stat;
		do {
			stat = 0;
			if(xSemaphoreTake(runningMutex, 2) == pdPASS) {
				stat = running;
				xSemaphoreGive(runningMutex);
			}
			if(!stat) {
				osDelay(100);
			}
		} while(!stat);
		xSemaphoreTake(client_stopped, 0); // in case the loop was because of a closed connection and not a call to client_stop()
		conn_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if(conn_fd < 0) {
        	switch(errno) {
        	    case ENOBUFS: {
        	        // no memory - couldn't create netconn
        	    	log_message(BB_ERR_TCP_CLIENT_SOCK_CREAT_NOBUF, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        	        break;
        	    }
        	    case ENFILE: {
        	        // couldn't create socket, none available
        	    	log_message(BB_ERR_TCP_CLIENT_SOCK_CREAT_NOSOCK, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when creating socket, use errno in message
        	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_SOCK_CREAT_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_SOCK_CREAT_UNKNOWN "%d", errno);
        	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        	        break;
        	    }
        	}
			osDelay(RETRY_DELAY_MS);
			continue;
		}

        int idle = TCP_KEEP_ALIVE_IDLE;
        int intvl = TCP_KEEP_ALIVE_INTERVAL;
        int optval = 1;
        int probecnt = TCP_KEEP_ALIVE_COUNT;
        struct timeval conntimeout;
        conntimeout.tv_sec = 0;
        conntimeout.tv_usec = 100000;
        setsockopt(conn_fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
        setsockopt(conn_fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(conn_fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(conn_fd, IPPROTO_TCP, TCP_KEEPCNT, &probecnt, sizeof(probecnt));
        setsockopt(conn_fd, SOL_SOCKET, SO_RCVTIMEO, &conntimeout, sizeof(conntimeout));
        setsockopt(conn_fd, SOL_SOCKET, SO_SNDTIMEO, &conntimeout, sizeof(conntimeout));

        if(connect(conn_fd, (struct sockaddr *)&fc_addr, sizeof(fc_addr)) < 0) {
        	switch(errno) {
        		case EBADF: {
        			// bad socket
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_NSOCK, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case ENOTCONN: {
        			// error connecting
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_ERR, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case EWOULDBLOCK: {
        			// connection timed out
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_TIMEO, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case EHOSTUNREACH: {
        			// no route to host
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_NRTE, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case ECONNABORTED: {
        			// connection internally aborted
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_ABORT, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case ENOBUFS: {
        			// buffer memory error
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_NBUF, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		case ENOMEM: {
        			// memory error
        	    	log_message(BB_ERR_TCP_CLIENT_CONNECT_NMEM, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}
        		default: {
        			// unknown error
        	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_CONNECT_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_CONNECT_UNKNOWN "%d", errno);
        	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_CONNECT);
        			break;
        		}

        	}
        	close(conn_fd);
			osDelay(RETRY_DELAY_MS);
			continue;
        }
        // connected
		log_message(BB_STAT_TCP_CLIENT_CONNECTED, -1);

		RawMessage msg = {0};
		while(xQueueReceive(txbuffer, (void *)&msg, 0) == pdPASS) {
			free(msg.bufferptr);
		}
		while(xQueueReceive(rxbuffer, (void *)&msg, 0) == pdPASS) {
			free(msg.bufferptr);
		}

		xSemaphoreGive(netconn_use);

		xSemaphoreTake(connectedMutex, portMAX_DELAY);
		connected = 1;
		xSemaphoreGive(connectedMutex);

		// receive
	    fd_set rfdset;
	    fd_set efdset;
	    struct timeval timeout;

	    FD_ZERO(&rfdset);
	    FD_ZERO(&efdset);
	    FD_SET(conn_fd, &rfdset);
	    FD_SET(conn_fd, &efdset);

	    timeout.tv_sec = 0;
	    timeout.tv_usec = 100000;

	    for(;;) {
			if(xSemaphoreTake(client_stop, 0) == pdPASS) {
				break;
			}

	    	int ready = select(conn_fd + 1, &rfdset, NULL, &efdset, &timeout);

	    	if(ready > 0) {
	            if(FD_ISSET(conn_fd, &rfdset)) {
                	uint8_t tempbuffer[MAX_MSG_LEN];

				    int packet_len = recv(conn_fd, tempbuffer, MAX_MSG_LEN, 0);
				    if(packet_len == 0) {
				    	break;
				    }
				    else if(packet_len < 0) {
				    	switch(errno) {
				    	    case EBADF: {
				    	        // invalid socket, closing and cleaning up fd
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_NSOCK, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    case EWOULDBLOCK: {
				    	        // receive timeout, not very critical, but also not common, just try again
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_TIMEOUT, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    case ECONNRESET: {
				    	        // connection reset by peer, closing and cleaning up fd
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_CONN_RST, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    case ENOTCONN: {
				    	        // socket is not connected, closing and cleaning up fd
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_SOCK_NCONN, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    case ENOMEM: {
				    	        // out of memory when receiving data
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_NOMEM, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    case ENOBUFS: {
				    	        // out of buffer space
			        	    	log_message(BB_ERR_TCP_CLIENT_READ_NOBUF, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	    default: {
				    	    	// unknown error when receiving data, use errno in message
			        	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_READ_UNKNOWN) + 3];
			        	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_READ_UNKNOWN "%d", errno);
			        	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_RECV);
				    	        break;
				    	    }
				    	}
				    	if(errno == EBADF || errno == ECONNRESET || errno == ENOTCONN) {
				    		break;
				    	}
				    }
				    else {
				    	RawMessage msg = {0};
					    uint8_t *buffer = malloc(packet_len);
					    if(buffer) {
						    memcpy(buffer, tempbuffer, packet_len);
						    msg.bufferptr = buffer;
						    msg.packet_len = packet_len;

		                    if(xQueueSend(rxbuffer, (void *)&msg, portMAX_DELAY) != pdPASS) {
		                    	// rxbuffer full
		            	    	log_message(BB_ERR_TCP_CLIENT_SAVE_BUFFULL, BB_ERR_TYPE_TCP_CLIENT_RECV);
		                    	free(buffer);
		                    }
					    }
					    else {
					    	// no memory creating receiving buffer
		        	    	log_message(BB_ERR_TCP_CLIENT_SAVE_NOMEM, BB_ERR_TYPE_TCP_CLIENT_RECV);
					    }
				    }
	            }
	            else if(FD_ISSET(conn_fd, &efdset)) {
	            	int err = 0;
                    socklen_t len = sizeof(err);
                    if(getsockopt(connection_fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err != 0) {
                        switch(err) {
                        	case ECONNRESET: {
                        		// reset by peer, closing
                    	    	log_message(BB_ERR_TCP_CLIENT_ERROR_RESET, BB_ERR_TYPE_TCP_CLIENT_RECV);
                        		break;
                        	}
                        	case ETIMEDOUT: {
                        		// unresponsive peer
                    	    	log_message(BB_ERR_TCP_CLIENT_ERROR_TIMEO, BB_ERR_TYPE_TCP_CLIENT_RECV);
                        		break;
                        	}
                        	case EPIPE: {
                        		// connection closed
                    	    	log_message(BB_ERR_TCP_CLIENT_ERROR_CLOSED, BB_ERR_TYPE_TCP_CLIENT_RECV);
                        		break;
                        	}
                        	case ENOTCONN: {
                        		// socket not connected
                    	    	log_message(BB_ERR_TCP_CLIENT_ERROR_NOTCONN, BB_ERR_TYPE_TCP_CLIENT_RECV);
                        		break;
                        	}
                        	default: {
                        		// unknown
                    	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_ERROR_UNKNOWN) + 3];
                    	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_ERROR_UNKNOWN "%d", err);
                    	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_RECV);
                        		break;
                        	}
                        }
                        if(err == ECONNRESET || err == ETIMEDOUT || err == EPIPE || err == ENOTCONN) {
                        	break; // close and try again
                        }
                    }
	            }
	    	}
	    	else if(ready < 0) {
	        	switch(errno) {
	        	    case EBADF: {
	        	    	// bad socket
	        	    	log_message(BB_ERR_TCP_CLIENT_WAIT_NSOCK, BB_ERR_TYPE_TCP_CLIENT_RECV);
	        	        break;
	        	    }
	        	    case ENOMEM: {
	        	        // out of memory creating semaphores to signal ready sockets
	        	    	log_message(BB_ERR_TCP_CLIENT_WAIT_NOMEM, BB_ERR_TYPE_TCP_CLIENT_RECV);
	        	        break;
	        	    }
	        	    case EBUSY: {
	        	        // too many threads waiting on one or more of the sockets
	        	    	log_message(BB_ERR_TCP_CLIENT_WAIT_BUSY, BB_ERR_TYPE_TCP_CLIENT_RECV);
	        	        break;
	        	    }
	        	    default: {
	        	    	// unknown error when waiting for socket events
	        	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_WAIT_UNKNOWN) + 3];
	        	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_WAIT_UNKNOWN "%d", errno);
	        	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_RECV);
	        	        break;
	        	    }
	        	}
	        	if(errno == EBADF) {
	        		break; // close
	        	}
	    	}
	    }

		// when the connection is going to be closed
	    // closed
    	log_message(BB_STAT_TCP_CLIENT_CLOSED, -1);
		xSemaphoreGive(netconn_hold);
		xSemaphoreTake(netconn_free, portMAX_DELAY);
		// close connection
		xSemaphoreTake(connectedMutex, portMAX_DELAY);
		connected = 0;
		xSemaphoreGive(connectedMutex);

		close(conn_fd);
		conn_fd = -1;
		xSemaphoreGive(client_stopped);
	}
}

void client_send_thread(void *arg) {
	for(;;) {
		xSemaphoreTake(netconn_use, portMAX_DELAY);
		for(;;) {
			// send loop
			if(xSemaphoreTake(netconn_hold, 0) == pdPASS) {
				break;
			}

			RawMessage msg = {0};
			if(xQueueReceive(txbuffer, (void *)&msg, 100) == pdPASS) {
				if(send(conn_fd, msg.bufferptr, msg.packet_len, 0) == -1) {
			    	switch(errno) {
			    	    case EBADF: {
			    	        // invalid socket, letting receive task clean up the connection
			    	    	log_message(BB_ERR_TCP_CLIENT_SEND_NSOCK, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	    case EINPROGRESS: {
			    	        // internal netconn is connecting, closing, or writing in a different place
			    	    	log_message(BB_ERR_TCP_CLIENT_SEND_SOCK_BUSY, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	    case ENOTCONN: {
			    	        // invalid netconn pcb
			    	    	log_message(BB_ERR_TCP_CLIENT_SEND_NPCB, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	    case EIO: {
			    	        // NULL netconn, this should be caught in EBADF but in a rare case this might happen
			    	    	log_message(BB_ERR_TCP_CLIENT_SEND_NNETCONN, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	    case EINVAL: {
			    	        // internal overflow or NULL pointer
			    	    	log_message(BB_ERR_TCP_CLIENT_SEND_MEM_ERR, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	    default: {
			    	    	// unknown error when sending data, use errno in message
		        	    	char logmsg[sizeof(BB_ERR_TCP_CLIENT_SEND_UNKNOWN) + 3];
		        	    	snprintf(logmsg, sizeof(logmsg), BB_ERR_TCP_CLIENT_SEND_UNKNOWN "%d", errno);
		        	    	log_message(logmsg, BB_ERR_TYPE_TCP_CLIENT_SEND);
			    	        break;
			    	    }
			    	}
				}
				free(msg.bufferptr);
			}
		}
		// stopped by receive thread
    	log_message(BB_ERR_TCP_CLIENT_SEND_STOPPED, -1);
		xSemaphoreGive(netconn_free);
	}
}

int client_send(RawMessage *msg, TickType_t block) {
	if(is_client_connected() > 0) {
		if(xQueueSend(txbuffer, (void *)msg, block) != pdPASS) {
			// Timed out
			return -2;
		}

		return 0; // success
	}
	else {
		return -1;
	}
}

int client_receive(RawMessage *msg, TickType_t block) {
	if(is_client_connected() > 0) {
		if(xQueueReceive(rxbuffer, (void *)msg, block) != pdPASS) {
			// No message available
			return -2;
		}

		// Return the length of the message
		return msg->packet_len;
	}
	else {
		return -1;
	}
}

int is_client_connected() {
	if(xSemaphoreTake(connectedMutex, 2) == pdPASS) {
		uint8_t stat = connected;
		xSemaphoreGive(connectedMutex);
		return stat;
	}
	else {
		return -1;
	}
}

int client_stop() {
	xSemaphoreTake(runningMutex, portMAX_DELAY);
	if(!running) {
		xSemaphoreGive(runningMutex);
		return -1;
	}
	else {
		running = 0;
		xSemaphoreGive(runningMutex);
	}

	xSemaphoreGive(client_stop);
	xSemaphoreTake(client_stopped, portMAX_DELAY);

	return 0;
}

int client_reinit() {
	xSemaphoreTake(runningMutex, portMAX_DELAY);
	if(running) {
		xSemaphoreGive(runningMutex);
		return -1;
	}
	else {
		running = 1;
		xSemaphoreGive(runningMutex);
	}
	return 0;
}
