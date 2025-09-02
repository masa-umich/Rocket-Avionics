/*
 * tcpreader.c
 *
 *  Created on: Apr 26, 2023
 *      Author: evanm
 */

#include "tcpreader.h"

static struct netconn *conn, *newconn;
static struct netbuf *buf;
static unsigned short port;
static ip_addr_t *addr;
char msg[MAX_MSG_LEN];
char smsg[MAX_MSG_LEN];

/*-----------------------------------------------------------------------------------*/
static void tcp_reader_thread(void *arg) {
	err_t err, accept_err, recv_err;

	LWIP_UNUSED_ARG(arg);

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn != NULL) {
		/* Bind connection to ephemeral port number 50000. */
		err = netconn_bind(conn, IP4_ADDR_ANY, 50000);

		if (err == ERR_OK) {
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1) {
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);

				/* Process the new connection. */
				/* receive the data from the client */
				while (netconn_recv(newconn, &buf) == ERR_OK)
				{
					/* Extrct the address and port in case they are required */
					addr = netbuf_fromaddr(buf);  // get the address of the client
					port = netbuf_fromport(buf);  // get the Port of the client
						do {
							strncpy (msg, buf->p->payload, buf->p->len);   // get the message from the client
							u16_t ret_len = 0;

							parse_t rc = devparse(msg, buf->p->len, smsg, &ret_len, newconn);

							if (ret_len > 0) {
								netconn_write(newconn, (void*)smsg, (size_t)ret_len, NETCONN_COPY);
							}

							memset (msg, '\0', 100);  // clear the buffer
						} while (netbuf_next(buf) >= 0);

						netbuf_delete(buf);
					}

				/* Close connection and discard connection identifier. */
				netconn_close(newconn);
				netconn_delete(newconn);
				}
			}
		else {
			netconn_delete(newconn);
			printf("cannot bind TCP netconn");
		}
	} else {
		printf("cannot create TCP netconn");
	}
}

/*-----------------------------------------------------------------------------------*/

void tcpreader_init(void) {
	sys_thread_new("tcp_reader_thread", tcp_reader_thread, NULL,
			DEFAULT_THREAD_STACKSIZE, TCP_READER_THREAD_PRIO);
	sys_thread_new("tcp_reader_thread", tcp_reader_thread, NULL,
				DEFAULT_THREAD_STACKSIZE, TCP_READER_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/
#ifndef DEVPARSE
#define DEVPARSE
parse_t devparse(u8_t *data, u16_t len, u8_t *response, u16_t *ret_len, struct netconn* newconn) {
	/*
	 *  Data stores len number of bytes
	 *
	 *  Packet structure:
	 *  	6 bits: opcode
	 */

	char* str;

	if (len > 0) {

		u8_t opcode = (*data );//>> 2); // get opcode from first 6 bits

		switch (opcode) {
		case 'A':
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Toggle LED1

			str = "Toggled LED1\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			break;
		case 'I':
			/*
			 * Set IAP Flag
			 */
			HAL_PWR_EnableBkUpAccess();
			// Clears IAP Flag in RTC Backup data Register 1
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xDEE2);
			HAL_PWR_DisableBkUpAccess();

			str = "IAP Flag Set\r\nResetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			netconn_write(newconn, (void*)response, (size_t)*ret_len, NETCONN_COPY);

			NVIC_SystemReset();
			break;

		case 'R':
			str = "Resetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			netconn_write(newconn, (void*)response, (size_t)*ret_len, NETCONN_COPY);

			NVIC_SystemReset();
			break;

		case 'G':
			*ret_len = strlen(GIT_INFO);
			strcpy(response, GIT_INFO);

			break;

		default:
			str = "Unknown Command\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			break;
		}

		return PARSE_OK;
	} else {
		return PARSE_ERR;
	}

}
#endif
