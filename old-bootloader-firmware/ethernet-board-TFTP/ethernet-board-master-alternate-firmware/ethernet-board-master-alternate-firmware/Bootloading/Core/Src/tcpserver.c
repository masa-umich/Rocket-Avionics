/*
 * tcpserver.c
 *
 *  Created on: Apr 26, 2023
 *      Author: evanm
 */

#include "string.h"
#include "lwip/opt.h"

typedef enum {
	PARSE_OK, PARSE_ERR,
} parse_t;

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"

parse_t devparse(u8_t *data, u16_t len, u8_t *response, u16_t *ret_len, struct netconn* newconn);

#define TCP_RECV_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )

#define MAX_RESPONSE_LEN 100 // 100 byte max response size

extern ip4_addr_t ipaddr;

extern RTC_HandleTypeDef hrtc;

/*-----------------------------------------------------------------------------------*/
static void tcp_recv_cmd_thread(void *arg) {
	struct netconn *conn, *newconn;
	err_t err, accept_err;
	struct netbuf *buf;
	void *data;
	u16_t len;
	err_t recv_err;

	LWIP_UNUSED_ARG(arg);

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn != NULL) {
		/* Bind connection to ephemeral port number 50000. */
		err = netconn_bind(conn, &ipaddr, 50000);

		if (err == ERR_OK) {
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1) {
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);

				/* Process the new connection. */
				if (accept_err == ERR_OK) {
					recv_err = netconn_recv(newconn, &buf);
					while (recv_err == ERR_OK) {
						do {
							netbuf_data(buf, &data, &len);

							u8_t response[MAX_RESPONSE_LEN];
							u16_t ret_len = 0;
							parse_t rc = devparse(data, len, response, &ret_len, newconn);

							if (ret_len > 0) {
								netconn_write(newconn, (void*)response, (size_t)ret_len, NETCONN_COPY);
							}
						} while (netbuf_next(buf) >= 0);

						netbuf_delete(buf);
						recv_err = netconn_recv(newconn, &buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		} else {
			netconn_delete(newconn);
			printf("cannot bind TCP netconn");
		}
	} else {
		printf("cannot create TCP netconn");
	}
}
/*-----------------------------------------------------------------------------------*/

void tcpdev_recv_cmd(void) {
	sys_thread_new("tcp_recv_cmd_thread", tcp_recv_cmd_thread, NULL,
			DEFAULT_THREAD_STACKSIZE, TCP_RECV_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */

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

			char* str = "Toggled LED1\r\n";
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
