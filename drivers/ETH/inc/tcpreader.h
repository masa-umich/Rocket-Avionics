/*
 * tcpreader.h
 *
 *  Created on: Jan 14, 2024
 *      Author: evanm
 */

#ifndef INC_TCPREADER_H_
#define INC_TCPREADER_H_

#include "string.h"
#include "lwip/opt.h"
#include "gitcommit.h"

typedef enum {
	PARSE_OK, PARSE_ERR,
} parse_t;

#include "lwip/sys.h"
#include "lwip/api.h"

#define TCP_READER_THREAD_PRIO  osPriorityNormal

#define MAX_MSG_LEN 200 // 100 byte max response size

extern ip4_addr_t ipaddr;

extern RTC_HandleTypeDef hrtc;

static void tcp_reader_thread(void *arg);

void tcpreader_init(void);

parse_t devparse(u8_t *data, u16_t len, u8_t *response, u16_t *ret_len, struct netconn* newconn);



#endif /* INC_TCPREADER_H_ */
