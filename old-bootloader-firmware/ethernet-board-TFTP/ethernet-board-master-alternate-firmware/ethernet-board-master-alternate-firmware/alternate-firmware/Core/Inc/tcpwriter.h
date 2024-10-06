/*
 * tcpwriter.h
 *
 *  Created on: Jan 14, 2024
 *      Author: evanm
 */

#ifndef INC_TCPWRITER_H_
#define INC_TCPWRITER_H_

#include "string.h"
#include "lwip/opt.h"
#include "gitcommit.h"

typedef enum {
	PARSE_OK, PARSE_ERR,
} parse_t;

#include "lwip/sys.h"
#include "lwip/api.h"

parse_t devparse(u8_t *data, u16_t len, u8_t *response, u16_t *ret_len, struct netconn* newconn);

#define TCP_WRITER_THREAD_PRIO  osPriorityNormal

#define MAX_RESPONSE_LEN 100 // 100 byte max response size

extern ip4_addr_t ipaddr;

extern RTC_HandleTypeDef hrtc;



#endif /* INC_TCPWRITER_H_ */
