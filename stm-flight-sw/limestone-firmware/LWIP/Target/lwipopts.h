/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Target/lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "main.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.1.2 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS enabled (Since FREERTOS is set) -----*/
#define WITH_RTOS 1
/* Temporary workaround to avoid conflict on errno defined in STM32CubeIDE and lwip sys_arch.c errno */
#undef LWIP_PROVIDE_ERRNO
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Default value in ETH configuration GUI in CubeMx: 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536
/*----- Default Value for MEMP_NUM_TCP_PCB: 5 ---*/
#define MEMP_NUM_TCP_PCB 9
/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
#define MEM_ALIGNMENT 4
/*----- Default Value for MEM_SIZE: 1600 ---*/
#define MEM_SIZE 32232
/*----- Default Value for H7 devices: 0x30004000 -----*/
#define LWIP_RAM_HEAP_POINTER 0x30000200
/*----- Default Value for MEMP_NUM_TCP_SEG: 16 ---*/
#define MEMP_NUM_TCP_SEG 32
/*----- Default Value for MEMP_NUM_SYS_TIMEOUT: 3 ---*/
#define MEMP_NUM_SYS_TIMEOUT 16
/*----- Default Value for MEMP_NUM_NETBUF: 2 ---*/
#define MEMP_NUM_NETBUF 10
/*----- Default Value for MEMP_NUM_NETCONN: 4 ---*/
#define MEMP_NUM_NETCONN 10
/*----- Default Value for MEMP_NUM_TCPIP_MSG_INPKT: 8 ---*/
#define MEMP_NUM_TCPIP_MSG_INPKT 16
/*----- Default Value for MEMP_NUM_NETDB: 1 ---*/
#define MEMP_NUM_NETDB 4
/*----- Default Value for MEMP_NUM_API_MSG: 8 ---*/
#define MEMP_NUM_API_MSG 10
/*----- Value supported for H7 devices: 1 -----*/
#define LWIP_SUPPORT_CUSTOM_PBUF 1
/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Default Value for TCP_MSS: 536 ---*/
#define TCP_MSS 1436
/*----- Default Value for TCP_SND_BUF: 2872 ---*/
#define TCP_SND_BUF 5744
/*----- Default Value for TCP_SND_QUEUELEN: 17 ---*/
#define TCP_SND_QUEUELEN 16
/*----- Value in opt.h for LWIP_NETIF_LINK_CALLBACK: 0 -----*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for TCPIP_THREAD_STACKSIZE: 0 -----*/
#define TCPIP_THREAD_STACKSIZE 1536
/*----- Value in opt.h for TCPIP_THREAD_PRIO: 1 -----*/
#define TCPIP_THREAD_PRIO 24
/*----- Value in opt.h for TCPIP_MBOX_SIZE: 0 -----*/
#define TCPIP_MBOX_SIZE 16
/*----- Value in opt.h for SLIPIF_THREAD_STACKSIZE: 0 -----*/
#define SLIPIF_THREAD_STACKSIZE 1024
/*----- Value in opt.h for SLIPIF_THREAD_PRIO: 1 -----*/
#define SLIPIF_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_THREAD_STACKSIZE: 0 -----*/
#define DEFAULT_THREAD_STACKSIZE 1024
/*----- Value in opt.h for DEFAULT_THREAD_PRIO: 1 -----*/
#define DEFAULT_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_UDP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_UDP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_TCP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_TCP_RECVMBOX_SIZE 32
/*----- Value in opt.h for DEFAULT_ACCEPTMBOX_SIZE: 0 -----*/
#define DEFAULT_ACCEPTMBOX_SIZE 16
/*----- Default Value for LWIP_TCP_KEEPALIVE: 0 ---*/
#define LWIP_TCP_KEEPALIVE 1
/*----- Default Value for LWIP_SO_SNDTIMEO: 0 ---*/
#define LWIP_SO_SNDTIMEO 1
/*----- Default Value for LWIP_SO_RCVTIMEO: 0 ---*/
#define LWIP_SO_RCVTIMEO 1
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Default Value for LWIP_SNTP: 0 ---*/
#define LWIP_SNTP 1
/*----- Default Value for LWIP_TFTP: 0 ---*/
#define LWIP_TFTP 1
/*----- Default Value for LWIP_STATS: 0 ---*/
#define LWIP_STATS 1
/*----- Value in opt.h for MIB2_STATS: 0 or SNMP_LWIP_MIB2 -----*/
#define MIB2_STATS 0
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*----- Default Value for ETHARP_DEBUG: LWIP_DBG_OFF ---*/
#define ETHARP_DEBUG LWIP_DBG_ON
/*----- Default Value for NETIF_DEBUG: LWIP_DBG_OFF ---*/
#define NETIF_DEBUG LWIP_DBG_ON
/*----- Default Value for PBUF_DEBUG: LWIP_DBG_OFF ---*/
#define PBUF_DEBUG LWIP_DBG_ON
/*----- Default Value for SYS_DEBUG: LWIP_DBG_OFF ---*/
#define SYS_DEBUG LWIP_DBG_ON
/*----- Default Value for TIMERS_DEBUG: LWIP_DBG_OFF ---*/
#define TIMERS_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_INPUT_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_INPUT_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_FR_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_FR_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_RTO_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_RTO_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_CWND_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_CWND_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_WND_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_WND_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_OUTPUT_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_OUTPUT_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_RST_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_RST_DEBUG LWIP_DBG_ON
/*----- Default Value for TCP_QLEN_DEBUG: LWIP_DBG_OFF ---*/
#define TCP_QLEN_DEBUG LWIP_DBG_ON
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define IP_SOF_BROADCAST      1
#define IP_SOF_BROADCAST_RECV 1

//#define TCP_KEEPIDLE 10000 // 10 seconds
//#define TCP_KEEPINTVL 5000 // 5 seconds
#define LWIP_DEBUG
#define SNTP_DEBUG    LWIP_DBG_ON
#define UDP_DEBUG     LWIP_DBG_ON
#define ICMP_DEBUG 	  LWIP_DBG_ON
#define LWIP_DBG_MIN_LEVEL LWIP_DBG_LEVEL_ALL
#define TCP_STATS 1
#define LINK_STATS 1
#define ETHARP_STATS 1
#define IPFRAG_STATS 1
#define UDP_STATS 1
#define TCP_STATS 1
#define MEM_STATS 1
#define MEMP_STATS 1
#define SYS_STATS 1
/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H__ */
