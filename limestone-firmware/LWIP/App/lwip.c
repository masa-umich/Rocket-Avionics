/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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

/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include <string.h>

/* USER CODE BEGIN 0 */
#include "sntp.h"
#include "server.h"
#include "tftp_server.h"
#include "lwip/udp.h"
#include "log_errors.h"
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
static void ethernet_link_status_updated(struct netif *netif);
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
extern const struct tftp_context my_tftp_ctx;
extern ip4_addr_t fc_addr;
extern struct netconn *errormsgudp;
extern SemaphoreHandle_t errorudp_mutex;
extern void send_udp_online(ip4_addr_t * ip);
extern uint8_t log_message(const char *msgtext, int msgtype);
/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
/* USER CODE BEGIN OS_THREAD_ATTR_CMSIS_RTOS_V2 */
#define INTERFACE_THREAD_STACK_SIZE ( 1024 )
osThreadAttr_t attributes;
/* USER CODE END OS_THREAD_ATTR_CMSIS_RTOS_V2 */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 0;
  IP_ADDRESS[1] = 0;
  IP_ADDRESS[2] = 0;
  IP_ADDRESS[3] = 0;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 0;
  GATEWAY_ADDRESS[1] = 0;
  GATEWAY_ADDRESS[2] = 0;
  GATEWAY_ADDRESS[3] = 0;

/* USER CODE BEGIN IP_ADDRESSES */
  IP_ADDRESS[0] = ip4_addr1(&fc_addr);
  IP_ADDRESS[1] = ip4_addr2(&fc_addr);
  IP_ADDRESS[2] = ip4_addr3(&fc_addr);
  IP_ADDRESS[3] = ip4_addr4(&fc_addr);
/* USER CODE END IP_ADDRESSES */

  /* Initialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  /* We must always bring the network interface up connection or not... */
  netif_set_up(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);

  /* Create the Ethernet link handler thread */
/* USER CODE BEGIN H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */

  	// Putting this here to avoid calling sntp and tftp functions in different running threads
    // Also has the advantage of trying to start the UDP message logging early on

	// Setup NTP listener
	sntp_setoperatingmode(SNTP_OPMODE_LISTENONLY);

	if(netif_is_link_up(&gnetif)) {
		sntp_init();
		tftp_init(&my_tftp_ctx);
		if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
			errormsgudp = netconn_new(NETCONN_UDP);
			if(errormsgudp) {
				ip_set_option(errormsgudp->pcb.udp, SOF_BROADCAST);
				send_udp_online(&ipaddr);
			}
			else {
				// failed to create netconn
				log_message(ERR_UDP_INIT_NNETCONN, -1);
			}
			xSemaphoreGive(errorudp_mutex);
		}
		else {
			// failed to take mutex
			log_message(ERR_UDP_INIT_MUTEX, -1);
		}
	}
	else {
		// ethernet link down
		log_message(ERR_LINK_INIT_DOWN, -1);
	}

  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "EthLink";
  attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
  attributes.priority = osPriorityNormal;
  osThreadNew(ethernet_link_thread, &gnetif, &attributes);
/* USER CODE END H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
static void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif))
  {
/* USER CODE BEGIN 5 */
	  if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		  if(!errormsgudp) {
		  	  errormsgudp = netconn_new(NETCONN_UDP);
		  	  if(errormsgudp) {
		  		  ip_set_option(errormsgudp->pcb.udp, SOF_BROADCAST);
		  	  }
		  	  else {
				  log_message(ERR_UDP_REINIT, -1);
		  	  }
		  }
		  else {
			  log_message(ERR_UDP_REINIT, -1);
		  }
	  	  xSemaphoreGive(errorudp_mutex);
	  }
	  else {
		  log_message(ERR_UDP_REINIT, -1);
	  }
	  sntp_init();
      switch(server_init()) {
      	  case 0: {
      	  	  // TCP server started
      	  	  log_message(FC_STAT_TCP_SERV_REINIT, -1);
      	  	  break;
      	  }
      	  case -1: {
      	  	  break;
      	  }
      	  case -2: {
      	  	  // one of the threads failed to start
      	  	  log_message(FC_ERR_REINIT_TCP_THREAD_ERR, -1);
      	  	  break;
      	  }
      	  case -3: {
      	  	  break;
      	  }
      	  default: {
      	  	  break;
      	  }
      }
	  tftp_init(&my_tftp_ctx);
	  // link up
	  log_message(STAT_LINK_UP, -1);

/* USER CODE END 5 */
  }
  else /* netif is down */
  {
/* USER CODE BEGIN 6 */
	  if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		  netconn_close(errormsgudp);
		  netconn_delete(errormsgudp);
		  errormsgudp = NULL;
		  xSemaphoreGive(errorudp_mutex);
	  }
	  sntp_stop();
	  shutdown_server();
	  tftp_cleanup();
	  // link down
	  log_message(STAT_LINK_DOWN, -1);
/* USER CODE END 6 */
  }
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

