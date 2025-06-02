# TCP Server
This is the firmware intended to act as a TCP server on the main flight computer. It is the single point of communication for everything onboard the rocket, as well as being the interface for Limewire.

# Setup
This code will only work if you have lwip, ethernet, and FreeRTOS setup correctly on your board, for how to do this on our Nucleo boards see [here](https://github.com/stm32-hotspot/STM32H7-LwIP-Examples/tree/main), or use the ethernet template elsewhere in this repo. Make sure that the FreeRTOS heap is set to more than **32kb** or else this will crash!

Configure the Ethernet adapter on your computer to have an IP address in the range of `192.168.50.x` where x is anything other than 10 and with the subnet mask of `255.255.255.0`. I'd recommend trying to ping the address `192.168.50.10` to confirm the connection as a debugging step.

# Example Usage
```c
  char buf[300];

  MX_LWIP_Init();

  server_init(); // Starts the listener, reader, and writer threads
  
  for(;;) {
	  Raw_message msg = {-1, buf, 300};
	  server_read(&msg); // Blocks until a message has been recieved

	  char* str = malloc(15);
	  memcpy(str, "Toggled LED\r\n", 14);
	  int ret_len = strlen(str);
	  Raw_message response = {msg.connection_fd, str, ret_len};

	  server_send(&response); // Sends a message by adding it to the queue of messages to be sent
	  HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
  }
  ```

On a Windows laptop:
Run `telnet 192.168.50.10 5000`
Send any message and you should get the response as defined above ^