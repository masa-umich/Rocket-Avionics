/*
 * telemetry.c
 *
 *  Created on: Oct 12, 2025
 *      Author: felix
 */

void TelemetryTask(void *argument) {
	// started telemetry thread
	log_message(STAT_TELEM_TASK_STARTED, -1);
	for(;;) {
		uint32_t startTime = HAL_GetTick();
		// Read from sensors
		uint16_t adc_values[16] = {0};
		read_adc(&hspi4, &(sensors_h.adc_h), adc_values);
		if(adc_values[ADC_3V3_BUS_I] == 0) {
			// ADC read error
			log_peri_message(FC_ERR_SING_ADC_READ, FC_ERR_PERI_TYPE_ADC);
		}

  		Accel XL_readings1 = {0};
  		Accel XL_readings2 = {0};
  		AngRate angRate_readings1 = {0};
  		AngRate angRate_readings2 = {0};
  		int imu1_stat = IMU_getAccel(&(sensors_h.imu1_h), &XL_readings1);
  		if(imu1_stat == -1) {
  			// IMU 1 read error
			log_peri_message(ERR_IMU_READ "1", FC_ERR_PERI_TYPE_IMU1);
  		}
  		int imu2_stat = IMU_getAccel(&(sensors_h.imu2_h), &XL_readings2);
  		if(imu2_stat == -1) {
  			// IMU 2 read error
			log_peri_message(ERR_IMU_READ "2", FC_ERR_PERI_TYPE_IMU2);
  		}
  		IMU_getAngRate(&(sensors_h.imu1_h), &angRate_readings1);
  		IMU_getAngRate(&(sensors_h.imu2_h), &angRate_readings2);

  	  	float pres1 = 0.0;
  	  	float pres2 = 0.0;
  	  	int bar1_stat = MS5611_getPres(&(sensors_h.bar1_h), &pres1, &(sensors_h.prom1), OSR_1024);
  	  	if(bar1_stat) {
  	  		// BAR 1 read error
			log_peri_message(ERR_BAR_READ "1", FC_ERR_PERI_TYPE_BAR1);
  	  	}
  	  	int bar2_stat = MS5611_getPres(&(sensors_h.bar2_h), &pres2, &(sensors_h.prom2), OSR_1024);
  	  	if(bar2_stat) {
  	  		// BAR 2 read error
			log_peri_message(ERR_BAR_READ "2", FC_ERR_PERI_TYPE_BAR2);
  	  	}

  	  	float TCvalues[3];
  	  	int TC_stat = ADS_readAll(&(sensors_h.tc_main_h), TCvalues);
  	  	if(TC_stat || isnan(TCvalues[0]) || isnan(TCvalues[1]) || isnan(TCvalues[2])) {
  	  		// ADS read error
			log_peri_message(ERR_ADS_READ, FC_ERR_PERI_TYPE_ADS);
  	  	}

  	  	VLV_OpenLoad vlv1_old = 0;
  	  	VLV_OpenLoad vlv2_old = 0;
  	  	VLV_OpenLoad vlv3_old = 0;
  	  	uint8_t old_stat = 1;

  	  	if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
  	  		vlv1_old = VLV_isOpenLoad(Rocket_h.fcValves[0]);
  	  		vlv2_old = VLV_isOpenLoad(Rocket_h.fcValves[1]);
  	  		vlv3_old = VLV_isOpenLoad(Rocket_h.fcValves[2]);
  	  		old_stat = 0;
  	  		xSemaphoreGive(Rocket_h.fcValve_access);
  	  	}

  	  	uint64_t recordtime = get_rtc_time();


  	  	// Set global rocket state struct
  		if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
  			if(!bar1_stat) {
  				Rocket_h.fcState.bar1 = pres1;
  			}

  			if(!bar2_stat) {
  				Rocket_h.fcState.bar2 = pres2;
  			}

  			Rocket_h.fcState.imu1_A = XL_readings1;
  			Rocket_h.fcState.imu1_W = angRate_readings1;
  			Rocket_h.fcState.imu2_A = XL_readings2;
  			Rocket_h.fcState.imu2_W = angRate_readings2;

  			Rocket_h.fcState.pt1 = PT_calc(PT1_h, adc_values[ADC_PT1_I]);
  			Rocket_h.fcState.pt2 = PT_calc(PT2_h, adc_values[ADC_PT2_I]);
  			Rocket_h.fcState.pt3 = PT_calc(PT3_h, adc_values[ADC_PT3_I]);
  			Rocket_h.fcState.pt4 = PT_calc(PT4_h, adc_values[ADC_PT4_I]);
  			Rocket_h.fcState.pt5 = PT_calc(PT5_h, adc_values[ADC_PT5_I]);

  			Rocket_h.fcState.bus24v_voltage = bus_voltage_calc(adc_values[ADC_24V_BUS_I], POWER_24V_RES_A, POWER_24V_RES_B);
  			Rocket_h.fcState.bus12v_voltage = bus_voltage_calc(adc_values[ADC_12V_BUS_I], POWER_12V_RES_A, POWER_12V_RES_B);
  			Rocket_h.fcState.bus5v_voltage = bus_voltage_calc(adc_values[ADC_5V_BUS_I], POWER_5V_RES_A, POWER_5V_RES_B);
  			Rocket_h.fcState.bus3v3_voltage = bus_voltage_calc(adc_values[ADC_3V3_BUS_I], POWER_3V3_RES_A, POWER_3V3_RES_B);

  			Rocket_h.fcState.bus24v_current = current_sense_calc(adc_values[ADC_24V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Rocket_h.fcState.bus12v_current = current_sense_calc(adc_values[ADC_12V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Rocket_h.fcState.bus5v_current = current_sense_calc(adc_values[ADC_5V_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);
  			Rocket_h.fcState.bus3v3_current = current_sense_calc(adc_values[ADC_3V3_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);

  			Rocket_h.fcState.vlv1_current = current_sense_calc(adc_values[ADC_VLV1_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Rocket_h.fcState.vlv2_current = current_sense_calc(adc_values[ADC_VLV2_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Rocket_h.fcState.vlv3_current = current_sense_calc(adc_values[ADC_VLV3_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);

  			if(!TC_stat) {
  				if(!isnan(TCvalues[0])) {
  					Rocket_h.fcState.tc1 = TCvalues[0];
  				}
  				if(!isnan(TCvalues[1])) {
  					Rocket_h.fcState.tc2 = TCvalues[1];
  				}
  				if(!isnan(TCvalues[2])) {
  					Rocket_h.fcState.tc3 = TCvalues[2];
  				}
  			}

  			if(!old_stat) {
  				Rocket_h.fcState.vlv1_old = vlv1_old;
  				Rocket_h.fcState.vlv2_old = vlv2_old;
  				Rocket_h.fcState.vlv3_old = vlv3_old;
  			}

  			Rocket_h.fcState.timestamp = recordtime;

  			xSemaphoreGive(Rocket_h.fcState_access);
  		}
  		else {
  			// No telemetry updated
  			log_message(ERR_TELEM_NOT_UPDATED, FC_ERR_TYPE_TELEM_NUPDATED);
  		}

  		Message telemsg = {0};
  		telemsg.type = MSG_TELEMETRY;
  		if(!pack_fc_telemetry_msg(&(telemsg.data.telemetry), recordtime, 5)) {
  			if(send_msg_to_device(LimeWire_d, &telemsg, 5, 11 + (4 * FC_TELEMETRY_CHANNELS) + 5) == -2) {
  				// txbuffer full or memory error
  	  			log_message(ERR_TELEM_MEM_ERR, FC_ERR_TYPE_TELEM_MEM_ERR);
  			}
  		}

  		// Target TELEMETRY_HZ polling rate, but always delay for at least 1 tick, otherwise we risk not letting other tasks get CPU time. Actually I'm not sure this is true, but a 1ms delay is fine regardless

		uint32_t delta = HAL_GetTick() - startTime;
		if(delta > (1000 / TELEMETRY_HZ) * 2) {
			// telemetry collection overtime by more than 2x
  			log_message(ERR_TELEM_OVERTIME, FC_ERR_TYPE_TELEM_OVERTIME);
		}
		osDelay((1000 / TELEMETRY_HZ) > delta ? (1000 / TELEMETRY_HZ) - delta : 1);
	}

	// Reference code

	/*struct netbuf *outbuf = netbuf_new();
	void *pkt_buf = netbuf_alloc(outbuf, 3);

	memcpy(pkt_buf, "hi", 3);
	err_t send_err = netconn_sendto(sendudp, outbuf, &debug_addr, 1234);
	netbuf_delete(outbuf);

	HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, 0);
	osDelay(2000);
	HAL_GPIO_WritePin(GPS_NRST_GPIO_Port, GPS_NRST_Pin, 1);
	gps_handler gps;
	gps.huart = &huart10;
	int status = init_gps(&gps);
	for(;;) {
		if(xSemaphoreTake(gps.semaphore, 2)) {
			char temp_rx[100];
			if(gps.active_rx_buffer == 1) {
				memcpy(temp_rx, gps.rx_buffer_2, 100);
			}
			else {
				memcpy(temp_rx, gps.rx_buffer_1, 100);
			}
			gps_data data;
			parse_gps_sentence(temp_rx, &data);
		}
		osDelay(100);
	}*/
}
