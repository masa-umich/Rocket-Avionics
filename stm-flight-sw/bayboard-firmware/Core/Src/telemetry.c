/*
 * telemetry.c
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#include "telemetry.h"

extern Sensors_t sensors_h;
extern SPI_HandleTypeDef hspi4;
extern PT_t PT1_h;
extern PT_t PT2_h;
extern PT_t PT3_h;
extern PT_t PT4_h;
extern PT_t PT5_h;
extern PT_t PT6_h;
extern PT_t PT7_h;
extern PT_t PT8_h;
extern PT_t PT9_h;
extern PT_t PT10_h;

void TelemetryTask(void *argument) {
	// started telemetry thread
	log_message(STAT_TELEM_TASK_STARTED, -1);
	for(;;) {
		uint32_t startTime = HAL_GetTick();
		// Read from sensors
		uint16_t adc1_values[16] = {0};
		uint16_t adc2_values[16] = {0};
		read_adc(&hspi4, &(sensors_h.adc1_h), adc1_values);
		if(adc1_values[ADC1_3V3_BUS_I] == 0) {
			// ADC read error
			log_peri_message(BB_ERR_ADC1_READ, BB_ERR_PERI_TYPE_ADC1);
		}
		read_adc(&hspi4, &(sensors_h.adc2_h), adc2_values);

  		Accel XL_readings1 = {0};
  		Accel XL_readings2 = {0};
  		AngRate angRate_readings1 = {0};
  		AngRate angRate_readings2 = {0};
  		int imu1_stat = IMU_getAccel(&(sensors_h.imu1_h), &XL_readings1);
  		if(imu1_stat == -1) {
  			// IMU 1 read error
			log_peri_message(ERR_IMU_READ "1", BB_ERR_PERI_TYPE_IMU1);
  		}
  		int imu2_stat = IMU_getAccel(&(sensors_h.imu2_h), &XL_readings2);
  		if(imu2_stat == -1) {
  			// IMU 2 read error
			log_peri_message(ERR_IMU_READ "2", BB_ERR_PERI_TYPE_IMU2);
  		}
  		IMU_getAngRate(&(sensors_h.imu1_h), &angRate_readings1);
  		IMU_getAngRate(&(sensors_h.imu2_h), &angRate_readings2);

  	  	float pres1 = 0.0;
  	  	float pres2 = 0.0;
  	  	int bar1_stat = MS5611_getPres(&(sensors_h.bar1_h), &pres1, &(sensors_h.prom1), OSR_1024);
  	  	if(bar1_stat) {
  	  		// BAR 1 read error
			log_peri_message(ERR_BAR_READ "1", BB_ERR_PERI_TYPE_BAR1);
  	  	}
  	  	int bar2_stat = MS5611_getPres(&(sensors_h.bar2_h), &pres2, &(sensors_h.prom2), OSR_1024);
  	  	if(bar2_stat) {
  	  		// BAR 2 read error
			log_peri_message(ERR_BAR_READ "2", BB_ERR_PERI_TYPE_BAR2);
  	  	}

  	  	float TCvalues[NUM_TCS];
  	  	int TC_stat = ADS_readAll(&(sensors_h.tc_main_h), TCvalues);
  	  	if(TC_stat || isnan(TCvalues[0]) || isnan(TCvalues[1]) || isnan(TCvalues[2]) || isnan(TCvalues[3]) || isnan(TCvalues[4]) || isnan(TCvalues[5])) {
  	  		// ADS read error
			log_peri_message(ERR_ADS_READ, BB_ERR_PERI_TYPE_ADS);
  	  	}

  	  	VLV_OpenLoad vlv1_old = 0;
  	  	VLV_OpenLoad vlv2_old = 0;
  	  	VLV_OpenLoad vlv3_old = 0;
  	  	VLV_OpenLoad vlv4_old = 0;
  	  	VLV_OpenLoad vlv5_old = 0;
  	  	uint8_t old_stat = 1;

  	  	if(xSemaphoreTake(Board_h.bbValve_access, 5) == pdPASS) {
  	  		vlv1_old = VLV_isOpenLoad(Board_h.bbValves[0]);
  	  		vlv2_old = VLV_isOpenLoad(Board_h.bbValves[1]);
  	  		vlv3_old = VLV_isOpenLoad(Board_h.bbValves[2]);
  	  		vlv4_old = VLV_isOpenLoad(Board_h.bbValves[3]);
  	  		vlv5_old = VLV_isOpenLoad(Board_h.bbValves[4]);
  	  		old_stat = 0;
  	  		xSemaphoreGive(Board_h.bbValve_access);
  	  	}

  	  	uint64_t recordtime = get_rtc_time();


  	  	// Set board state struct
  		if(xSemaphoreTake(Board_h.bbState_access, 5) == pdPASS) {
  			if(!bar1_stat) {
  				Board_h.bbState.bar1 = pres1;
  			}

  			if(!bar2_stat) {
  				Board_h.bbState.bar2 = pres2;
  			}

  			Board_h.bbState.imu1_A = XL_readings1;
  			Board_h.bbState.imu1_W = angRate_readings1;
  			Board_h.bbState.imu2_A = XL_readings2;
  			Board_h.bbState.imu2_W = angRate_readings2;

  			Board_h.bbState.pt1 = PT_calc(PT1_h, adc2_values[ADC2_PT1_I]);
  			Board_h.bbState.pt2 = PT_calc(PT2_h, adc2_values[ADC2_PT2_I]);
  			Board_h.bbState.pt3 = PT_calc(PT3_h, adc2_values[ADC2_PT3_I]);
  			Board_h.bbState.pt4 = PT_calc(PT4_h, adc2_values[ADC2_PT4_I]);
  			Board_h.bbState.pt5 = PT_calc(PT5_h, adc2_values[ADC2_PT5_I]);
  			Board_h.bbState.pt6 = PT_calc(PT6_h, adc2_values[ADC2_PT6_I]);
  			Board_h.bbState.pt7 = PT_calc(PT7_h, adc2_values[ADC2_PT7_I]);
  			Board_h.bbState.pt8 = PT_calc(PT8_h, adc2_values[ADC2_PT8_I]);
  			Board_h.bbState.pt9 = PT_calc(PT9_h, adc2_values[ADC2_PT9_I]);
  			Board_h.bbState.pt10 = PT_calc(PT10_h, adc2_values[ADC2_PT10_I]);

  			Board_h.bbState.bus24v_voltage = bus_voltage_calc(adc1_values[ADC1_24V_BUS_I], POWER_24V_RES_A, POWER_24V_RES_B);
  			Board_h.bbState.bus12v_voltage = bus_voltage_calc(adc1_values[ADC1_12V_BUS_I], POWER_12V_RES_A, POWER_12V_RES_B);
  			Board_h.bbState.bus5v_voltage = bus_voltage_calc(adc1_values[ADC1_5V_BUS_I], POWER_5V_RES_A, POWER_5V_RES_B);
  			Board_h.bbState.bus3v3_voltage = bus_voltage_calc(adc1_values[ADC1_3V3_BUS_I], POWER_3V3_RES_A, POWER_3V3_RES_B);

  			Board_h.bbState.bus24v_current = current_sense_calc(adc1_values[ADC1_24V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Board_h.bbState.bus12v_current = current_sense_calc(adc1_values[ADC1_12V_CURRENT_I], POWER_SHUNT_12V_24V, DIVIDER_12V_24V);
  			Board_h.bbState.bus5v_current = current_sense_calc(adc1_values[ADC1_5V_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);
  			Board_h.bbState.bus3v3_current = current_sense_calc(adc1_values[ADC1_3V3_CURRENT_I], POWER_SHUNT_3V3_5V, DIVIDER_3V3_5V);

  			Board_h.bbState.vlv1_current = current_sense_calc(adc1_values[ADC1_VLV1_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Board_h.bbState.vlv2_current = current_sense_calc(adc1_values[ADC1_VLV2_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Board_h.bbState.vlv3_current = current_sense_calc(adc1_values[ADC1_VLV3_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Board_h.bbState.vlv4_current = current_sense_calc(adc1_values[ADC1_VLV4_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);
  			Board_h.bbState.vlv5_current = current_sense_calc(adc1_values[ADC1_VLV5_CURRENT_I], VALVE_SHUNT_RES, DIVIDER_VALVE);

  			if(!TC_stat) {
  				if(!isnan(TCvalues[0])) {
  					Board_h.bbState.tc1 = TCvalues[0];
  				}
  				if(!isnan(TCvalues[1])) {
  					Board_h.bbState.tc2 = TCvalues[1];
  				}
  				if(!isnan(TCvalues[2])) {
  					Board_h.bbState.tc3 = TCvalues[2];
  				}
  				if(!isnan(TCvalues[3])) {
  					Board_h.bbState.tc4 = TCvalues[3];
  				}
  				if(!isnan(TCvalues[4])) {
  					Board_h.bbState.tc5 = TCvalues[4];
  				}
  				if(!isnan(TCvalues[5])) {
  					Board_h.bbState.tc6 = TCvalues[5];
  				}
  			}

  			if(!old_stat) {
  				Board_h.bbState.vlv1_old = vlv1_old;
  				Board_h.bbState.vlv2_old = vlv2_old;
  				Board_h.bbState.vlv3_old = vlv3_old;
  				Board_h.bbState.vlv4_old = vlv4_old;
  				Board_h.bbState.vlv5_old = vlv5_old;
  			}

  			Board_h.bbState.timestamp = recordtime;

  			xSemaphoreGive(Board_h.bbState_access);
  		}
  		else {
  			// No telemetry updated
  			log_message(ERR_TELEM_NOT_UPDATED, BB_ERR_TYPE_TELEM_NUPDATED);
  		}

  		Message telemsg = {0};
  		telemsg.type = MSG_TELEMETRY;
  		if(!pack_bb_telemetry_msg(&(telemsg.data.telemetry), recordtime, 5)) {
  			if(send_msg_to_device(&telemsg, 5, 11 + (4 * BB1_TELEMETRY_CHANNELS) + 5) == -2) {
  				// txbuffer full or memory error
  	  			log_message(ERR_TELEM_MEM_ERR, BB_ERR_TYPE_TELEM_MEM_ERR);
  			}
  		}

  		// Target TELEMETRY_HZ polling rate, but always delay for at least 1 tick, otherwise we risk not letting other tasks get CPU time. Actually I'm not sure this is true, but a 1ms delay is fine regardless

		uint32_t delta = HAL_GetTick() - startTime;
		if(delta > (1000 / TELEMETRY_HZ) * 2) {
			// telemetry collection overtime by more than 2x
  			log_message(ERR_TELEM_OVERTIME, BB_ERR_TYPE_TELEM_OVERTIME);
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
