#include "autosequence-script.h"

void execute_flight_autosequence(){
    FlightPhase phase = ST_OPEN_MPV1; // starts at state0

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = xTaskGetTickCount();

    Detector baro_detector = {0};
    Detector imu_detector = {0};
    Detector temp_C_detector = {0};

    // For subsonic flights just set 'phase' to ST_WAIT_APOGEE
    // Doing so will bypass the MECO phase and immediately arm/monitor the barometers
    // ST_WAIT_MECO is phase 1 for full supersonic flights
    // flags

    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;

    uint32_t handoff_timestamp = 0; // time (in ms) when handoff occurs
    uint32_t ignition_timestamp = 0;// approximate time (in ms) when igntion occurs
    uint32_t meco_timestamp = 0;         // time (in ms) when MECO is detected
    // uint32_t lockout_tick = 0; // will be computed when MECO_flag is set --> Relevance of lockout?

    for (;;){

        float bar1 = 0.0f;
        float bar2 = 0.0f;
        float imu1 = 0.0f;
        float imu2 = 0.0f;

        //temp from barometers not yet implemented in barometer driver
        //but would be helpful here, otherwise will need to estimate it
        float bar1_temp_K = 0.0f;
        float bar2_temp_K = 0.0f;

        if (xSemaphoreTake(Rocket_h.fcState_access, pdMS_TO_TICKS(5)) == pdPASS)
        {
            bar1 = Rocket_h.fcState.bar1; // convert to needed units if necessary
            bar2 = Rocket_h.fcState.bar2;
            imu1 = Rocket_h.fcState.imu1_A.XL_x;
            imu2 = Rocket_h.fcState.imu2_A.XL_x;

            //Temp from barometers would be useful here too 
            //immediately convert to Kelvin
            bar1_temp_K = Rocket_h.fcState.bar1_temp_C + 273.15f;
            bar2_temp_K = Rocket_h.fcState.bar2_temp_C + 273.15f;

            xSemaphoreGive(Rocket_h.fcState_access);
        }



        switch (phase) {
            case ST_OPEN_MPV1:
                energizeMPV1();

                if (getTime() - handoff_timestamp > MPV_DELAY){
                    ignition_timestamp = getTime();
                    phase = ST_OPEN_BOTH_MPVS;
                }                    

                break;

            
            case ST_OPEN_BOTH_MPVS:
                energizeMPV1();
                energizeMPV2();
                
                phase = ST_WAIT_MECO;
                break;


            case ST_WAIT_MECO: {
                insert(&imu_detector, imu1, imu2, phase);
                insert(&baro_detector, bar1, bar2, phase);
                insert(&temp_C_detector, bar1_temp_K, bar2_temp_K, phase); // insert temp in K or C?

                if(imu_detector.avg_size >= AD_CAPACITY)
                {

                    if (detect_MECO(&imu_detector) || getTime() - ignition_timestamp > MAX_MECO_WAIT)
                    {
                        MECO_flag = 1;
                        meco_timestamp = xTaskGetTickCount();
                        // TODO: SEND EVENT
                        phase = ST_WAIT_APOGEE;

                        /*
                        //COMPUTE LOCKOUT/WAIT-TIME
                        {
                            float avg_pressure;
                            float avg_temp;

                            //if baro buffer is ready just get the average
                            if(baro_detector.avg_size >= AD_CAPACITY)
                            {
                                int idx = (baro_detector.avg_index + AD_CAPACITY - 1) % AD_CAPACITY;
                                avg_pressure = baro_detector.average[idx];
                            }
                            //emergency fallback, avg the two most recent barometer readings
                            else
                            {
                                //MUST be hPa/mbar coming in from Rocket_h
                                avg_pressure = 0.5f * ((bar1) + (bar2));
                            }

                            //if temp buffer is ready just get the average
                            if (temp_C_detector.avg_size >= AD_CAPACITY)
                            {
                                int idx = (temp_C_detector.avg_index + AD_CAPACITY - 1) % AD_CAPACITY;
                                avg_temp = temp_C_detector.average[idx];
                            }
                            //emergency fallback, avg the two most recent temperature readings
                            else
                            {
                                //MUST be Celsius coming in from Rocket_h
                                avg_temp = 0.5f * ((bar1_temp_C) + (bar2_temp_C));
                            }

                                int meco_time_ms = (int)(meco_tick * portTICK_PERIOD_MS);

                                int wait_time_ms = compute_wait_time(meco_time_ms, avg_pressure, avg_temp);

                                //lockout_tick = pdMS_TO_TICKS(wait_time_ms);
                        }

                        */
                    }
                }

                break;
            } 


            case ST_WAIT_APOGEE: {
                insert(&baro_detector, bar1, bar2, phase);

                if(baro_detector.slope_size >= AD_CAPACITY) {
                    if (detect_apogee(&baro_detector)) {
                        apogee_flag = 1;
                        // TODO: SEND EVENT
                        phase = ST_WAIT_DROGUE;
                    }
                }

                break;
            } 

            case ST_WAIT_DROGUE: {
                insert(&baro_detector, bar1, bar2, phase);
            
                if(baro_detector.avg_size >= AD_CAPACITY)
                {
                    if (detect_altitude(&baro_detector, phase))
                    {
                        drogue_flag = 1;
                        // TODO: SEND EVENT
                        phase = ST_WAIT_MAIN;
                    }
                }

                break;
            } 


            case ST_WAIT_MAIN: {
                insert(&baro_detector, bar1, bar2, phase);
                if(baro_detector.avg_size >= AD_CAPACITY)
                {
                    if (detect_altitude(&baro_detector, phase))
                    {
                        main_flag = 1;
                        // TODO: SEND EVENT
                        phase = ST_DONE;
                    }
                }

                break;
            } 

            case ST_DONE: {
                vTaskSuspend(NULL);
            } break;
            
        }//end switch(phase)

        vTaskDelayUntil(&last, period);
    }
}