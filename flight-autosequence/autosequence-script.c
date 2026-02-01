#include "autosequence-script.h"
#include "complementary-filter.h"
#include "simplified-curve-fit.h"

extern const uint32_t dt;
const ALTITUDE_BUFFER_SIZE = 11;

void fill_altitude_buffer(float* altitude_buffer, float* time_buffer, float bar1, float bar2) {
    for (int i = 0; i < ALTITUDE_BUFFER_SIZE; ++i) {
        altitude_buffer[i] = compute_height((bar1 + bar2) / 2.0f);
        time_buffer[i] = getTime() / 1000.0f; // convert ms to seconds
        vTaskDelay(pdMS_TO_TICKS(dt)); //FIX THE DELAY ON THIS LINE
    }
}

void execute_flight_autosequence(){
    FlightPhase phase = ST_OPEN_MPV1; // starts at state0

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = xTaskGetTickCount();

    Detector baro_detector = {0};
    Detector imu_detector = {0};
    Detector temp_C_detector = {0};

    ComplementaryFilter imu_cf;
    cf_init(&imu_cf, 0.5f); // tau = 0.5 seconds

    // For subsonic flights just set 'phase' to ST_WAIT_APOGEE
    // Doing so will bypass the MECO phase and immediately arm/monitor the barometers
    // ST_WAIT_MECO is phase 1 for full supersonic flights
    // flags

    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;

    uint32_t handoff_timestamp = 0; // time (in ms) when handoff occursy
    uint32_t ignition_timestamp = 0;// approximate time (in ms) when igntion occurs
    uint32_t meco_timestamp = 0;         // time (in ms) when MECO is detected
    uint32_t lockout_timestamp = 0; // will be computed when MECO_flag is set

    float alttitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    bool heights_recorded = false;

    float pitch_buf[7] = {0}; // need to create a 5 or 7 elt buffer to store the pitch and yaw after MECO
    float yaw_buf[7] = {0};

    for (;;){

        float bar1 = 0.0f;
        float bar2 = 0.0f;
        float imu1 = 0.0f;
        float imu2 = 0.0f;

        //temp from barometers not yet implemented in barometer driver
        //but would be helpful here, otherwise will need to estimate it
        float bar1_temp_C = 0.0f;
        float bar2_temp_C = 0.0f;
        float bar1_temp_K = 0.0f;
        float bar2_temp_K = 0.0f;

        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y;

        float post_lockout_accel, post_lockout_vel, post_lockout_alt;

        if (xSemaphoreTake(Rocket_h.fcState_access, pdMS_TO_TICKS(5)) == pdPASS)
        {
            bar1 = Rocket_h.fcState.bar1; // convert to needed units if necessary
            bar2 = Rocket_h.fcState.bar2;
            imu1 = Rocket_h.fcState.imu1_A.XL_x;
            imu2 = Rocket_h.fcState.imu2_A.XL_x;

            /*
            accel_x = (Rocket_h.fcstate.imu1.XL_x + Rocket_h.fcstate.imu2.XL_x) / 2.0f;
            accel_y = (Rocket_h.fcstate.imu1.XL_y + Rocket_h.fcstate.imu2.XL_y) / 2.0f;
            accel_z = (Rocket_h.fcstate.imu1.XL_z + Rocket_h.fcstate.imu2.XL_z) / 2.0f;
            gyro_x = (Rocket_h.fcstate.imu1.W_X + Rocket_h.fcstate.imu2.W_X) / 2.0f;
            gyro_y = (Rocket_h.fcstate.imu1.W_Y + Rocket_h.fcstate.imu2.W_Y) / 2.0f;
            */

            bar1_temp_C = Rocket_h.fcState.bar1_temp_C;
            bar1_temp_K = bar1_temp_C + 273.15f;
            bar2_temp_C = Rocket_h.fcState.bar2_temp_C;
            bar2_temp_K = bar2_temp_C + 273.15f;

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
                energizeMPV1();
                energizeMPV2();
                
                insert(&imu_detector, imu1, imu2, phase);
                insert(&baro_detector, bar1, bar2, phase);
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase); // insert temp in K or C?

                if(imu_detector.avg_size >= AD_CAPACITY)
                {

                    if (detect_MECO(&imu_detector)) {
                        MECO_flag = 1;
                        meco_timestamp = getTime();
                        phase = ST_WAIT_APOGEE;
                        
                        //COMPUTE LOCKOUT/WAIT-TIME
                        float avg_pressure;
                        float avg_temp;

                        //if baro buffer is ready just get the average
                        if(baro_detector.avg_size >= AD_CAPACITY) {
                            int idx = (baro_detector.avg_index + AD_CAPACITY - 1) % AD_CAPACITY;
                            avg_pressure = baro_detector.average[idx];
                        }
                        //emergency fallback, avg the two most recent barometer readings
                        else {
                            avg_pressure = 0.5f * ((bar1) + (bar2)); //MUST be hPa/mbar coming in from Rocket_h
                        }


                        //if temp buffer is ready just get the average
                        if (temp_C_detector.avg_size >= AD_CAPACITY) {
                            int idx = (temp_C_detector.avg_index + AD_CAPACITY - 1) % AD_CAPACITY;
                            avg_temp = temp_C_detector.average[idx];
                        }
                        //emergency fallback, avg the two most recent temperature readings
                        else {
                            avg_temp = 0.5f * ((bar1_temp_C) + (bar2_temp_C)); //MUST be Celsius coming in from Rocket_h
                        }

                        int meco_time_ms = (int)(meco_timestamp * portTICK_PERIOD_MS);
                        lockout_timestamp = pdMS_TO_TICKS(compute_wait_time(meco_time_ms, avg_pressure, avg_temp)) + meco_timestamp;
                
                    }
                }

                break;
            } 

            case ST_MACH_LOCKOUT:
            {   
                // ORIENTATION FILTER UPDATE - NOT USED YET
                //float pitch, yaw;
                //cf_update(&imu_cf, &pitch, &yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y);
                /*
                for (int i = 6; i > 0; --i) {
                    pitch_buf[i] = pitch_buf[i - 1];
                    yaw_buf[i] = yaw_buf[i - 1];
                }
                pitch_buf[0] = pitch;
                yaw_buf[0] = yaw;
                */

                insert(&baro_detector,bar1, bar2, phase);

                if (getTime() >= lockout_timestamp)
                {
                    phase = ST_WAIT_APOGEE;
                }
                
                break;

            } 

            case ST_WAIT_APOGEE: {
                if (!heights_recorded) {
                    fill_altitude_buffer(alttitude_readings, bar1, bar2);
                    quadr_curve_fit(alttitude_readings, time_readings, &post_lockout_accel, &post_lockout_vel, &post_lockout_alt);
                    heights_recorded = true;
                }
                
                insert(&baro_detector, bar1, bar2, phase);

                if(baro_detector.slope_size >= AD_CAPACITY) {
                    if (detect_apogee(&baro_detector)) {
                        apogee_flag = 1;
                        blowoutPanel();
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
                        deployDrogue();
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
                        deployMain();
                        phase = ST_DONE;
                    }
                }

                break;
            } 

            case ST_DONE: {
                return;
            } break;
            
        }//end switch(phase)

        vTaskDelayUntil(&last, period);
    }
}