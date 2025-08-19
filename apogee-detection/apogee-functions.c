#include "apogee-functions.h"
#include <math.h>

#include "FreeRTOS.h"
#include "LSM6DSO32XTR.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "task.h"
#include "main.h"
#include "speed_LUT.h"

//TO DELETE 
//This just has copy/pasted typedefs for 
//Rocket_State_t, Flight_Computer_State_t etc.
//Made while I was building this file out so I didn't need to 
//change main.c/main.h in limestone-firmware part of repo! 
//Needed becuase this program couldn't 'see' Rocket_h - the main rocket handle 
#include "temp-header.h" 

float ad_mean(int size, float * arr)
{
    if (size == 0)
        return 0;
    float sum = 0;
    for(int i = 0; i < size; ++i)
        sum += arr[i];

    return (sum / (float)size);
}

int ad_min(int x, int y)
{
    if(x < y)
        return x;
    else return y;
}

void insert(Detector * detector, float reading1, float reading2, FlightPhase phase)
{
    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    float bar1_reading = reading1 / 100.0; //Division by 100.0 may not be necessary here
    if(bar1_reading == 0.0){                  //Depending on the barometer value received from the driver
        bar1_reading = ad_mean(detector->size_1, detector->readings_1);
    }

    detector->readings_1[detector->index_1] = bar1_reading;
    detector->index_1++;
    detector->index_1 = detector->index_1 % AD_CAPACITY;
    detector->size_1 = ad_min(detector->size_1 + 1, AD_CAPACITY);

    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    float bar2_reading = reading2 / 100.0; //Division by 100.0 may not be necessary here 
    if(bar2_reading == 0.0){                  //Depending on the barometer value received from the driver
        bar2_reading = ad_mean(detector->size_2, detector->readings_2);
    }

    detector->readings_2[detector->index_2] = bar2_reading;
    detector->index_2++;
    detector->index_2 = detector->index_2 % AD_CAPACITY;
    detector->size_2 = ad_min(detector->size_2 + 1, AD_CAPACITY);

    detector->average[detector->avg_index] = 
        (ad_mean(detector->size_1, detector->readings_1) + 
            ad_mean(detector->size_2, detector->readings_2)) / 2;        
    detector->avg_index++;
    detector->avg_index = detector->avg_index % AD_CAPACITY;
    detector->avg_size = ad_min(detector->avg_size + 1, AD_CAPACITY);

    if (phase == ST_WAIT_APOGEE)
    {
        if (detector->avg_size >= AD_CAPACITY)
        {
            detector->slope[detector->slope_i] =
                (detector->average[(detector->avg_index + AD_CAPACITY - 1) % AD_CAPACITY] -
                 detector->average[(detector->avg_index + AD_CAPACITY - AD_CAPACITY) % AD_CAPACITY]) /
                AD_CAPACITY;
            
            detector->slope_size = ad_min(detector->slope_size + 1, AD_CAPACITY);
            detector->slope_i = (detector->slope_i + 1) % AD_CAPACITY;
        }
    }
}

int is_buffer_average_less_than_this_value(Detector * detector, float search_value)
{
    if (detector->avg_size < AD_CAPACITY)
        return 0;
    else
    {
        int count = 0;
        for (int i = 0; i < AD_CAPACITY; ++i)
        {
            if (detector->average[i] < search_value)
                ++count;
        }
        if (count >= (AD_CAPACITY * 0.8))
            return 1;
    }
    return 0;
}

int is_buffer_average_greater_than_this_value(Detector * detector, float search_value)
{
    if (detector->avg_size < AD_CAPACITY)
        return 0;
    else
    {
        int count = 0;
        for (int i = 0; i < AD_CAPACITY; ++i)
        {
            if (detector->average[i] > search_value)
                ++count;
        }
        if (count >= (AD_CAPACITY * 0.8))
            return 1;
    }
    return 0;
}

int detect_MECO(Detector *detector)
{
    return is_buffer_average_less_than_this_value(detector, 0);
}

int detect_apogee(Detector *detector)
{
    if (detector->slope_size < AD_CAPACITY)
        return 0;
    else
    {
        int count = 0;
        for (int i = 0; i < AD_CAPACITY; ++i)
        {
            if (detector->slope[i] > 0)
                ++count;
        }

        // if 80% of the readings in our slope buffer are positive DETECT APOGEE
        if (count >= (AD_CAPACITY * 0.8))
            return 1;
    }
}

int detect_altitude(Detector * detector, FlightPhase phase)
{
    if (phase == ST_WAIT_DROGUE)
        return is_buffer_average_greater_than_this_value(detector, 825); //approximate pressure in hPa at 1500 m

    else if (phase == ST_WAIT_MAIN)
        return is_buffer_average_greater_than_this_value(detector, 975); //aproximate pressure in hPa at 300 m 
    
    else 
        return 0;
}



static void apogee_task(void *arg)
{

    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    TickType_t last = xTaskGetTickCount();

    Detector baro_detector = {0};
    Detector imu_detector = {0};

    // For subsonic flights just set 'phase' to ST_WAIT_APOGEE
    // Doing so will bypass the MECO phase and immediately arm/monitor the barometers
    // ST_WAIT_MECO is phase 1 for full supersonic flights
    FlightPhase phase = ST_WAIT_MECO;

    // flags
    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;

    // NOT SURE ABOUT THIS
    TickType_t meco_tick = 0;
    TickType_t lockout_ms = 0; // will be computed when MECO_flag is set
    // NOT SURE ABOUT THIS

    while (1)
    {
        float bar1 = 0;
        float bar2 = 0;
        float imu1 = 0;
        float imu2 = 0;

        if (xSemaphoreTake(Rocket_h.fcState_access, pdMS_TO_TICKS(5)) == pdPASS)
        {
            bar1 = Rocket_h.fcState.bar1; // convert to needed units if necessary
            bar2 = Rocket_h.fcState.bar2;
            imu1 = Rocket_h.fcState.imu1_A.XL_x;
            imu2 = Rocket_h.fcState.imu2_A.XL_x;
            xSemaphoreGive(Rocket_h.fcState_access);
        }

        switch (phase)
        {
        case ST_WAIT_MECO:
        {
            if (imu_detector.size_1 < AD_CAPACITY && imu_detector.size_2 < AD_CAPACITY)
            {
                insert(&imu_detector, imu1, imu2, phase);
            }
            else
            {
                insert(&imu_detector, imu1, imu2, phase);

                if (detect_MECO(&imu_detector))
                {
                    MECO_flag = 1;
                    // TODO: SEND EVENT
                    phase = ST_MACH_LOCKOUT;
                    // TODO: COMPUTE LOCKOUT TIME
                }
            }
        } break;

        case ST_MACH_LOCKOUT:
        {
            if ((xTaskGetTickCount() - meco_tick) >= pdMS_TO_TICKS(lockout_ms))
            {
                phase = ST_WAIT_APOGEE;
            }
        } break;

        case ST_WAIT_APOGEE:
        {
            if (baro_detector.size_1 < AD_CAPACITY && baro_detector.size_2 < AD_CAPACITY)
            {
                insert(&baro_detector, bar1, bar2, phase);
            }
            else
            {
                insert(&baro_detector, bar1, bar2, phase);

                if (detect_apogee(&baro_detector))
                {
                    apogee_flag = 1;
                    // TODO: SEND EVENT
                    phase = ST_WAIT_DROGUE;
                }
            }
        } break;

        case ST_WAIT_DROGUE:
        {
            if (baro_detector.size_1 < AD_CAPACITY && baro_detector.size_2 < AD_CAPACITY)
            {
                insert(&baro_detector, bar1, bar2, phase);
            }
            else
            {
                insert(&baro_detector, bar1, bar2, phase);

                if (detect_altitude(&baro_detector, phase))
                {
                    drogue_flag = 1;
                    // TODO: SEND EVENT
                    phase = ST_WAIT_MAIN;
                }
            }
        } break;

        case ST_WAIT_MAIN:
        {
            if (baro_detector.size_1 < AD_CAPACITY && baro_detector.size_2 < AD_CAPACITY)
            {
                insert(&baro_detector, bar1, bar2, phase);
            }
            else
            {
                insert(&baro_detector, bar1, bar2, phase);

                if (detect_altitude(&baro_detector, phase))
                {
                    main_flag = 1;
                    // TODO: SEND EVENT
                    phase = ST_DONE;
                }
            }
        } break;

        case ST_DONE:
        {
            vTaskSuspend(NULL);
        } break;

        }//end switch(phase)

        vTaskDelayUntil(&last, period);
    }
}



