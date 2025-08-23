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
//Needed becuase my intellisense couldn't 'see' Rocket_h - the main rocket handle 
#include "temp-header.h" 

//all necessary environment / rocket parameters defined here
const float CROSS_SECT_AREA = 0.08f;     // m^2
const float MASS_AT_MECO = 255.0f;       // kg
const float CD = 0.50f;                   //drag coefficient

const float P_SEA_LEVEL = 1.0f;          // atmospheres
const float P_11k = 0.2301575866f;       // atmospheres  
const float LAPSE_RATE = 0.0065f;        // Celsius per meter
const float T_SEA_LEVEL = 293.0f;        // Kelvin (20 deg C)
const float T_ISO = 216.65f;             // temp constant in isothermic region
const float G = 9.8f;                    // gravity
const float MOLAR_MASS_AIR = 0.02896f;   // kg/mol
const float R_GAS_CONST = 8.315f;        // gas constant

float ad_mean(int size, float * arr)
{
    if (size == 0)
        return 0.0f;
    float sum = 0.0f;
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

int ad_max(int x, int y)
{
    if(x > y)
        return x;
    else return y;
}

void insert(Detector * detector, float reading1, float reading2, FlightPhase phase)
{
    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    //TODO: VERIFY UNITIS HERE
    float bar1_reading = reading1 / 100.0f; //Division by 100.0 may not be necessary here
    if(bar1_reading == 0.0f){               //Depending on the barometer value received from the driver
        bar1_reading = ad_mean(detector->size_1, detector->readings_1);
    }

    detector->readings_1[detector->index_1] = bar1_reading;
    detector->index_1++;
    detector->index_1 = detector->index_1 % AD_CAPACITY;
    detector->size_1 = ad_min(detector->size_1 + 1, AD_CAPACITY);

    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    //TODO: VERIFY UNITS HERE
    float bar2_reading = reading2 / 100.0f;    //Division by 100.0 may not be necessary here 
    if(bar2_reading == 0.0f){                  //Depending on the barometer value received from the driver
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

    if (phase == ST_MACH_LOCKOUT || phase == ST_WAIT_APOGEE)
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
        if (count >= (AD_CAPACITY * 0.8f))
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
    return 0;
}

int detect_altitude(Detector * detector, FlightPhase phase)
{
    if (phase == ST_WAIT_DROGUE)                         //TODO:VERIFY UNITS HERE
        return is_buffer_average_greater_than_this_value(detector, 825.0f); //approximate pressure in hPa at 1500 m

    else if (phase == ST_WAIT_MAIN)                      //TODO:VERIFY UNITS HERE
        return is_buffer_average_greater_than_this_value(detector, 975.0f); //aproximate pressure in hPa at 300 m 
    
    else 
        return 0;
}

//Accepts temperature in Kelvin, returns sp of sound in m/s
float speed_of_sound(float temp)
{
    const float gamma = 1.4f;
    float Rspec = R_GAS_CONST / MOLAR_MASS_AIR;
    return sqrtf(gamma * Rspec * temp);
}

//Accepts temperatures in Kelvin, pressure in pascals?? 
float compute_rho(float pressure, float temp)
{             
    return (pressure * MOLAR_MASS_AIR) / (R_GAS_CONST * temp);
}

//MECO expected to occur below 11,000 m 
//So here we will just use the equation for the troposphere. 
//If that changes and we suspect we'll need to compute height above 11,000 m 
//we should add a separate `if` branch for the isotropic region (cause the formulas change).
//For the exact math, see Apogee Detection PDR slide on 'Noise Generation' 
float compute_height(float avg_pressure /*, float avg_temp*/)
{
    
    float expo = (R_GAS_CONST * LAPSE_RATE) / (G * MOLAR_MASS_AIR);
    float estimated_height = (T_SEA_LEVEL / LAPSE_RATE) * (1.0f - powf(avg_pressure / P_SEA_LEVEL, expo));

 
    return estimated_height;
}

//wait-time helper 
float advance_chunk(float *h, float *v, float chunk_dt, float press, float temp)
{
    const float local_s_of_s  = speed_of_sound(temp);
    const float rho = compute_rho(press, temp); // air density           
    const float drag_accel_coeff = ad_max(1e-9f, 0.5f * rho * CD * CROSS_SECT_AREA / MASS_AT_MECO); // guard

    // Helper lambdas for closed-form v(t)
    const float s      = sqrtf(drag_accel_coeff/G);
    const float omega  = sqrtf(G*drag_accel_coeff);

    //CLOSED FORM SOLUTION to dv/dt = -g -(drag_coeff_accel)v^2
    //Since we are precomputing drag_coeff_accel once per chunk, we treat it as constant.
    //This simplifies the above differential equation allowing us to come to this clean
    //solution that we can simply evaluate once per chunk! 
    //*******************************************************/
    #define vel_at(t) (sqrtf(G/drag_accel_coeff) * tanf(atanf(s*(*v)) - omega*(t)))
    //*******************************************************/

    // Time (with this drag_accel_coeff) to drop to local Mach 1 speed
    //This is just solving the above closed form solution for t 
    const float t_to_mach = ad_max(0.0f, (atanf(s*(*v)) - atanf(s*local_s_of_s)) / omega);

    // Use at most chunk_dt this pass
    const float t_used = ad_min(chunk_dt, t_to_mach);
    const float v_new  = vel_at(t_used);

    // Altitude update: trapezoid (good within a short chunk)
    const float dh = 0.5f * ((*v) + v_new) * t_used;

    //Update 
    *h += dh;
    *v  = v_new;
    return t_used; // caller decides if we reached Mach 1
}

//wait-time helper 
float wait_time_peicewise(float h0, float v0, float pressure, float temp, float chunk_dt, float max_time)
{
    float t_tot = 0.0, h = h0, v = v0;
    if (fabsf(v) <= speed_of_sound(h))
        return 0.0f;

    for (int i = 0 ; i < (int)ceilf(max_time/chunk_dt); ++i)
    {
        const float local_s_of_s = speed_of_sound(h);
        if(fabsf(v) <= local_s_of_s)
            break;
        
        float used = advance_chunk(&h, &v, chunk_dt, pressure, temp);
        t_tot += used;

        // If we hit Mach within this chunk (used < chunk_dt), stop.
        if (used + 1e-9f < chunk_dt) 
            break;
    }
    return t_tot;
}


float compute_wait_time(int meco_time, float avg_pressure, float avg_temp)
{
    //TODO:
    //goal here is to use integer division to index into our lookup
    //table to get estimate of current speed. 
    //VERIFY UNITS HERE.
    float current_speed = speed_LUT[(int)(meco_time / 25)];

    float current_height = compute_height(avg_pressure /*,avg_temp*/);

    float wait_s = wait_time_peicewise(current_height, current_speed, avg_pressure, avg_temp, 0.5f, 20.0f);

    //TODO: VERIFY UNITIS HERE
    return wait_s;
}

static void apogee_task(void *arg)
{

    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    TickType_t last = xTaskGetTickCount();

    Detector baro_detector = {0};
    Detector imu_detector = {0};
    Detector temp_C_detector = {0};

    // For subsonic flights just set 'phase' to ST_WAIT_APOGEE
    // Doing so will bypass the MECO phase and immediately arm/monitor the barometers
    // ST_WAIT_MECO is phase 1 for full supersonic flights
    FlightPhase phase = ST_WAIT_MECO;

    // flags
    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;

    TickType_t meco_tick = 0;  //will be set when MECO is detected 
    TickType_t lockout_tick = 0; // will be computed when MECO_flag is set

    while (1)
    {
        float bar1 = 0.0f;
        float bar2 = 0.0f;
        float imu1 = 0.0f;
        float imu2 = 0.0f;

        //temp from barometers not yet implemented in barometer driver
        //but would be helpful here, otherwise will need to estimate it
        float bar1_temp_C = 0.0f;
        float bar2_temp_C = 0.0f;

        if (xSemaphoreTake(Rocket_h.fcState_access, pdMS_TO_TICKS(5)) == pdPASS)
        {
            bar1 = Rocket_h.fcState.bar1; // convert to needed units if necessary
            bar2 = Rocket_h.fcState.bar2;
            imu1 = Rocket_h.fcState.imu1_A.XL_x;
            imu2 = Rocket_h.fcState.imu2_A.XL_x;

            //temp from barometers would be useful here too 
            bar1_temp_C = Rocket_h.fcState.bar1_temp_C;
            bar2_temp_C = Rocket_h.fcState.bar2_temp_C;

            xSemaphoreGive(Rocket_h.fcState_access);
        }

        switch (phase)
        {
        case ST_WAIT_MECO:
        {
            insert(&imu_detector, imu1, imu2, phase);
            insert(&baro_detector, bar1, bar2, phase);
            insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase);

            if(imu_detector.avg_size >= AD_CAPACITY)
            {
                if (detect_MECO(&imu_detector))
                {
                    MECO_flag = 1;
                    meco_tick = xTaskGetTickCount();
                    // TODO: SEND EVENT
                    phase = ST_MACH_LOCKOUT;

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
                            //TODO: VERIFY UNITS HERE
                            avg_pressure = 0.5f * ((bar1/100.0f) + (bar2/100.0f));
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
                            //TODO: VERIFY UNITS HERE 
                            avg_temp = 0.5f * ((bar1_temp_C/100.0f) + (bar2_temp_C/100.0f));
                        }

                            int meco_time = (int)(meco_tick * portTICK_PERIOD_MS);

                            int wait_time_s = compute_wait_time(meco_time, avg_pressure, avg_temp);

                            //TODO: VERIFY UNITS HERE
                            lockout_tick = pdMS_TO_TICKS(wait_time_s);
                    }
                }
            }
        } break;

        case ST_MACH_LOCKOUT:
        {
            insert(&baro_detector,bar1, bar2, phase);

            if ((xTaskGetTickCount() - meco_tick) >= lockout_tick)
            {
                phase = ST_WAIT_APOGEE;
            }

        } break;

        case ST_WAIT_APOGEE:
        {
            insert(&baro_detector, bar1, bar2, phase);

            if(baro_detector.slope_size >= AD_CAPACITY)
            {
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
        } break;

        case ST_WAIT_MAIN:
        {
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
        } break;

        case ST_DONE:
        {
            vTaskSuspend(NULL);
        } break;

        }//end switch(phase)

        vTaskDelayUntil(&last, period);
    }
}



