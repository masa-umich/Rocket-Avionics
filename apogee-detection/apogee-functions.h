#ifndef APOGEE_FUNCTIONS_H
#define APOGEE_FUNCTIONS_H

//size of the array of readings we maintain during the flight
//used with the barometer and the IMU
#define AD_CAPACITY 5          

typedef struct 
{
    //SET ALL VALUES TO ZERO 
    //UPON INITIALIZATION! 
    float readings_1[AD_CAPACITY]; 
    int index_1;
    int size_1;

    float readings_2[AD_CAPACITY];
    int index_2;
    int size_2;

    float average[AD_CAPACITY];
    int avg_index;
    int avg_size;

    float slope[AD_CAPACITY];
    int slope_i;
    int slope_size;
} Detector;

//What 'state' is our apogee-detection system in? 
typedef enum {
    ST_WAIT_MECO = 0,
    ST_MACH_LOCKOUT,  
    ST_WAIT_APOGEE,
    ST_WAIT_DROGUE,
    ST_WAIT_MAIN,
    ST_DONE
} FlightPhase;

float ad_mean(int size, float * arr);

int ad_min(int x, int y);

int ad_max(int x, int y); 

//Accepts temperatures in Kelvin
//Accepts pressures in hPa (converts to Pa)
float compute_rho(float pressure, float temp);

//Accepts temperature in Kelvin
//Returns sp of sound in m/s
float speed_of_sound(float temp);

//Estimate our height given pressure and temp
//Expects hPa and Kelvin
float compute_height(float avg_pressure /*,float avg_temp*/);

//Safely inserts data from barometers & imus using a circular buffer. 
//If we 'miss' a reading from a sensor, don't pass in bad data! Pass 0.0f
//Reason: this insert module will be used for barometer temp, barometer pressure, 
//and IMU x acceleration (up), and relies on imputation to handle missed readings.
//If the IMU or barometer driver misses data from the sensor it should pass 0.0f. 
//to the Flight_Computer_State_t in main.c so the data is good by the time it reaches here
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase);

int is_buffer_average_less_than_this_value(Detector *detector, float search_value);

int is_buffer_average_greater_than_this_value(Detector *detector, float search_value);

//phase 1
int detect_MECO(Detector *detector);

//wait-time helper 
float advance_chunk(float *h, float *v, float chunk_dt, float press, float temp);

//wait-time helper 
float wait_time_peicewise(float h0, float v0, float pressure, float temp, float chunk_dt, float max_time);

//computes the time the rocket will take to drop back below mach 1
//needs meco_time -> to index into speed_LUT for estimated velocity
float compute_wait_time(int meco_time_ms, float avg_pressure, float avg_temp);

//phase 2
int detect_apogee(Detector *detector);

//phase 3
int detect_altitude(Detector * detector, FlightPhase phase);

#endif