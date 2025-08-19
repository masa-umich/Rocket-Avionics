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

void insert(Detector * detector, float reading1, float reading2, FlightPhase phase);

int is_buffer_average_less_than_this_value(Detector *detector, float search_value);

int is_buffer_average_greater_than_this_value(Detector *detector, float search_value);

//phase 1
int detect_MECO(Detector *detector);

//wait-time helper 
float advance_chunk();

//wait-time helper 
float wait_time_peicewise();

TickType_t compute_wait_time();

//phase 2
int detect_apogee(Detector *detector);

//phase 3
int detect_altitude(Detector * detector, FlightPhase phase);

#endif