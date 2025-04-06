#ifndef AD_TASK
#define AD_TASK

//size of the array of readings we maintain during the flight
//used with the barometer and the IMU
#define AD_CAPACITY 5

typedef struct 
{
    //SET ALL VALUES TO ZERO 
    //UPON INITIALIZATION! 
    float slope[5];
    int slope_i;
    int slope_size;

    float readings_1[AD_CAPACITY]; 
    int index_1;
    int size_1;

    float readings_2[AD_CAPACITY];
    int index_2;
    int size_2;

    float previous[5];
    int prev_index;
    int prev_size;

}Detector;

typedef struct {
    //Acceleration in g
    float XL_x;
    float XL_y;
    float XL_z;
} Accel;

float ad_mean(int size, float * arr);

int ad_min(int x, int y);

void insert(Detector * detector, float * bar1_mutex, float * bar2_mutex);

int detect_apog(Detector * detector);

void Detect_MECO_task(Detector * monitor, Accel * IMU_1, Accel * IMU_2, int * MECO_flag);

void Detect_Apogee_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int * apogee_flag);

void Detect_MainChute_Task(Detector * detector, float *bar1_mutex, float*bar2_mutex, int *mainchute_flag);

#endif