#ifndef AD_TASK
#define AD_TASK

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
}Detector;

typedef struct {
    float XL_x;
    float XL_y;
    float XL_z;
} Accel;

typedef enum {
    PILOT = 0,
    DROGUE,
    MAIN,
    IMU
} DetectionType;

float ad_mean(int size, float * arr);

int ad_min(int x, int y);

//Helper function used by all our tasks to insert barometer readings into our circular buffer
void insert(Detector * detector, float * bar1_mutex, float * bar2_mutex, DetectionType Type);

//We will need this if we rely on the IMUs. 
//Otherwise, simply start Detect_Apogee_Task sometime after liftoff
void Detect_MECO_task(Detector * monitor, Accel * IMU_1, Accel * IMU_2, int * MECO_flag);

//First Parachute
void Detect_Apogee_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int * apogee_flag);

//Second Parachute
void Detect_Drogue_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int *drogue_flag);

//Third Parachute
void Detect_Main_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int *main_flag);

//Used by Detect_Drogue_Task & Detection_Main_Task
int detect_altitude(Detector *detector, DetectionType Type);

//Used by Detect_Apogee_Task
int detect_apog(Detector * detector);

#endif