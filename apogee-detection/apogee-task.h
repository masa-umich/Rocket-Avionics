#ifndef AD_TASK
#define AD_TASK

#define AD_CAPACITY 5

typedef struct 
{
    //set all vals to zero upon initialization
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

float ad_mean(int size, float * arr);

int ad_min(int x, int y);

void insert(Detector * detector, float * bar1_mutex, float * bar2_mutex);

int detect_apog(Detector * detector);

void Detect_Apogee_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int * apogee_flag);

#endif