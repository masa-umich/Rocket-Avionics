#include "apogee-task.h"

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

void insert(Detector * detector, float * bar1_mutex, float * bar2_mutex)
{
    float bar1_reading = *bar1_mutex / 100;
    if(bar1_reading == 0.0){
        bar1_reading = ad_mean(detector->size_1, detector->readings_1);
    }

    detector->readings_1[detector->index_1] = bar1_reading;
    detector->index_1++;
    detector->index_1 = detector->index_1 % AD_CAPACITY;
    detector->size_1 = ad_min(detector->size_1 + 1, 5);

    float bar2_reading = *bar2_mutex / 100;
    if(bar2_reading == 0.0){
        bar2_reading = ad_mean(detector->size_2, detector->readings_2);
    }

    detector->readings_2[detector->index_2] = bar2_reading;
    detector->index_2++;
    detector->index_2 = detector->index_2 % AD_CAPACITY;
    detector->size_2 = ad_min(detector->size_2 + 1, 5);

    detector->previous[detector->prev_index] = 
        (ad_mean(detector->size_1, detector->readings_1) + 
            ad_mean(detector->size_2, detector->readings_2)) / 2;        
    detector->prev_index++;
    detector->prev_index = detector->prev_index % 5;
    detector->prev_size = ad_min(detector->prev_size + 1, 5);

    if(detector->prev_size >= 5)
    {
        detector->slope[detector->slope_i] = 
            (detector->previous[(detector->prev_index + 5 - 1) % 5 ] - 
            detector->previous[(detector->prev_index + 5 - 5) % 5]) / 5;
        printf("Slope : %f\n", detector->slope[detector->slope_i]); 
        detector->slope_size = ad_min(detector->slope_size + 1, 5);
        detector->slope_i = (detector->slope_i + 1) % 5;
    }
}

int detect_apog(Detector * detector)
{
    if (detector->slope_size < 5)
        return 0;
    else
    { 
        int count = 0;
        for(int i = 0; i < 5; ++i)
        {
            if(detector->slope[i] > 0)
                ++count;
        }

        if(count >= 4)
            return 1;
    }
    return 0;
}

void Detect_Apogee_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int * apogee_flag)
{
    if (detector->size_1 < 5 && detector->size_2 < 5){
        insert(detector, bar1_mutex, bar2_mutex);
    }
    
    else
    {
        insert(detector, bar1_mutex, bar2_mutex);
        if (detect_apog(detect_apog) && *apogee_flag == 0)
        {
            printf("Apogee Detected\n");
            *apogee_flag = 1;
        }
    }  
}