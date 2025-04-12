#include "apogee-task.h"
#include "math.h"

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

void insert(Detector * detector, float * bar1_mutex, float * bar2_mutex, DetectionType Type)
{
    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    float bar1_reading = *bar1_mutex / 100.0; //Division by 100.0 may not be necessary here
    if(bar1_reading == 0.0){                  //Depending on the barometer value received from the driver
        bar1_reading = ad_mean(detector->size_1, detector->readings_1);
    }

    detector->readings_1[detector->index_1] = bar1_reading;
    detector->index_1++;
    detector->index_1 = detector->index_1 % AD_CAPACITY;
    detector->size_1 = ad_min(detector->size_1 + 1, AD_CAPACITY);

    //WARNING!!!! COMPARING FLOAT TO ZERO HAS CHANCE OF FAILURE!!!!!!
    float bar2_reading = *bar2_mutex / 100.0; //Division by 100.0 may not be necessary here 
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

    if (Type == PILOT)
    {
        if (detector->avg_size >= AD_CAPACITY)
        {
            detector->slope[detector->slope_i] =
                (detector->average[(detector->avg_index + AD_CAPACITY - 1) % AD_CAPACITY] -
                 detector->average[(detector->avg_index + AD_CAPACITY - AD_CAPACITY) % AD_CAPACITY]) /
                AD_CAPACITY;
            printf("Slope : %f\n", detector->slope[detector->slope_i]);
            detector->slope_size = ad_min(detector->slope_size + 1, AD_CAPACITY);
            detector->slope_i = (detector->slope_i + 1) % AD_CAPACITY;
        }
    }
}

int detect_apog(Detector * detector)
{
    if (detector->slope_size < AD_CAPACITY)
        return 0;
    else
    { 
        int count = 0;
        for(int i = 0; i < AD_CAPACITY; ++i)
        {
            if(detector->slope[i] > 0)
                ++count;
        }

        if(count >= 4)
            return 1;
    }
    return 0;
}

int detect_MECO(Detector * detector)
{
    //probably need some different logic here than slope.
    //checking for if IMU is nearing 1.2 or similar. 
}

int detect_altitude(Detector *detector, DetectionType Type)
{
    if (detector->avg_size < AD_CAPACITY)
        return 0;

    else
    {
        int count = 0;

        if(Type == DROGUE)
        {
            for (int i = 0; i < AD_CAPACITY; ++i)
            {
                //drogue occurs at 1500 meters or roughly 850 millibar
                if (detector->average[i] > 850)
                    ++count;
            }
        }

        else if(Type == MAIN)
        {
            for (int i = 0; i < AD_CAPACITY; ++i)
            {
                //main occurs at 300 meters or roughly 900 millibar
                if (detector->average[i] > 900)
                    ++count;
            }
        }

        if (count >= 4)
            return 1;
    }

    return 0;
}


void Detect_Apogee_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int * apogee_flag)
{
    if (detector->size_1 < AD_CAPACITY && detector->size_2 < AD_CAPACITY){
        insert(detector, bar1_mutex, bar2_mutex, PILOT);
    }
    
    else
    {
        insert(detector, bar1_mutex, bar2_mutex, PILOT);
        if (detect_apog(detector) && *apogee_flag == 0)
        {
            //Deploy Pilot Chute
            printf("Apogee Detected\n");
            *apogee_flag = 1;
        }
    }  
}

void Detect_Drogue_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int *drogue_flag)
{
    if(detect_altitude(detector, DROGUE))
    {
        *drogue_flag = 1;
    }
}

void Detect_Main_Task(Detector * detector, float *bar1_mutex, float *bar2_mutex, int *main_flag)
{
    if(detect_altitude(detector, MAIN))
    {
        *main_flag = 1;
    }
}


void Detect_MECO_task(Detector * monitor, Accel * IMU_1, Accel * IMU_2, int * MECO_flag)
{
    float reading_1 = sqrt(pow(IMU_1->XL_x, 2) + pow(IMU_1->XL_y, 2) + pow(IMU_1->XL_z, 2));
    float reading_2 = sqrt(pow(IMU_2->XL_x, 2) + pow(IMU_2->XL_y, 2) + pow(IMU_2->XL_z, 2));

    if (monitor->size_1 < AD_CAPACITY && monitor->size_2 < AD_CAPACITY){
        insert(monitor, &reading_1, &reading_2, IMU);
    }
    
    else
    {
        insert(monitor, &reading_1, &reading_2, IMU);
        if (detect_MECO(monitor) && *MECO_flag == 0)
        {
            printf("MECO Detected\n");
            *MECO_flag = 1;
        }
    }  
}

