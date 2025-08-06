/*The purpose of this script is to test the effectiveness of our apogee detection 
algorithm on SIMULATED DATA. This is not intended for use on hardware.
If you're looking for code to use on the board you want 'apogee-task.c' and 'apogee-task.h'
For this to work you need to have noise_generator.hpp and noise_generator.cpp in your directory.

Those files use a .txt that has the MASTRAN (MASA's in-house flight prediction software) 
data for the altitude of the rocket. The current .txt file in this repo 'height_data.txt' is what 
we used in 2025 and may be out-dated depending on when you access this.
Post hot-fire we are using a_x_data_new.txt, which is the acceleration data from 
If you're having trouble figuring out how to get up-to-date flight sim data out of MASTRAN 
email pickos@umich.edu OR the current A&R team lead. 
*/

#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cmath>

#include <numeric>
#include <iterator>
#include <iomanip>

#include "noise_generator.hpp"

using namespace std;

struct Program_Info
{
    int answer;
    int baro_size;
    int imu_size;
    int detection_method;
    bool MECO_flag = false;
    bool apogee_flag = false;
    bool drogue_flag = false;
    bool main_flag = false;
    bool flight_over = false;
};

struct Baro_Package
{
    Barometer baro_1;
    Barometer baro_2;
    std::pair<int, int> baro1_D1_D2;
    std::pair<int, int> baro2_D1_D2;
};

class Detector
{
    public: 

    Detector(const int CAP_IN, bool is_baro) : CAPACITY(CAP_IN), is_baro(is_baro)
    {
        readings_1 = new double[CAPACITY];
        readings_2 = new double[CAPACITY];
        average = new double[CAPACITY];
        slope = new double[CAPACITY];
    };

    ~Detector()
    {
        delete[] readings_1;
        delete[] readings_2;
        delete[] average;
        delete[] slope;
    }

    const int CAPACITY;
    bool is_baro;
    double*readings_1;
    double*readings_2;
    double*average;
    double*slope;

    //buffer for readings from barometer 2 
    //double readings_1[CAPACITY];
    int index_1 = 0;
    int size_1 = 0;

    //buffer for readings from barometer 2
    // double readings_2[CAPACITY];
    int index_2 = 0;
    int size_2 = 0;

    //buffer for the average of the two barometers
    // double average[5];
    int avg_index = 0;
    int avg_size = 0;

    //buffer for the 'slope' or how the average of the barometer readings 
    //changes over time
    // double slope[5];
    int slope_i = 0;
    int slope_size = 0;

    //just a basic mean. WATCH OUT for integer division!
    double mean(int size, double * arr)
    {
        if (size == 0)
            return 0;
        double sum = std::accumulate(arr, arr + size, 0.0);
        return sum / static_cast<double>(size);
    }

    //The premise of 'insert':
    //•Circular buffers -> necessary so we don't use up all the memory on the microcontroller. 
    //•Two barometers -> we want to take a moving / smoothed out average of the 2 readings. 
    void insert(double baro1, double baro2, Program_Info &info)
    {
        //this technique is called imputation! 
        //if we receive a bad reading, we don't want to corrupt the average.
        //to prevent this: if we detect nan or infinite, we take the mean of the readings 
        //CURRENTLY IN THE BUFFER, and insert that number into the place where 
        //NAN would have been inserted. 
            
        if(std::isnan(baro1) || !std::isfinite(baro1) || (is_baro && (baro1 <= 0 || baro1 >= 2000))){
            baro1 = mean(size_1, readings_1);}
        readings_1[index_1] = baro1;
        index_1++;
        index_1 = index_1 % CAPACITY;
        size_1 = std::min(size_1 + 1, CAPACITY);

        //same as above!
        if(std::isnan(baro2) || !std::isfinite(baro2) || (is_baro && (baro2 <= 0 || baro2 >= 2000))){
            baro2 = mean(size_2, readings_2);}
        readings_2[index_2] = baro2;
        index_2++;
        index_2 = index_2 % CAPACITY;
        size_2 = std::min(size_2 + 1, CAPACITY);

        average[avg_index] = (mean(size_1, readings_1) + mean(size_2, readings_2)) / 2;
        avg_index++;
        avg_index = avg_index % CAPACITY;
        avg_size = std::min(avg_size + 1, CAPACITY);



        //Wait until the 'average' buffer is full to start computing the slopes
        if(avg_size >= CAPACITY)
        {
            ////this 'if' block controls the detection of the steep drop in acceleration for IMUs (or just basic barometer handling)
            //if negative acceleration MECO detection was selected this block should be skipped  

            //example using step size of 5
            //Compare the MOST RECENT average reading to the reading from 5 STEPS ago
            //Our whole windows is size 5 so this effectively gives an estimate of the 
            //average change per step over our whole window
    
            //For example if our 'average' buffer is [2 3 5 9 8] and avg_index is currently 0.
            //This means the most recent reading will have been placed at avg_index 4 (becuase we've
            //now wrapped around to the start of our buffer by the time we reach this block) 

            //0 + 5 - 1 = 4 (most recent reading) average[4] = 8
            //0 + 5 - 5 = 0 (oldest reading) average[0] = 2
            //(8 - 2) / 5 = 1.2 
            //So we can estimate that our average change was roughly 1.2 per step over this window.
            if(info.detection_method == 2 || info.answer == 2 || info.MECO_flag == true)
            {
                slope[slope_i] = (average[(avg_index + CAPACITY - 1) % CAPACITY ] - 
                    average[(avg_index + CAPACITY - CAPACITY) % CAPACITY]) / CAPACITY;
            


                std::cout << slope[slope_i] << "\n";
                slope_size = std::min(slope_size + 1, CAPACITY);
                slope_i = (slope_i + 1) % CAPACITY;
            }
        }
    }

    bool is_buffer_average_less_than_this_value(double search_value)
    {
        if (avg_size < CAPACITY)
            return false;
        else
        {
            int count = 0;
            for (int i = 0; i < CAPACITY; ++i)
            {
                if (average[i] < search_value)
                    ++count;
            }
            if (count >= (CAPACITY * 0.8))
                return true;
        }
        return false;
    }

    bool is_buffer_average_greater_than_this_value(double search_value)
    {
        if (avg_size < CAPACITY)
            return false;
        else
        {
            int count = 0;
            for (int i = 0; i < CAPACITY; ++i)
            {
                if (average[i] > search_value)
                    ++count;
            }
            if (count >= (CAPACITY * 0.8))
                return true;
        }
        return false;
    }

    bool detect_apog(Program_Info &info)
    {
        if(info.apogee_flag == false)
        {
            if (slope_size < CAPACITY)
                return false;
            else
            { 
                int count = 0;
                for(int i = 0; i < CAPACITY; ++i)
                {
                    if(slope[i] > 0)
                        ++count;
                }

                //if 80% of the readings in our slope buffer are positive DETECT APOGEE
                if(count >= (CAPACITY * 0.8))
                    return true;
            }
        }
        else if (info.apogee_flag == true && info.drogue_flag == false)
        {
            return is_buffer_average_greater_than_this_value(825); //approximate pressure in hPa at 1500 m
        }

        else if(info.apogee_flag == true && info.drogue_flag == true && info.main_flag == false)
        {
            return is_buffer_average_greater_than_this_value(975); //aproximate pressure in hPa at 300 m 
        }

        return false;
    }

    bool detect_MECO(Program_Info &info)
    {
        //for steep drop detection
        //WARNING! MAGIC NUMBER USED HERE. Works for window size 5 on OUR CURRENT SIMULATED DATA 
        //Very possibly safer to simply search for 'negative acceleration' in real flight, as that  
        //should work across many different flight scenarios! 
        if (info.detection_method == 2)
        {
            if (slope_size < CAPACITY)
                return false;
            else
            {
                int count = 0;
                for (int i = 0; i < CAPACITY; ++i)
                {
                    //this number works best for window size 5
                    if (slope[i] < -0.2475)
                        ++count;
                }

                // if 80% of the readings in our slope buffer are less than -0 DETECT MECO
                if (count >= (CAPACITY * 0.8))
                    return true;
            }
            return false;
        }

        //for negative acceleration detection
        else if(info.detection_method == 1)
        {
            // if (avg_size < CAPACITY)
            //     return false;
            // else
            // {
            //     int count = 0;
            //     for (int i = 0; i < CAPACITY; ++i)
            //     {
            //         if (average[i] < 0)
            //             ++count;
            //     }
            //     if (count >= (CAPACITY * 0.8))
            //         return true;

            // } 
            // return false;

            return is_buffer_average_less_than_this_value(0);
        }


        return false;
    }
    
};

void read_detect_decide(int j, double current_height, Baro_Package &baro_package, Program_Info &info, Detector &detect, Environment &env)
{
        if (current_height > 21200 || info.apogee_flag == true)
        {
            double P_true = env.compute_pressure(current_height);
            double T_true = env.compute_temp(current_height);

            // inject noise
            double P_noisy_1 = P_true + gaussian_random(0, 1.5);
            double T_noisy_1 = T_true + gaussian_random(0, 8);

            double P_noisy_2 = P_true + gaussian_random(0, 1.5);
            double T_noisy_2 = T_true + gaussian_random(0, 8);

            baro_package.baro1_D1_D2 = baro_package.baro_1.guess_and_check(P_noisy_1, T_noisy_1);
            baro_package.baro2_D1_D2 = baro_package.baro_2.guess_and_check(P_noisy_2, T_noisy_2);

            auto [baro1_final_P, baro1_final_TEMP] = 
                baro_package.baro_1.forward_calculation(baro_package.baro1_D1_D2.first, baro_package.baro1_D1_D2.second);

            auto [baro2_final_P, baro2_final_TEMP] = 
                baro_package.baro_2.forward_calculation(baro_package.baro2_D1_D2.first, baro_package.baro2_D1_D2.second);

            cout << setw(10) << left << current_height;
            cout << setw(10) << baro1_final_P / 100 << setw(10) << baro2_final_P / 100;

            cout << setw(10) << right << static_cast<double>(j) / 100 << "\n";

            if (detect.size_1 < 5 && detect.size_2 < 5)
            {
                // detect.insert(std::round((baro1_final_P / 100) * 1000) / 1000 , std::round((baro2_final_P / 100) * 1000) / 1000);
                detect.insert(baro1_final_P / 100, baro2_final_P / 100, info);
            }
            else
            {
                // detect.insert(std::round((baro1_final_P / 100) * 1000) / 1000 , std::round((baro2_final_P / 100) * 1000) / 1000);
                detect.insert(baro1_final_P / 100, baro2_final_P / 100, info);

                if (detect.detect_apog(info))
                {
                    if (info.apogee_flag == false)
                    {
                        cout << "Apogee Detected!\n";
                        cout << "Beginning phase 3\n";
                        info.apogee_flag = true;
                    }

                    else if(info.apogee_flag == true && info.drogue_flag == false)
                    {
                        cout << "1500 meters, deploy drogue!\n";
                        info.drogue_flag = true;
                    }

                    else if(info.apogee_flag == true && info.drogue_flag == true && info.main_flag == false)
                    {
                        cout << "300 meters, deploy main!\n";
                        info.main_flag = true;
                    }
                }
            }
        }
}


void launch()
{
    Environment env;
    Program_Info info;
    // Barometer baro_1;
    // Barometer baro_2;
    Baro_Package baro_package;

    // IMU imu_1;
    // IMU imu_2;

    cout << "MECO & baro detection ... or just baro? 1 or 2 \n";
    cin >> info.answer;

    cout << "What window size for the barometers? (5 is a good default)\n";
    cin >> info.baro_size;
    //creating a detector for the barometers with a window of baro_size
    Detector detect(info.baro_size, true);

    if (info.answer == 1)
    {
        cout << "What window size for the IMUs? (5 is a good default)\n";
        cin >> info.imu_size;

        cout << "What detection method? Search for negative acceleration, or look for steep drop in acceleration? 1 or 2\n";
        cin >> info.detection_method;

        //creating a detector for the IMU with a window of imu_size
        Detector detect_IMU(info.imu_size, false);

        //read in data for IMUs to detect MECO
        std::ifstream FS_1("a_x_data_new.txt");
        std::vector<long double> a_x_data;
        string a_x;

        while (FS_1 >> a_x)
            a_x_data.push_back(std::stold(a_x));

        FS_1.close();

        //IMUs are armed from the ground. 
        //Once MECO is detected logic is passed off to the barometers after some predetermined time    
        //new code for MECO detection

        int i = 0;
        
        while(info.MECO_flag == false && i < a_x_data.size())
        // while(MECO_flag == false)
        {

            double current_a_x = a_x_data[i];

            //inject noise
            //need to determine how much noise to inject to make this realistic
            //that can be found in the IMU datasheet. 
            //for now, just using the same noise as the barometers
            double a_x_noisy_1 = (current_a_x + gaussian_random(0, 1.5)) / 9.8;
            double a_x_noisy_2 = (current_a_x + gaussian_random(0, 1.5)) / 9.8;

            if (detect_IMU.size_1 < 5 && detect_IMU.size_2 < 5)
                {
                    detect_IMU.insert(a_x_noisy_1, a_x_noisy_2, info);
                }
                else
                {
                    detect_IMU.insert(a_x_noisy_1, a_x_noisy_2, info);

                    cout << "Time: " << static_cast<double>(i) / 100 << " (s)\n";

                    if (detect_IMU.detect_MECO(info) && info.MECO_flag == false)
                    {
                        cout << "MECO Detected!\n";
                        cout << "Wait to drop below Mach 1...\n";
                        cout << "Beginning phase 2 with barometers\n";
                        info.MECO_flag = true;
                    }
                }
                //just using this index variable to cycle through 
                //the array data from the a_x_data.txt file
                ++i;
        }

        if (info.MECO_flag == false)
            cout << "MECO not detected!\n";

        // int j = i + 1000;
        // while(j < heights.size())
        // while(j < heights.size() && apogee_flag == false)

    }    

    // std::pair<int, int> baro1_D1_D2;
    // std::pair<int, int> baro2_D1_D2;

    //read in altidude data for barometers to detect apogee after MECO
    std::ifstream FS_2("height_data_new.txt");
    std::vector<long double> heights;
    string height;

    while (FS_2 >> height)
        heights.push_back(std::stold(height));

    FS_2.close();

    
    int j = 0;
    while(info.apogee_flag == false)
    {
        double current_height = heights[j];

        read_detect_decide(j, current_height, baro_package, info, detect, env);

        //using this index variable 'j' to cycle through the values in height_data.txt
        //Note 
        //MASTRAN data for some reason skips to sampling every 0.05 seconds instead of 0.01 seconds 
        //almost immediately after apogee. 
        //That's why I update the loop variable i += 1 instead of i += 5 after index 7722 (apogee)
        //In the AD system for the flight computer, we DONT wan't to do this.
        //Thus, the time column (column 4 is not accurate after apogee, so just count the number of steps 
        //until Apogee Detected is printed to the screen. Roughly 25 steps is 2.5 seconds.). 
        if(j < 7722)
            j += 5;
        else
            j += 1;
    }

    while(info.drogue_flag == false)
    {
        double current_height = heights[j];

        read_detect_decide(j, current_height, baro_package, info, detect, env);

        j += 5;
    }

    while(info.main_flag == false)
    {
        double current_height = heights[j];

        read_detect_decide(j, current_height, baro_package, info, detect, env);

        j+= 5;
    }

    for(; j < heights.size(); j += 5)
    {
        cout << heights[j] << "\n";
    }
    
    cout << "Flight is over!\n";
}

int main()
{

    /******For IMUs ******/
    //the output being printed as 
    //slope : ________ 
    //index of MASTRAN data: ______
    
    /******For barometers******/
    //the output being printed is 
    //height(m) baro_1 pressure (mbar) baro_2 pressure (mbar) time (s, only valid pre-apogee)
    //Below that I'm keeping track of the "slope" of the readings which should be consistently negative on the way up 
    //and consistently positive on the way down. Hovers around 0 at apogee. 

    launch();
    // Barometer bar;
    // bar.view_data();
}

