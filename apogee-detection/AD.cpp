/*The purpose of this script is to test the effectiveness of our apogee detection 
algorithm on SIMULATED DATA. This is not intended for use on hardware.
If you're looking for code to use on the board you want 'apogee-task.c' and 'apogee-task.h'
For this to work you need to have noise_generator.hpp and noise_generator.cpp in your directory.

Those files use a .txt that has the MASTRAN (MASA's in-house flight prediction software) 
data for the altitude of the rocket. The current .txt file in this repo 'height_data.txt' is what 
we used in 2025 and may be out-dated depending on when you access this.
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

class Detector
{
    public: 
    
    //buffer for readings from barometer 1
    static const int CAPACITY = 5;
    double readings_1[CAPACITY]; 
    int index_1 = 0;
    int size_1 = 0;

    //buffer for readings from barometer 2
    double readings_2[CAPACITY];
    int index_2 = 0;
    int size_2 = 0;

    //buffer for the average of the two barometers
    double average[5];
    int avg_index = 0;
    int avg_size = 0;

    //buffer for the 'slope' or how the average of the barometer readings 
    //changes over time
    double slope[5];
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
    void insert(double baro1, double baro2)
    {
        //this technique is called imputation! 
        //if we receive a bad reading, we don't want to corrupt the average.
        //to prevent this: if we detect nan, we take the mean of the readings 
        //CURRENTLY IN THE BUFFER, and insert that number into the place where 
        //NAN would have been inserted. 
        if(std::isnan(baro1)){
            baro1 = mean(size_1, readings_1);}
        readings_1[index_1] = baro1;
        index_1++;
        index_1 = index_1 % CAPACITY;
        size_1 = std::min(size_1 + 1, 5);

        //same as above!
        if(std::isnan(baro2)){
            baro2 = mean(size_2, readings_2);}
        readings_2[index_2] = baro2;
        index_2++;
        index_2 = index_2 % CAPACITY;
        size_2 = std::min(size_2 + 1, 5);

        average[avg_index] = (mean(size_1, readings_1) + mean(size_2, readings_2)) / 2;
        avg_index++;
        avg_index = avg_index % 5;
        avg_size = std::min(avg_size + 1, 5);

        //Wait until the 'average' buffer is full to start computing the slopes
        if(avg_size >= 5)
        {
            //Compare the MOST RECENT average reading to the reading from 5 STEPS ago
            //Our whole windows is size 5 so this affectively gives an estimate of the 
            //average change per step over our whole window
    
            //For example if our 'average' buffer is [2 3 5 9 8] and avg_index is currently 0.
            //This means the most recent reading will have been placed at avg_index 4 (becuase we've
            //now wrapped around to the start of our buffer by the time we reach this block) 

            //0 + 5 - 1 = 4 (most recent reading) average[4] = 8
            //0 + 5 - 5 = 0 (oldest reading) average[0] = 2
            //(8 - 2) / 5 = 1.2 
            //So we can estimate that our average change was roughly 1.2 per step over this window.
            slope[slope_i] = (average[(avg_index + 5 - 1) % 5 ] - average[(avg_index + 5 - 5) % 5]) / 5;

            std::cout << slope[slope_i] << "\n";
            slope_size = std::min(slope_size + 1, 5);
            slope_i = (slope_i + 1) % 5;
        }
    }

    

    bool detect_apog()
    {
        if (slope_size < 5)
            return false;
        else
        { 
            int count = 0;
            for(int i = 0; i < 5; ++i)
            {
                if(slope[i] > 0)
                    ++count;
            }

            //if 80% of the readings in our slope buffer are positive DETECT APOGEE
            if(count >= 4)
                return true;
        }
        return false;
    }
    
};


void launch()
{
    Environment env;
    Barometer baro_1;
    Barometer baro_2;
    Detector detect;
    bool apogee_flag = false;

    std::pair<int, int> baro1_D1_D2;
    std::pair<int, int> baro2_D1_D2;

    std::ifstream inFS("height_data.txt");
    std::vector<long double> heights;
    string height;

    while (inFS >> height)
        heights.push_back(std::stold(height));

    inFS.close();

    int i = 0;
    while(i < heights.size())
    {
        double current_height = heights[i];

        //13647
        // if (current_height > 13647)
        if (current_height > 18100)

        {
            double P_true = env.compute_pressure(current_height);
            double T_true = env.compute_temp(current_height);

            // inject noise
            double P_noisy_1 = P_true + gaussian_random(0, 1.5);
            double T_noisy_1 = T_true + gaussian_random(0, 8);

            double P_noisy_2 = P_true + gaussian_random(0, 1.5);
            double T_noisy_2 = T_true + gaussian_random(0, 8);

            baro1_D1_D2 = baro_1.guess_and_check(P_noisy_1, T_noisy_1);
            baro2_D1_D2 = baro_2.guess_and_check(P_noisy_2, T_noisy_2);

            auto [baro1_final_P, baro1_final_TEMP] = baro_1.forward_calculation(baro1_D1_D2.first, baro1_D1_D2.second);
            auto [baro2_final_P, baro2_final_TEMP] = baro_2.forward_calculation(baro2_D1_D2.first, baro2_D1_D2.second);

            cout << setw(10) << left << current_height;
            cout << setw(10) << baro1_final_P / 100 << setw(10) << baro2_final_P / 100;

            cout << setw(10) << right << static_cast<double>(i) / 100 << "\n";

            if (detect.size_1 < 5 && detect.size_2 < 5)
            {
                // detect.insert(std::round((baro1_final_P / 100) * 1000) / 1000 , std::round((baro2_final_P / 100) * 1000) / 1000);
                detect.insert(baro1_final_P / 100, baro2_final_P / 100);
            }
            else
            {
                // detect.insert(std::round((baro1_final_P / 100) * 1000) / 1000 , std::round((baro2_final_P / 100) * 1000) / 1000);
                detect.insert(baro1_final_P / 100, baro2_final_P / 100);
                if (detect.detect_apog() && apogee_flag == false)
                {
                    cout << "Apogee Detected\n";
                    apogee_flag = true;

                }
            }
        }

        if(i < 7722)
            i += 5;
        else
            i += 1;
    }
}

int main()
{
    //Note 
    //MASTRAN data for some reason skips to sampling every 0.05 seconds instead of 0.01 seconds 
    //almost immediately after apogee. 
    //That's why I update the loop variable i += 1 instead of i += 5 after index 7722 (apogee)
    //In the AD system for the flight computer, we DONT wan't to do this.
    //Thus, the time column (column 4 is not accurate after apogee, so just count the number of steps 
    //until Apogee Detected is printed to the screen. Roughly 25 steps is 2.5 seconds.). 

    //the output being printed is 
    //height(m) baro_1 pressure (mbar) baro_2 pressure (mbar) time (s, only valid pre-apogee)
    //Below that I'm keeping track of the "slope" of the readings which should be consistently negative on the way up 
    //and consistently positive on the way down. Hovers around 0 at apogee. 

    launch();
    // Barometer bar;
    // bar.view_data();
}

