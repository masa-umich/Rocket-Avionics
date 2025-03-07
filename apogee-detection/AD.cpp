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

struct Detector
{
    double slope[10];
    int slope_i = 0;
    int slope_size = 0;

    static const int CAPACITY = 20;
    double readings_1[CAPACITY]; 
    int index_1 = 0;
    int size_1 = 0;

    double readings_2[CAPACITY];
    int index_2 = 0;
    int size_2 = 0;

    double previous[10];
    int prev_index = 0;
    int prev_size = 0;

    double mean(int size, double * arr)
    {
        if (size == 0)
            return 0;
        double sum = std::accumulate(arr, arr + size, 0);
        return sum / size;
    }

    void insert(double baro1, double baro2)
    {
        if(std::isnan(baro1))
            baro1 = mean(size_1, readings_1);
        readings_1[index_1] = baro1;
        index_1++;
        index_1 = index_1 % CAPACITY;
        size_1 = std::min(size_1 + 1, 20);


        if(std::isnan(baro2))
            baro2 = mean(size_2, readings_2);
        readings_2[index_2] = baro2;
        index_2++;
        index_2 = index_2 % CAPACITY;
        size_2 = std::min(size_2 + 1, 20);

        previous[prev_index] = (mean(size_1, readings_1) + mean(size_2, readings_2)) / 2;
        prev_index++;
        prev_index = prev_index % 10;
        prev_size = std::min(prev_size + 1, 10);

        if(prev_size >= 10)
        {
            slope[slope_i] = (previous[(prev_index + 10 - 1) % 10 ] - previous[(prev_index + 10 - 10) % 10]) / 10;
            std::cout << slope[slope_i] << "\n";
            slope_size = std::min(slope_size + 1, 10);
            slope_i = (slope_i + 1) % 10;
        }
    }

    bool detect_apog()
    {
        // if(prev_size < 5)
        if (slope_size < 10)
            return false;
        else
        { 
            int count = 0;
            for(int i = 0; i < 10; ++i)
            {
                if(slope[i] > 0)
                    ++count;
            }

            if(count >= 8)
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
    // int apogee_delay = 0;

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
    // for (size_t i = 0; i < heights.size(); i += 5)
    {
        double current_height = heights[i];

        //13647
        if (current_height > 13647)
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

            if (detect.size_1 < 20 && detect.size_2 < 20)
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
    //Below I'm keeping track of the "slope" of the readings which should be consistently neg on the way up 
    //and consistently pos on the way down. Hovers around 0 at apogee. 

    //this detector currently smooths the data a little too much for my liking, we are consistently 
    //getting detections at around 2.5 seconds post apogee, which is acceptable, and we have very little 
    //risk of early detection but I'd like to shave that down to about 1 second post apogee. 

    launch();
    // Barometer bar;
    // bar.view_data();
}

