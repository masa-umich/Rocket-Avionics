
#ifndef TEST_AUTOSEQUENCE_FLIGHTDATA_LM3_H
#define TEST_AUTOSEQUENCE_FLIGHTDATA_LM3_H

#include <stdlib.h>
#include <stdint.h>
#include "ad-functions-test.h"

#define AUTOSEQUENCE_DEBUG 0
#include <stdio.h>
#include <string.h>
#include <time.h>

uint32_t simulated_time_ms = 0; 

uint32_t getTime() {
    return simulated_time_ms;
}

// Helper function to swap European commas with standard periods for float parsing
void replace_comma_with_dot(char *str) {
    while (*str) {
        if (*str == ',') {
            *str = '.';
        }
        str++;
    }
}

// Function returns 1 if a line was successfully read, and 0 if we hit the End of File (EOF)
int get_computed_data(float *computed_altitude, float *vert_accel, float *baro_speed, float *amb_temp_C) {
    // Keep the file pointer open across multiple function calls
    static FILE *file = NULL;
    char line[1024];
    static int row_count = 0; // Moved this up so we can sync it

    // Set exactly which line you want to start reading from
    const int START_ROW = 2090; 

    // 1. Open the file on the very first call
    if (file == NULL) {
        file = fopen("flightData_lm3.csv", "r");
        if (file == NULL) {
            printf("Error: Could not open flightData_lm3.csv\n");
            exit(1); 
        }
        
        // Fast-forward through the file to skip the pad data
        // (This also automatically skips your 2 header lines!)
        for (int i = 0; i < START_ROW; i++) {
            if (fgets(line, sizeof(line), file) == NULL) {
                printf("Error: File has fewer lines than START_ROW!\n");
                exit(1);
            }
        }
        
        // Sync our row tracking to match where we skipped to
        row_count = START_ROW; 
    }

    row_count++;
    
    // 2. Read the next line of data
    if (fgets(line, sizeof(line), file) != NULL) {
        
        // Swap all ',' to '.' so atof() can convert them to floats
        replace_comma_with_dot(line);

        // 3. Tokenize the string by semicolons ';'
        char *token = strtok(line, ";");
        int col_index = 0;

        // Loop through the columns and grab the specific indices we need
        while (token != NULL) {
            switch (col_index) {
                case 0: // Time (assuming column 0)
                    simulated_time_ms = (uint32_t)(atof(token));
                    break;
                case 3: // baro-altitude (m)
                    *computed_altitude = atof(token);
                    break;
                case 7: // vert-accel (m/s2)
                    *vert_accel = atof(token);
                    break;
                case 10: // baro-speed (m/s)
                    *baro_speed = atof(token);
                    break;
                case 11: // amb-temp (deg c)
                    *amb_temp_C = atof(token);
                    break;
            }
            token = strtok(NULL, ";");
            col_index++;
        }
        
        // Updated printf to include the time so you can verify it's working!
        if (row_count % 100 == 0) {
            printf("Time: %u ms | Row %d: Alt=%.2f m, Vert Accel=%.2f m/s^2, Baro Speed=%.2f m/s, Amb Temp=%.2f C\n", 
                simulated_time_ms, row_count, *computed_altitude, *vert_accel, *baro_speed, *amb_temp_C);
        }
        
        return 1; // Successfully read the line
    }

    // 4. End of file reached
    fclose(file);
    file = NULL; // Reset in case you want to loop the simulation again
    return 0; // Return 0 to tell the main loop to stop
}

// Returns the milliseconds elapsed since a given timestamp
uint32_t time_since(uint32_t timestamp) {
    return getTime() - timestamp;
}

// Mocks the valve state. In a simulation, we return 1 (true) to kick off the sequence immediately.
uint8_t valves_open() {
    return 1; 
}

#endif