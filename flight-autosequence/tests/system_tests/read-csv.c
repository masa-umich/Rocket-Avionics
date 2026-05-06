// Source - https://stackoverflow.com/a/61078852
// Posted by fuentesj, modified by community. See post 'Timeline' for change history
// Retrieved 2026-04-11, License - CC BY-SA 4.0
/*
#include <stdio.h>  // file handling functions
#include <stdlib.h> // atoi
#include <string.h> // strtok
#define DATA_SIZE 500
double div1000000(double n);
int main(){
    char buffer[2024];
    FILE* f = fopen("target_flight_data-apogee_12000.0.csv", "r");
    if (f == NULL) {
    perror("Error opening file");
    return 1;
}
static float bar1[DATA_SIZE];
static float bar2[DATA_SIZE];
static float imu1[DATA_SIZE];
static float imu2[DATA_SIZE];

double prev = 0.0;
int count = 0;
    while (fgets(buffer, 2024, f)) {
        int column = 0;
        // If you need all the values in a row
        char *token = strtok(buffer, ",");
        while (token) { 
            // Just printing each integer here but handle as needed
            if(column == 0) {
                double raw = atof(token);
                double delta = div1000000(raw) - div1000000(prev);
                prev = raw;
                printf("%f\n", delta);

            }
            if(column == 17) {
                float value = atof(token);
                imu1[count] = value;
                printf("%f\n", imu1[count]);

            }
            if(column == 23) {
                float value = atof(token);
                imu2[count] = value;
                printf("%f\n", imu2[count]);

            }
            if(column == 31) {
                float value = atof(token);
                bar1[count] = value;
                printf("%f\n", bar1[count]);

            }
            if(column == 32) {
                float value = atof(token);
                bar2[count] = value;
                printf("%f\n", bar2[count]);

            }
            column++;

            token = strtok(NULL, ",");
        }
        count++;
    }
    fclose(f);


}
double div1000000(double n) {
    return n / 1000000.0;
}
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LENGTH 4096

// User defined IMU struct
typedef struct {
    float XL_x; // accel in x direction
    float XL_y; // y direction is axial to the rocket - positive points from engine to nose cone (upwards)
    float XL_z;
    float W_x;  // angular velocity around x axis
    float W_y;
    float W_z;
} IMU_values;

// Struct to hold all the parsed data for a single row
typedef struct {
    long long timestamp;
    IMU_values imu1;
    IMU_values imu2;
    float baro1;
    float baro2;
} TelemetryData;

int main() {
    // Open the CSV file (Replace "telemetry.csv" with your actual file name)
    FILE *file = fopen("flight-autosequence/tests/telemetry_lm3.csv", "r");
    if (file == NULL) {
        printf("Error: Could not open file.\n");
        return 1;
    }

    char line[MAX_LINE_LENGTH];
    int row_count = 0;

    // Read and discard the header line
    if (fgets(line, sizeof(line), file) == NULL) {
        printf("File is empty or could not read header.\n");
        fclose(file);
        return 1;
    }

    // Read data line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        TelemetryData data = {0};
        int col = 0;

        // Use strtok to split the line by commas
        char *token = strtok(line, ",");

        while (token != NULL) {
            // Check the column index and parse the necessary fields
            switch (col) {
                case 0:  data.timestamp = atoll(token); break;
                
                // --- IMU 1 ---
                case 15: data.imu1.XL_x = atof(token); break;
                case 16: data.imu1.XL_y = atof(token); break;
                case 17: data.imu1.XL_z = atof(token); break;
                case 18: data.imu1.W_x  = atof(token); break; // wp mapped to W_x
                case 19: data.imu1.W_y  = atof(token); break; // wr mapped to W_y
                case 20: data.imu1.W_z  = atof(token); break; // wy mapped to W_z

                // --- IMU 2 ---
                case 21: data.imu2.XL_x = atof(token); break;
                case 22: data.imu2.XL_y = atof(token); break;
                case 23: data.imu2.XL_z = atof(token); break;
                case 24: data.imu2.W_x  = atof(token); break; // wp mapped to W_x
                case 25: data.imu2.W_y  = atof(token); break; // wr mapped to W_y
                case 26: data.imu2.W_z  = atof(token); break; // wy mapped to W_z

                // --- Barometers ---
                case 31: data.baro1 = atof(token); break;
                case 32: data.baro2 = atof(token); break;
            }
            
            token = strtok(NULL, ",");
            col++;
        }

        row_count++;

        // Print the parsed data for demonstration
        printf("Row %d [Time: %lld]:\n", row_count, data.timestamp);
        printf("  IMU1 Accel(y): %.3f, Baro1: %.2f hPa\n", data.imu1.XL_y, data.baro1);
        
        // If you only want to process a few rows to test, you can break early:
        // if(row_count >= 5) break; 
    }

    fclose(file);
    printf("\nSuccessfully parsed %d rows of telemetry data.\n", row_count);

    return 0;
}