// Source - https://stackoverflow.com/a/61078852
// Posted by fuentesj, modified by community. See post 'Timeline' for change history
// Retrieved 2026-04-11, License - CC BY-SA 4.0

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