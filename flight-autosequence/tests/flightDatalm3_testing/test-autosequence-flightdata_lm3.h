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

// Set by main() before simulation starts
const char *g_csv_filename = NULL;
int g_start_row = 2090;

uint32_t getTime() {
    return simulated_time_ms;
}

void replace_comma_with_dot(char *str) {
    while (*str) {
        if (*str == ',') *str = '.';
        str++;
    }
}

int get_computed_data(float *computed_altitude, float *vert_accel, float *baro_speed, float *amb_temp_C) {
    static FILE *file = NULL;
    char line[1024];
    static int row_count = 0;

    if (file == NULL) {
        if (g_csv_filename == NULL) {
            printf("Error: g_csv_filename not set!\n");
            exit(1);
        }

        file = fopen(g_csv_filename, "r");
        if (file == NULL) {
            printf("Error: Could not open %s\n", g_csv_filename);
            exit(1);
        }

        for (int i = 0; i < g_start_row; i++) {
            if (fgets(line, sizeof(line), file) == NULL) {
                printf("Error: File has fewer lines than g_start_row!\n");
                exit(1);
            }
        }

        row_count = g_start_row;
    }

    row_count++;

    if (fgets(line, sizeof(line), file) != NULL) {
        replace_comma_with_dot(line);

        char *token = strtok(line, ";");
        int col_index = 0;

        while (token != NULL) {
            switch (col_index) {
                case 0:  simulated_time_ms  = (uint32_t)(atof(token)); break;
                case 3:  *computed_altitude = atof(token); break;
                case 7:  *vert_accel        = atof(token); break;
                case 10: *baro_speed        = atof(token); break;
                case 11: *amb_temp_C        = atof(token); break;
            }
            token = strtok(NULL, ";");
            col_index++;
        }
        return 1;
    }

    fclose(file);
    file = NULL;
    return 0;
}

uint32_t time_since(uint32_t timestamp) {
    return getTime() - timestamp;
}

uint8_t valves_open() {
    return 1;
}

int find_launch_row(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        printf("Error: Could not open %s to find launch row\n", filename);
        exit(1);
    }

    char line[1024];
    int row = 0;
    int prev_status = -1;

    while (fgets(line, sizeof(line), f)) {
        // Skip the first two header lines (sep=; and column names)
        if (row < 2) {
            row++;
            continue;
        }

        char tmp[1024];
        strncpy(tmp, line, sizeof(tmp));
        replace_comma_with_dot(tmp);

        char *token = strtok(tmp, ";");
        int col = 0;
        int status = prev_status;

        while (token != NULL) {
            if (col == 2) {
                status = atoi(token);
                break;
            }
            token = strtok(NULL, ";");
            col++;
        }

        if (prev_status == 3 && status == 4) {
            fclose(f);
            printf("Launch detected at row %d (status 3->4)\n", row);
            return row;
        }

        prev_status = status;
        row++;
    }

    fclose(f);
    printf("Warning: No 3->4 status transition found, defaulting to row 0\n");
    return 0;
}

#endif