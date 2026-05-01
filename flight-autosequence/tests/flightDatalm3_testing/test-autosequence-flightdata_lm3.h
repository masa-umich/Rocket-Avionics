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
int g_start_row = 0;  // Only meaningful for CSV files; set to 0 for PF2

// ---------------------------------------------------------------------------
// File type detection
// ---------------------------------------------------------------------------

typedef enum {
    FILE_TYPE_CSV,
    FILE_TYPE_PF2,
    FILE_TYPE_UNKNOWN
} FileType;

static FileType detect_file_type(const char *filename) {
    const char *ext = strrchr(filename, '.');
    if (ext == NULL) return FILE_TYPE_UNKNOWN;
    // Case-insensitive compare for .pf2 / .csv
    char ext_lower[16];
    strncpy(ext_lower, ext, sizeof(ext_lower) - 1);
    ext_lower[sizeof(ext_lower) - 1] = '\0';
    for (int i = 0; ext_lower[i]; i++) {
        if (ext_lower[i] >= 'A' && ext_lower[i] <= 'Z')
            ext_lower[i] += 32;
    }
    if (strcmp(ext_lower, ".pf2") == 0) return FILE_TYPE_PF2;
    if (strcmp(ext_lower, ".csv") == 0) return FILE_TYPE_CSV;
    return FILE_TYPE_UNKNOWN;
}

// ---------------------------------------------------------------------------
// Unit conversion helpers
// ---------------------------------------------------------------------------

static float feet_to_meters(float ft)           { return ft * 0.3048f; }
static float fahrenheit_to_celsius(float f)     { return (f - 32.0f) * (5.0f / 9.0f); }

// ---------------------------------------------------------------------------
// Shared utilities
// ---------------------------------------------------------------------------

uint32_t getTime() {
    return simulated_time_ms;
}

void replace_comma_with_dot(char *str) {
    while (*str) {
        if (*str == ',') *str = '.';
        str++;
    }
}

// ---------------------------------------------------------------------------
// PF2 helpers
// ---------------------------------------------------------------------------

// Advances `file` past the "Data:" header line so the next fgets() call
// returns the first actual data row.
static void pf2_skip_to_data(FILE *file) {
    char line[1024];
    while (fgets(line, sizeof(line), file) != NULL) {
        if (strncmp(line, "Data:", 5) == 0)
            return;  // Next read will be the first data row
    }
    printf("Error: PF2 file has no 'Data:' section!\n");
    exit(1);
}

// ---------------------------------------------------------------------------
// get_computed_data — reads one row from the open flight data file
// ---------------------------------------------------------------------------

int get_computed_data(float *computed_altitude, float *vert_accel,
                      float *baro_speed, float *amb_temp_C) {
    static FILE     *file      = NULL;
    static FileType  file_type = FILE_TYPE_UNKNOWN;
    static int       row_count = 0;
    char line[1024];

    // ---- First call: open file and seek to starting row ----
    if (file == NULL) {
        if (g_csv_filename == NULL) {
            printf("Error: g_csv_filename not set!\n");
            exit(1);
        }

        file_type = detect_file_type(g_csv_filename);
        file = fopen(g_csv_filename, "r");
        if (file == NULL) {
            printf("Error: Could not open %s\n", g_csv_filename);
            exit(1);
        }

        if (file_type == FILE_TYPE_PF2) {
            // Skip the metadata header; position just before the first data row
            pf2_skip_to_data(file);
            // Optionally skip g_start_row rows into the data section
            for (int i = 0; i < g_start_row; i++) {
                if (fgets(line, sizeof(line), file) == NULL) {
                    printf("Error: PF2 file has fewer data rows than g_start_row!\n");
                    exit(1);
                }
            }
        } else {
            // CSV: skip g_start_row rows from the top
            for (int i = 0; i < g_start_row; i++) {
                if (fgets(line, sizeof(line), file) == NULL) {
                    printf("Error: CSV file has fewer lines than g_start_row!\n");
                    exit(1);
                }
            }
        }

        row_count = g_start_row;
    }

    row_count++;

    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        file = NULL;
        return 0;
    }

    // ---- PF2 parsing ----
    if (file_type == FILE_TYPE_PF2) {
        float time_s, alt_ft, vel_fps, temp_f, voltage;
        // Format: "0.00, -2, 0, 65.86, 8.30"
        int parsed = sscanf(line, "%f , %f , %f , %f , %f",
                            &time_s, &alt_ft, &vel_fps, &temp_f, &voltage);
        if (parsed < 4) {
            // Malformed or trailing blank line — treat as end of data
            fclose(file);
            file = NULL;
            return 0;
        }

        simulated_time_ms  = (uint32_t)(time_s * 1000.0f);
        *computed_altitude = feet_to_meters(alt_ft);
        *baro_speed        = feet_to_meters(vel_fps);
        *amb_temp_C        = fahrenheit_to_celsius(temp_f);
        *vert_accel        = 5.0f;  // PF2 has no acceleration data

        //fprintf(stderr, "Row %d: time=%ums alt=%.2fm vel=%.2fm/s temp=%.2fC\n",
        //        row_count, simulated_time_ms, *computed_altitude, *baro_speed, *amb_temp_C);
        return 1;
    }

    // ---- CSV parsing (original behaviour) ----
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

// ---------------------------------------------------------------------------
// Misc helpers
// ---------------------------------------------------------------------------

uint32_t time_since(uint32_t timestamp) {
    return getTime() - timestamp;
}

uint8_t valves_open() {
    return 1;
}

// ---------------------------------------------------------------------------
// find_launch_row
// ---------------------------------------------------------------------------

int find_launch_row(const char *filename) {
    FileType ft = detect_file_type(filename);

    if (ft == FILE_TYPE_PF2) {
        // PF2 has no status column; the entire data section is the flight.
        printf("PF2 file: no launch detection needed, starting from row 0\n");
        return 0;
    }

    // CSV: original status 3->4 transition detection
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