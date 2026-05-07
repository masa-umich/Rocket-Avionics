/*
 * sim_hardware.c
 *
 * Simulation back-end for execute_flight_autosequence().
 *
 * Data file format  (3 rows, comma-separated, ~10 000 values each):
 *   Row 1 : static pressure   [Pa]
 *   Row 2 : vertical accel    [m/s²]   (axial, +ve = up)
 *   Row 3 : time              [s]
 *
 * Both barometers receive the same pressure reading.
 * Both IMUs receive the same axial (XL_y) acceleration; all other axes = 0.
 * Temperature is not present in the data file; a constant 20 °C is returned
 * so the barometric altitude model has a valid ground-temperature baseline.
 */

#include "sim_hardware.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * User-tuneable constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Path to the 3-row data file. */
#ifndef SIM_DATA_FILE
#  define SIM_DATA_FILE "tests/system_tests/matlab_testing/data.txt"
#endif

/* Sim-time (ms) at which valves_open() first returns 1.
 * Set to match the index in your data where ignition occurs.
 * If your data starts at ignition (t=0), set this to 0.           */
#ifndef VALVES_OPEN_SIM_MS
#  define VALVES_OPEN_SIM_MS 0U
#endif

/* Fluctus broadcast times, in ms after ignition.
 * Update these once you know your flight profile from the data.    */
#ifndef FLUCTUS_APOGEE_FLIGHT_MS
#  define FLUCTUS_APOGEE_FLIGHT_MS  51900U   /* placeholder: 60 s  */
#endif
#ifndef FLUCTUS_5K_FLIGHT_MS
#  define FLUCTUS_5K_FLIGHT_MS     197000U   /* placeholder: 246 s */
#endif
#ifndef FLUCTUS_1K_FLIGHT_MS
#  define FLUCTUS_1K_FLIGHT_MS     254000U   /* placeholder: 295 s */
#endif

/* Constant temperature returned for both barometer sensors [°C].
 * Only used by the autosequence to set T_GROUND at ignition.       */
#define SIM_GROUND_TEMP_C  21.00f

/* Maximum number of samples that can be loaded from the file.
 * Increase if your dataset is larger than 50 000 points.           */
#define MAX_SAMPLES 50000

/* ═══════════════════════════════════════════════════════════════════════════
 * Internal state
 * ═══════════════════════════════════════════════════════════════════════════ */
static float    s_pressure[MAX_SAMPLES];   /* Pa  */
static float    s_accel   [MAX_SAMPLES];   /* m/s² */
static float    s_time_s  [MAX_SAMPLES];   /* s   */
static int      s_num_samples  = 0;
static int      s_sample_idx   = 0;
static int      s_data_loaded  = 0;

static uint32_t s_sim_time_ms  = 0;
static uint32_t s_ignition_ms  = 0;
static int      s_ignited       = 0;

static int      s_pilot_fired   = 0;
static int      s_drogue_fired  = 0;
static int      s_main_fired    = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * File loading
 * Reads one row at a time: splits on commas, converts each token to float.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int parse_row(FILE *f, float *out, int max_vals)
{
    /* Large static buffer: 25 chars per value × 50 000 values = 1.25 MB    */
    static char line[MAX_SAMPLES * 25];

    if (!fgets(line, sizeof(line), f)) return 0;

    int count = 0;
    char *ptr = line;
    while (*ptr && count < max_vals) {
        /* skip whitespace / commas before the number */
        while (*ptr == ' ' || *ptr == '\t' || *ptr == ',') ptr++;
        if (*ptr == '\0' || *ptr == '\n' || *ptr == '\r') break;

        char *end;
        out[count++] = (float)strtod(ptr, &end);
        if (end == ptr) break;   /* no conversion → stop */
        ptr = end;
    }
    return count;
}

static void load_data(void)
{
    if (s_data_loaded) return;

    FILE *f = fopen(SIM_DATA_FILE, "r");
    if (!f) {
        fprintf(stderr, "[SIM] ERROR: cannot open data file '%s'\n", SIM_DATA_FILE);
        exit(EXIT_FAILURE);
    }

    static float row_pressure[MAX_SAMPLES];
    static float row_accel   [MAX_SAMPLES];
    static float row_time    [MAX_SAMPLES];

    int n_p = parse_row(f, row_pressure, MAX_SAMPLES);
    int n_a = parse_row(f, row_accel,    MAX_SAMPLES);
    int n_t = parse_row(f, row_time,     MAX_SAMPLES);
    fclose(f);

    if (n_p == 0 || n_a == 0 || n_t == 0) {
        fprintf(stderr, "[SIM] ERROR: could not parse 3 rows from '%s'\n", SIM_DATA_FILE);
        exit(EXIT_FAILURE);
    }
    if (!(n_p == n_a && n_a == n_t)) {
        fprintf(stderr,
            "[SIM] WARNING: row lengths differ (pressure=%d, accel=%d, time=%d). "
            "Using shortest.\n", n_p, n_a, n_t);
    }

    s_num_samples = n_p < n_a ? n_p : n_a;
    s_num_samples = s_num_samples < n_t ? s_num_samples : n_t;

    memcpy(s_pressure, row_pressure, s_num_samples * sizeof(float));
    memcpy(s_accel,    row_accel,    s_num_samples * sizeof(float));
    memcpy(s_time_s,   row_time,     s_num_samples * sizeof(float));

    s_data_loaded = 1;

    printf("[SIM] Loaded %d samples | "
           "t=[%.3f, %.3f] s | "
           "P=[%.1f, %.1f] Pa | "
           "a=[%.2f, %.2f] m/s²\n",
           s_num_samples,
           s_time_s[0], s_time_s[s_num_samples - 1],
           s_pressure[0], s_pressure[s_num_samples - 1],
           s_accel[0],    s_accel[s_num_samples - 1]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Time
 * ═══════════════════════════════════════════════════════════════════════════ */
uint32_t getTime(void)
{
    return s_sim_time_ms;
}

uint32_t time_since(uint32_t timestamp)
{
    return s_sim_time_ms - timestamp;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Sensor data  – advances one sample per call
 * ═══════════════════════════════════════════════════════════════════════════ */
void get_sensor_data(float *bar1_hPa, float *bar2_hPa,
                     IMU_values *imu1, IMU_values *imu2,
                     float *bar1_temp_C, float *bar2_temp_C)
{
    load_data();

    if (s_sample_idx >= s_num_samples) {
        fprintf(stderr, "[SIM] WARNING: data exhausted – holding last sample\n");
        s_sample_idx = s_num_samples - 1;
    }

    /* Convert time from seconds to milliseconds for the sim clock. */
    s_sim_time_ms = (uint32_t)(s_time_s[s_sample_idx] * 1000.0f + 0.5f);

    /* Pressure: your data is in Pa; autosequence expects hPa → divide by 100. */
    float p_hPa = s_pressure[s_sample_idx] / 100.0f;
    *bar1_hPa = p_hPa;
    *bar2_hPa = p_hPa;

    /* Temperature: constant 20 °C (not in source data). */
    *bar1_temp_C = SIM_GROUND_TEMP_C;
    *bar2_temp_C = SIM_GROUND_TEMP_C;

    /* Acceleration: axial (y-axis) only; lateral axes = 0. */
    float ax = s_accel[s_sample_idx];
    imu1->XL_x = 0.0f; imu1->XL_y = ax; imu1->XL_z = 0.0f;
    imu1->W_x  = 0.0f; imu1->W_y  = 0.0f; imu1->W_z = 0.0f;

    imu2->XL_x = 0.0f; imu2->XL_y = ax; imu2->XL_z = 0.0f;
    imu2->W_x  = 0.0f; imu2->W_y  = 0.0f; imu2->W_z = 0.0f;

    s_sample_idx+= 2;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Abort / arming
 * ═══════════════════════════════════════════════════════════════════════════ */
int should_abort(void)
{
    return 0;
}

int valves_open(void)
{
    if (s_sim_time_ms >= VALVES_OPEN_SIM_MS) {
        if (!s_ignited) {
            s_ignited     = 1;
            s_ignition_ms = s_sim_time_ms;
            printf("[SIM] t=%6.3f s │ IGNITION – valves open\n",
                   s_sim_time_ms / 1000.0);
        }
        return 1;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Deployment channels
 * ═══════════════════════════════════════════════════════════════════════════ */
void deployPilot(void)
{
    if (!s_pilot_fired) {
        s_pilot_fired = 1;
        printf("[SIM] t=%6.3f s │ *** PILOT CHUTE DEPLOYED ***\n",
               s_sim_time_ms / 1000.0);
    }
}

void deployDrogue(void)
{
    if (!s_drogue_fired) {
        s_drogue_fired = 1;
        printf("[SIM] t=%6.3f s │ *** DROGUE DEPLOYED ***\n",
               s_sim_time_ms / 1000.0);
    }
}

void deployMain(void)
{
    if (!s_main_fired) {
        s_main_fired = 1;
        printf("[SIM] t=%6.3f s │ *** MAIN CHUTE DEPLOYED ***\n",
               s_sim_time_ms / 1000.0);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Fluctus FC discrete outputs
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint32_t flight_ms(void)
{
    if (!s_ignited) return 0U;
    return s_sim_time_ms - s_ignition_ms;
}

uint8_t get_fluctus_apogee(void)
{
    return (flight_ms() >= FLUCTUS_APOGEE_FLIGHT_MS) ? 1U : 0U;
}

uint8_t get_fluctus_5k(void)
{
    return (flight_ms() >= FLUCTUS_5K_FLIGHT_MS) ? 1U : 0U;
}

uint8_t get_fluctus_1k(void)
{
    return (flight_ms() >= FLUCTUS_1K_FLIGHT_MS) ? 1U : 0U;
}