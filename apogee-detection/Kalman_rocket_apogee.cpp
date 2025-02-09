#include <iostream>
#include <vector>
#include <array>

using namespace std;

// Constants
const double ALTITUDE_SIGMA = 15.0;
const double ACCELERATION_SIGMA = 6.0;
const double MODEL_SIGMA = 0.6;

// Variances
const double altitude_variance = ALTITUDE_SIGMA * ALTITUDE_SIGMA;
const double acceleration_variance = ACCELERATION_SIGMA * ACCELERATION_SIGMA;
const double model_variance = MODEL_SIGMA * MODEL_SIGMA;

// Convert accelerometer and gyroscope noise densities to variances
const double accel_noise_density = 120e-6;  //  ¶Ãg in g units
const double gyro_noise_density = 3.8e-3;    // 3.8 mdps in dps units
//¶Ãg/°ÃHz
//FS = °¿4 g: 120
//FS = °¿8 g: 130
//FS = °¿16 g: 160
//FS = °¿32 g: 220
//Acceleration RMS noise in normal/low-power mode
//mg(RMS)
//FS = °¿4 g 3.2
//FS = °¿8 g 3.4
//FS = °¿16 g 4.0
//FS = °¿32 g 5.4
// Sampling frequency (in Hz)
const double ODR = 100.0; // Replace with actual ODR used. 12.5 Hz up to 6.66 kHz 
// Calculate noise variances
const double accel_variance = (accel_noise_density * accel_noise_density) * ODR;
const double gyro_variance = (gyro_noise_density * gyro_noise_density) * ODR;

// Convert GPS noise densities to variances. NOTE HERE that GPS noise is non-zero-mean, non Gaussian
// Define variances based on NEO-M9N specifications
const double position_variance = 4.0;       // (meters)^2
const double velocity_variance = 0.0025;    // (meters/second)^2
const double acceleration_variance = 0.01;  // (meters/second^2)^2 (EXAMPLE estimate)
// Initialize the covariance matrix as a 3x3 matrix
std::array<std::array<double, 3>, 3> P = {{
    {position_variance, 0.0, 0.0},
    {0.0, velocity_variance, 0.0},
    {0.0, 0.0, acceleration_variance}
}};
// Velocity accuracy
// 0.05 m/s
//Dynamic heading accuracy
// 0.3 deg
// Position accuracy PVT   GPS: 2.0 m CEP


// Convert barometer noise densities to variances
const double Pressure_variance = 1.5;  //  mbar
const double Pressure_resolution = 0.012;    // mbar
// Pressure Accuracy 25°„C, 750 mbar: -1.5~+1.5 mbar
// Pressure resolution: 0.065 / 0.042 / 0.027 / 0.018 / 0.012 @ Oversampling Ratio: 256 / 512 / 1024 / 2048 / 4096
// Converting Pressure Noise to Altitude Noise:
Delta_h = Delta_P / (air_density*gravity_acceleration);		// Density and g values will change. But to initiallize, can use ground values...
// Use altitude noise to get covariance matrix
// Initialize the covariance matrix as a 3x3 matrix
std::array<std::array<double, 3>, 3> P = {{
    {altitude_variance, 0.0, 0.0},
    {0.0, velocity_variance, 0.0},
    {0.0, 0.0, acceleration_variance}
}};



// State and Covariance
array<double, 3> est = {0.0, 0.0, 0.0}; // Position, Velocity, Acceleration
//array<array<double, 3>, 3> pest = {{
//    {2.0, 0.0, 0.0},
//    {0.0, 9.0, 0.0},
//    {0.0, 0.0, 9.0}
//}};

// Initialize expanded 12x12 covariance matrix
std::array<std::array<double, 12>, 12> pest = {};
// Initialize position, velocity, acceleration, and angular rate variances
for (int i = 0; i < 3; ++i) {
    pest[i][i] = 1e3;               	 // Position variance (large initial uncertainty if unknown)
    pest[i + 3][i + 3] = accel_variance; // Velocity variance (based on accelerometer noise)
    pest[i + 6][i + 6] = accel_variance; // Acceleration variance (directly from accelerometer)
    pest[i + 9][i + 9] = gyro_variance;  // Angular rate variance (from gyroscope noise)
}


// State transition matrix and its transpose
array<array<double, 3>, 3> phi = {{
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
}};
array<array<double, 3>, 3> phit = phi; // Transpose of phi

// Kalman Gain
array<array<double, 2>, 3> kgain = {};

// Kalman Filter Step
void kalman_filter_step(double time, double accel, double pressure, double &last_time) {
    // Calculate time difference
    double dt = time - last_time;
    last_time = time;

    // Update state transition matrix
    phi[0][1] = dt;
    phi[1][2] = dt;
    phi[0][2] = dt * dt / 2.0;
    
    // Transpose of phi
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            phit[i][j] = phi[j][i];
        }
    }

    // Prediction Step
    array<array<double, 3>, 3> pestp = {}; // Temporary variable for predicted covariance
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                pestp[i][j] += phi[i][k] * pest[k][j];
            }
        }
    }
    pestp[2][2] += model_variance;

    // Kalman Gain Calculation
    double det = (pestp[0][0] + altitude_variance) * (pestp[2][2] + acceleration_variance) - pestp[2][0] * pestp[0][2];
    kgain[0][0] = (pestp[0][0] * (pestp[2][2] + acceleration_variance) - pestp[0][2] * pestp[2][0]) / det;
    kgain[0][1] = (pestp[0][0] * (-pestp[0][2]) + pestp[0][2] * (pestp[0][0] + altitude_variance)) / det;
    kgain[1][0] = (pestp[1][0] * (pestp[2][2] + acceleration_variance) - pestp[1][2] * pestp[2][0]) / det;
    kgain[1][1] = (pestp[1][0] * (-pestp[0][2]) + pestp[1][2] * (pestp[0][0] + altitude_variance)) / det;
    kgain[2][0] = (pestp[2][0] * (pestp[2][2] + acceleration_variance) - pestp[2][2] * pestp[2][0]) / det;
    kgain[2][1] = (pestp[2][0] * (-pestp[0][2]) + pestp[2][2] * (pestp[0][0] + altitude_variance)) / det;

    // Update covariance matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 2; ++k) {
                sum += kgain[i][k] * pestp[k][j];
            }
            pest[i][j] = pestp[i][j] - sum;
        }
    }

    // Update step
    double alt_innovation = pressure - est[0];
    double accel_innovation = accel - est[2];
    array<double, 2> innovation = {alt_innovation, accel_innovation};

    for (int i = 0; i < 3; ++i) {
        est[i] += kgain[i][0] * innovation[0] + kgain[i][1] * innovation[1];
    }
}

int main() {
    // Example data (replace these with actual sensor readings)
    vector<double> time_data = {0, 1, 2, 3, 4}; // Sample time data
    vector<double> accel_data = {0, 0.1, 0.2, 0.1, -0.1}; // Sample acceleration data
    vector<double> pressure_data = {100, 102, 104, 106, 108}; // Sample pressure data

    double last_time = time_data[0];
    for (size_t i = 0; i < time_data.size(); ++i) {
        kalman_filter_step(time_data[i], accel_data[i], pressure_data[i], last_time);
        cout << "Estimated Position: " << est[0] 
             << ", Estimated Velocity: " << est[1] 
             << ", Estimated Acceleration: " << est[2] << endl;
    }

    return 0;
}
