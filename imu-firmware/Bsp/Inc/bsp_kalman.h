#ifndef BSP_KALMAN_H
#define BSP_KALMAN_H

#include "stdint.h"
#include "main.h"

#define KALMAN_DIM_X 7  // State dimension: [q0, q1, q2, q3, wx_bias, wy_bias, wz_bias]
#define KALMAN_DIM_Z 6  // Measurement dimension: [ax, ay, az, mx, my, mz]

typedef struct {
    float q[4];           // Quaternion [q0, q1, q2, q3]
    float gyro_bias[3];   // Gyroscope bias [wx_bias, wy_bias, wz_bias]
    float P[KALMAN_DIM_X][KALMAN_DIM_X];  // Covariance matrix
    float Q[KALMAN_DIM_X][KALMAN_DIM_X];  // Process noise covariance
    float R[KALMAN_DIM_Z][KALMAN_DIM_Z];  // Measurement noise covariance
    
    float dt;             // Time step
    float beta;           // Madgwick filter gain
    
    // Calibration parameters
    float accel_offset[3];
    float accel_scale[3];
    float gyro_offset[3];
    float gyro_scale[3];
    
    // Zero drift compensation
    float static_threshold;
    uint32_t static_count;
    float gyro_drift_rate[3];
    
    // Output angles
    float roll;
    float pitch;
    float yaw;
    
    // Debug info
    uint32_t update_count;
    float computation_time;
} bsp_kalman_t;

// Initialization
void bsp_kalman_init(bsp_kalman_t *kalman);
void bsp_kalman_reset(bsp_kalman_t *kalman);

// Main update function
void bsp_kalman_update(bsp_kalman_t *kalman, float gyro[3], float accel[3], float dt);

// Get Euler angles
void bsp_kalman_get_angles(bsp_kalman_t *kalman, float *roll, float *pitch, float *yaw);

// Calibration functions
void bsp_kalman_calibrate_gyro(bsp_kalman_t *kalman, float gyro[3], uint32_t samples);
void bsp_kalman_calibrate_accel(bsp_kalman_t *kalman, float accel[3]);

// Zero drift compensation
void bsp_kalman_update_static_detection(bsp_kalman_t *kalman, float gyro[3], float accel[3]);

// Helper functions
void bsp_kalman_quaternion_to_euler(float q[4], float *roll, float *pitch, float *yaw);
void bsp_kalman_normalize_quaternion(float q[4]);

#endif // BSP_KALMAN_H