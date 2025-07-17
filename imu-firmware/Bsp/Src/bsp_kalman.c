#include "bsp_kalman.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.295779513082323f

static void matrix_multiply(float *C, float *A, float *B, int m, int n, int p);
static void matrix_add(float *C, float *A, float *B, int m, int n);
static void matrix_transpose(float *AT, float *A, int m, int n);
static int matrix_inverse(float *A_inv, float *A, int n);
static void quaternion_multiply(float q_out[4], float q1[4], float q2[4]);

void bsp_kalman_init(bsp_kalman_t *kalman) {
    memset(kalman, 0, sizeof(bsp_kalman_t));
    
    // Initialize quaternion to identity
    kalman->q[0] = 1.0f;
    kalman->q[1] = 0.0f;
    kalman->q[2] = 0.0f;
    kalman->q[3] = 0.0f;
    
    // Initialize covariance matrix P (diagonal)
    for (int i = 0; i < KALMAN_DIM_X; i++) {
        for (int j = 0; j < KALMAN_DIM_X; j++) {
            if (i == j) {
                if (i < 4) {
                    kalman->P[i][j] = 1e-2f;  // Quaternion uncertainty
                } else {
                    kalman->P[i][j] = 1e-4f;  // Bias uncertainty
                }
            } else {
                kalman->P[i][j] = 0.0f;
            }
        }
    }
    
    // Process noise covariance Q
    for (int i = 0; i < KALMAN_DIM_X; i++) {
        for (int j = 0; j < KALMAN_DIM_X; j++) {
            if (i == j) {
                if (i < 4) {
                    kalman->Q[i][j] = 1e-6f;  // Reduced quaternion process noise
                } else {
                    kalman->Q[i][j] = 1e-9f;  // Reduced bias process noise
                }
            } else {
                kalman->Q[i][j] = 0.0f;
            }
        }
    }
    
    // Measurement noise covariance R
    for (int i = 0; i < KALMAN_DIM_Z; i++) {
        for (int j = 0; j < KALMAN_DIM_Z; j++) {
            if (i == j) {
                if (i < 3) {
                    kalman->R[i][j] = 1e-3f;  // Reduced accelerometer noise
                } else {
                    kalman->R[i][j] = 1e-2f;  // Magnetometer noise (if used)
                }
            } else {
                kalman->R[i][j] = 0.0f;
            }
        }
    }
    
    // Madgwick filter gain - increased for better drift compensation
    kalman->beta = 0.5f;
    
    // Static detection threshold (rad/s) - more sensitive
    kalman->static_threshold = 0.02f;
    
    // Initialize scale factors to 1.0
    for (int i = 0; i < 3; i++) {
        kalman->accel_scale[i] = 1.0f;
        kalman->gyro_scale[i] = 1.0f;
    }
}

void bsp_kalman_reset(bsp_kalman_t *kalman) {
    // Reset quaternion to identity
    kalman->q[0] = 1.0f;
    kalman->q[1] = 0.0f;
    kalman->q[2] = 0.0f;
    kalman->q[3] = 0.0f;
    
    // Reset biases
    memset(kalman->gyro_bias, 0, sizeof(kalman->gyro_bias));
    
    // Reset angles
    kalman->roll = 0.0f;
    kalman->pitch = 0.0f;
    kalman->yaw = 0.0f;
    
    // Reset counters
    kalman->update_count = 0;
    kalman->static_count = 0;
}

void bsp_kalman_update(bsp_kalman_t *kalman, float gyro[3], float accel[3], float dt) {
    kalman->dt = dt;
    kalman->update_count++;
    
    // Apply calibration
    float gyro_cal[3], accel_cal[3];
    for (int i = 0; i < 3; i++) {
        gyro_cal[i] = (gyro[i] - kalman->gyro_offset[i]) * kalman->gyro_scale[i];
        accel_cal[i] = (accel[i] - kalman->accel_offset[i]) * kalman->accel_scale[i];
    }
    
    // Update static detection first
    bsp_kalman_update_static_detection(kalman, gyro_cal, accel_cal);
    
    // Enhanced bias estimation
    if (kalman->static_count > 50) {  // Lowered threshold for faster response
        float alpha = 0.01f;  // Increased for faster convergence
        for (int i = 0; i < 3; i++) {
            kalman->gyro_bias[i] = kalman->gyro_bias[i] * (1.0f - alpha) + gyro_cal[i] * alpha;
        }
    }
    
    // Remove gyro bias
    float gyro_corrected[3];
    for (int i = 0; i < 3; i++) {
        gyro_corrected[i] = gyro_cal[i] - kalman->gyro_bias[i];
    }
    
    // Check accelerometer magnitude for validity
    float accel_mag = sqrtf(accel_cal[0]*accel_cal[0] + accel_cal[1]*accel_cal[1] + accel_cal[2]*accel_cal[2]);
    float accel_weight = 1.0f;
    
    // Reduce accelerometer weight if not close to 1g
    if (fabsf(accel_mag - 1.0f) > 0.2f) {
        accel_weight = 0.1f;
    }
    
    // Normalize accelerometer
    if (accel_mag > 0.0f) {
        accel_cal[0] /= accel_mag;
        accel_cal[1] /= accel_mag;
        accel_cal[2] /= accel_mag;
    }
    
    // State prediction using quaternion kinematics
    float q_dot[4];
    q_dot[0] = 0.5f * (-kalman->q[1]*gyro_corrected[0] - kalman->q[2]*gyro_corrected[1] - kalman->q[3]*gyro_corrected[2]);
    q_dot[1] = 0.5f * ( kalman->q[0]*gyro_corrected[0] + kalman->q[2]*gyro_corrected[2] - kalman->q[3]*gyro_corrected[1]);
    q_dot[2] = 0.5f * ( kalman->q[0]*gyro_corrected[1] - kalman->q[1]*gyro_corrected[2] + kalman->q[3]*gyro_corrected[0]);
    q_dot[3] = 0.5f * ( kalman->q[0]*gyro_corrected[2] + kalman->q[1]*gyro_corrected[1] - kalman->q[2]*gyro_corrected[0]);
    
    // Integrate quaternion
    kalman->q[0] += q_dot[0] * dt;
    kalman->q[1] += q_dot[1] * dt;
    kalman->q[2] += q_dot[2] * dt;
    kalman->q[3] += q_dot[3] * dt;
    
    // Normalize quaternion
    bsp_kalman_normalize_quaternion(kalman->q);
    
    // Gradient descent correction from accelerometer
    float q0 = kalman->q[0], q1 = kalman->q[1], q2 = kalman->q[2], q3 = kalman->q[3];
    
    // Gravity vector in body frame
    float g_x = 2.0f * (q1*q3 - q0*q2);
    float g_y = 2.0f * (q0*q1 + q2*q3);
    float g_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // Error between measured and estimated gravity
    float e_x = accel_cal[1]*g_z - accel_cal[2]*g_y;
    float e_y = accel_cal[2]*g_x - accel_cal[0]*g_z;
    float e_z = accel_cal[0]*g_y - accel_cal[1]*g_x;
    
    // Apply adaptive feedback
    float beta_adaptive = kalman->beta * accel_weight;
    if (kalman->static_count > 10) {
        beta_adaptive *= 2.0f;  // Increase gain when static
    }
    
    kalman->q[1] += beta_adaptive * e_x * dt;
    kalman->q[2] += beta_adaptive * e_y * dt;
    kalman->q[3] += beta_adaptive * e_z * dt;
    kalman->q[0] -= beta_adaptive * (q1*e_x + q2*e_y + q3*e_z) * dt;
    
    // Normalize quaternion again
    bsp_kalman_normalize_quaternion(kalman->q);
    
    // Convert to Euler angles
    bsp_kalman_quaternion_to_euler(kalman->q, &kalman->roll, &kalman->pitch, &kalman->yaw);
}

void bsp_kalman_get_angles(bsp_kalman_t *kalman, float *roll, float *pitch, float *yaw) {
    *roll = kalman->roll;
    *pitch = kalman->pitch;
    *yaw = kalman->yaw;
}

void bsp_kalman_calibrate_gyro(bsp_kalman_t *kalman, float gyro[3], uint32_t samples) {
    static float gyro_sum[3] = {0};
    static uint32_t sample_count = 0;
    
    if (sample_count < samples) {
        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];
        sample_count++;
    } else {
        kalman->gyro_offset[0] = gyro_sum[0] / samples;
        kalman->gyro_offset[1] = gyro_sum[1] / samples;
        kalman->gyro_offset[2] = gyro_sum[2] / samples;
        
        // Reset for next calibration
        gyro_sum[0] = 0;
        gyro_sum[1] = 0;
        gyro_sum[2] = 0;
        sample_count = 0;
    }
}

void bsp_kalman_calibrate_accel(bsp_kalman_t *kalman, float accel[3]) {
    // Simple calibration assuming device is level and Z-axis points up
    kalman->accel_offset[0] = accel[0];
    kalman->accel_offset[1] = accel[1];
    kalman->accel_offset[2] = accel[2] - 1.0f;  // Assuming 1g in Z direction
}

void bsp_kalman_update_static_detection(bsp_kalman_t *kalman, float gyro[3], float accel[3]) {
    // Check if device is static based on gyroscope magnitude
    float gyro_mag = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    
    // Also check accelerometer variance for better static detection
    static float accel_prev[3] = {0};
    float accel_diff = fabsf(accel[0] - accel_prev[0]) + 
                       fabsf(accel[1] - accel_prev[1]) + 
                       fabsf(accel[2] - accel_prev[2]);
    
    accel_prev[0] = accel[0];
    accel_prev[1] = accel[1];
    accel_prev[2] = accel[2];
    
    // Device is static if gyro is low AND accel is stable
    if (gyro_mag < kalman->static_threshold && accel_diff < 0.05f) {
        kalman->static_count++;
        if (kalman->static_count > 65535) {
            kalman->static_count = 65535;  // Prevent overflow
        }
    } else {
        kalman->static_count = 0;
    }
}

void bsp_kalman_quaternion_to_euler(float q[4], float *roll, float *pitch, float *yaw) {
    // Convert quaternion to Euler angles (ZYX convention)
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    *roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD_TO_DEG;
    
    float sin_pitch = 2.0f * (q0*q2 - q3*q1);
    if (fabsf(sin_pitch) >= 1.0f) {
        *pitch = copysignf(90.0f, sin_pitch);
    } else {
        *pitch = asinf(sin_pitch) * RAD_TO_DEG;
    }
    
    *yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * RAD_TO_DEG;
}

void bsp_kalman_normalize_quaternion(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

// Matrix operation helpers
static void matrix_multiply(float *C, float *A, float *B, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += A[i*n + k] * B[k*p + j];
            }
            C[i*p + j] = sum;
        }
    }
}

static void matrix_add(float *C, float *A, float *B, int m, int n) {
    for (int i = 0; i < m*n; i++) {
        C[i] = A[i] + B[i];
    }
}

static void matrix_transpose(float *AT, float *A, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            AT[j*m + i] = A[i*n + j];
        }
    }
}

static int matrix_inverse(float *A_inv, float *A, int n) {
    // Simple 2x2 or 3x3 matrix inversion
    // For larger matrices, use LU decomposition or similar
    // This is a placeholder - implement proper matrix inversion as needed
    return 0;
}

static void quaternion_multiply(float q_out[4], float q1[4], float q2[4]) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}