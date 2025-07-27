#include "bsp_madgwick.h"
#include <string.h>

#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 快速平方根倒数
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 四元数归一化
static void quaternion_normalize(float q[4]) {
    float norm = inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;
}

// 初始化Madgwick滤波器
void bsp_madgwick_init(bsp_madgwick_t *filter) {
    // 初始化四元数 (单位四元数)
    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    
    // 初始化陀螺仪偏差
    memset(filter->gyro_offset, 0, sizeof(filter->gyro_offset));
    
    // 设置滤波器参数
    filter->beta = 0.1f;              // Madgwick增益
    filter->gyro_bias_alpha = 0.001f; // 偏差估计系数
    filter->static_threshold = 0.05f; // 静态检测阈值 (rad/s)
    
    // 初始化状态
    filter->static_count = 0;
    filter->dt = 0.001f;
    filter->is_initialized = 0;
    filter->is_static = 0;
}

// 陀螺仪校准
void bsp_madgwick_calibrate_gyro(bsp_madgwick_t *filter, float gyro[3], uint16_t samples) {
    static uint16_t sample_count = 0;
    static float gyro_sum[3] = {0};
    
    if (sample_count == 0) {
        // 重置累加器
        memset(gyro_sum, 0, sizeof(gyro_sum));
    }
    
    // 累加陀螺仪数据
    gyro_sum[0] += gyro[0];
    gyro_sum[1] += gyro[1];
    gyro_sum[2] += gyro[2];
    sample_count++;
    
    // 当采集到足够样本时计算平均值
    if (sample_count >= samples) {
        filter->gyro_offset[0] = gyro_sum[0] / samples;
        filter->gyro_offset[1] = gyro_sum[1] / samples;
        filter->gyro_offset[2] = gyro_sum[2] / samples;
        sample_count = 0;
    }
}

// 加速度计校准（设置初始姿态）
void bsp_madgwick_calibrate_accel(bsp_madgwick_t *filter, float accel[3]) {
    // 计算初始roll和pitch角
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2]));
    
    // 将欧拉角转换为四元数
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    
    filter->q[0] = cr * cp;  // w
    filter->q[1] = sr * cp;  // x
    filter->q[2] = cr * sp;  // y
    filter->q[3] = 0.0f;     // z (yaw = 0)
    
    quaternion_normalize(filter->q);
    filter->is_initialized = 1;
}

// 静态检测
static uint8_t detect_static(bsp_madgwick_t *filter, float gyro[3]) {
    float gyro_magnitude = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    
    if (gyro_magnitude < filter->static_threshold) {
        filter->static_count++;
        if (filter->static_count > 100) {  // 100个样本都静态
            filter->is_static = 1;
            return 1;
        }
    } else {
        filter->static_count = 0;
        filter->is_static = 0;
    }
    
    return filter->is_static;
}

// 更新Madgwick滤波器
void bsp_madgwick_update(bsp_madgwick_t *filter, float gyro[3], float accel[3], float dt) {
    if (!filter->is_initialized) {
        return;
    }
    
    filter->dt = dt;
    
    // 去除陀螺仪偏差
    float corrected_gyro[3];
    corrected_gyro[0] = gyro[0] - filter->gyro_offset[0];
    corrected_gyro[1] = gyro[1] - filter->gyro_offset[1];
    corrected_gyro[2] = gyro[2] - filter->gyro_offset[2];
    
    // 静态检测
    uint8_t is_static = detect_static(filter, corrected_gyro);
    
    // 四元数预测步骤（基于陀螺仪）
    float q0 = filter->q[0], q1 = filter->q[1], q2 = filter->q[2], q3 = filter->q[3];
    float gx = corrected_gyro[0], gy = corrected_gyro[1], gz = corrected_gyro[2];
    
    // 四元数微分方程
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    
    // 积分更新四元数
    filter->q[0] += qDot0 * dt;
    filter->q[1] += qDot1 * dt;
    filter->q[2] += qDot2 * dt;
    filter->q[3] += qDot3 * dt;
    
    // Madgwick校正（基于加速度计）
    float ax = accel[0], ay = accel[1], az = accel[2];
    
    // 归一化加速度计数据
    float norm = inv_sqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // 重力方向估计（从四元数计算）
    float _2q0 = 2.0f * filter->q[0];
    float _2q1 = 2.0f * filter->q[1];
    float _2q2 = 2.0f * filter->q[2];
    float _2q3 = 2.0f * filter->q[3];
    float _4q0 = 4.0f * filter->q[0];
    float _4q1 = 4.0f * filter->q[1];
    float _4q2 = 4.0f * filter->q[2];
    float _8q1 = 8.0f * filter->q[1];
    float _8q2 = 8.0f * filter->q[2];
    float q0q0 = filter->q[0] * filter->q[0];
    float q1q1 = filter->q[1] * filter->q[1];
    float q2q2 = filter->q[2] * filter->q[2];
    float q3q3 = filter->q[3] * filter->q[3];
    
    // 重力方向
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * filter->q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * filter->q[3] - _2q1 * ax + 4.0f * q2q2 * filter->q[3] - _2q2 * ay;
    
    // 归一化
    norm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    
    // 应用校正
    filter->q[0] -= filter->beta * s0;
    filter->q[1] -= filter->beta * s1;
    filter->q[2] -= filter->beta * s2;
    filter->q[3] -= filter->beta * s3;
    
    // 归一化四元数
    quaternion_normalize(filter->q);
    
    // 在静态状态下更新陀螺仪偏差估计
    if (is_static) {
        filter->gyro_offset[0] += filter->gyro_bias_alpha * gyro[0];
        filter->gyro_offset[1] += filter->gyro_bias_alpha * gyro[1];
        filter->gyro_offset[2] += filter->gyro_bias_alpha * gyro[2];
    }
}

// 获取欧拉角
void bsp_madgwick_get_angles(bsp_madgwick_t *filter, float *roll, float *pitch, float *yaw) {
    float q0 = filter->q[0], q1 = filter->q[1], q2 = filter->q[2], q3 = filter->q[3];
    
    // 四元数转欧拉角
    *roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD_TO_DEG;
    *pitch = asinf(2.0f * (q0*q2 - q3*q1)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * RAD_TO_DEG;
}
