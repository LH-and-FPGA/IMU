#ifndef BSP_KALMAN_H
#define BSP_KALMAN_H

#include "stdint.h"
#include "math.h"

// 卡尔曼滤波器状态结构体
typedef struct {
    // 四元数状态 [q0, q1, q2, q3]
    float q[4];
    
    // 陀螺仪偏差 [wx_bias, wy_bias, wz_bias]
    float gyro_offset[3];
    
    // 滤波器参数
    float beta;              // Madgwick滤波器增益
    float gyro_bias_alpha;   // 陀螺仪偏差估计系数
    
    // 静态检测
    uint16_t static_count;   // 静态计数器
    float static_threshold;  // 静态检测阈值
    
    // 时间相关
    float dt;               // 时间间隔
    
    // 状态标志
    uint8_t is_initialized;
    uint8_t is_static;
} bsp_kalman_t;

// 函数声明
void bsp_kalman_init(bsp_kalman_t *filter);
void bsp_kalman_calibrate_gyro(bsp_kalman_t *filter, float gyro[3], uint16_t samples);
void bsp_kalman_calibrate_accel(bsp_kalman_t *filter, float accel[3]);
void bsp_kalman_update(bsp_kalman_t *filter, float gyro[3], float accel[3], float dt);
void bsp_kalman_get_angles(bsp_kalman_t *filter, float *roll, float *pitch, float *yaw);

#endif // BSP_KALMAN_H
