#ifndef __SERVO_MOTOR_DRIVER_H
#define __SERVO_MOTOR_DRIVER_H

#include "Robotic_Arm_Config.h"
#include "tim.h"

/* ==================== 舵机参数定义 ==================== */

/* 国华 A009 舵机：与小臂相连
 * 20ms 时基，0.5ms ~ 2.5ms 对应 0° ~ 180°
 * 机械安装：0.6rad 时舵机与小臂在一条线上（φ_servo = 0）
 * 接口角度定义：Motor_Position 直接等于 φ_servo（末端相对小臂延长线的偏角）
 *   接口 0       → 末端在小臂延长线上（φ_servo = 0）
 *   接口 π/2     → 末端垂直于小臂向上（φ_servo = π/2）
 *   接口 < 0     → 末端向内弯折
 */
#define A009_PWM_PULSE_MIN_US       500u    /* 0.5 ms */
#define A009_PWM_PULSE_MAX_US       2500u   /* 2.5 ms */
#define A009_ANGLE_OFFSET_RAD       0.6f    /* 偏置：实际舵机角度 = 接口角度 + 0.6 */
#define A009_ANGLE_MIN_RAD          0.0f    /* 实际机械最小角 0° */
#define A009_ANGLE_MAX_RAD          (PI)    /* 实际机械最大角 180° */

/* FT90M 舵机：与摄像头直接相连
 * 20ms 时基，1.0ms ~ 2.0ms 对应 0° ~ 280°
 * 机械安装：自上而下看顺时针旋转，中心点(140°)时摄像头朝右
 * 偏置修正：标称中心 140° = 2.443rad，实测中心 = 2.35rad
 */
#define FT90M_PWM_PULSE_MIN_US      1000u   /* 1.0 ms */
#define FT90M_PWM_PULSE_MAX_US      2000u   /* 2.0 ms */
#define FT90M_ANGLE_OFFSET_RAD      (2.35f - 140.0f * PI / 180.0f)  /* ≈ -0.093 rad */
#define FT90M_ANGLE_MIN_RAD         0.0f    /* 实际机械最小角 0° */
#define FT90M_ANGLE_MAX_RAD         (280.0f * PI / 180.0f)          /* 实际机械最大角 280° ≈ 4.8869 rad */

/* ==================== 兼容宏 ==================== */
#define Servo_Control_Cycle         1

/* ==================== 数据结构 ==================== */
typedef struct
{
    Motor_Type_t Motor_Type;
    TIM_HandleTypeDef *Motor_PWM_Timer; // PWM输出的定时器句柄
    uint32_t Motor_PWM_Channel;         // PWM输出的定时器通道
    volatile float Motor_Position;      // 接口目标角度，单位 rad

    /* 舵机特性参数，用于通用线性映射 */
    uint16_t PWM_Pulse_Min;             // 最小脉宽，单位 us
    uint16_t PWM_Pulse_Max;             // 最大脉宽，单位 us
    float Angle_Offset_Rad;             // 机械安装偏置：实际角度 = 接口角度 + 偏置
    float Angle_Min_Rad;                // 实际机械最小角，单位 rad
    float Angle_Max_Rad;                // 实际机械最大角，单位 rad
} Servo_Motor_Handle_t;

extern Servo_Motor_Handle_t Servo_Motor_Handle[];

void Servo_Motor_Control_Init(void);
void Servo_Motor_Set_Angle(Servo_Motor_Handle_t *handle);

#endif
