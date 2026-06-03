#ifndef __ROBOTIC_ARM_CONFIG_H
#define __ROBOTIC_ARM_CONFIG_H

#include "main.h"

typedef enum
{
    Gimbal, //云台电机
    Joint_Upper, //大臂关节电机
    Joint_Fore, //小臂关节电机
    Servo, //舵机
} Motor_Type_t;

typedef enum
{
    idle,
    init,
    phase1,
    phase2,
    phase3,
    phase3_end,
    phase4,
    phase5,
    phase6,
    phase7,
} Speed_Plan_State_type;

typedef struct
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} IMU_Raw_Data_t;

typedef struct
{
    SPI_HandleTypeDef *SPI_Handle;
    uint8_t Tx_Data_Buff;
    uint8_t Rx_Data_Buff;
    IMU_Raw_Data_t IMU_Raw_Data;
    float yaw;
    float pitch;
    float roll;
} IMU_Handle_t;

typedef struct 
{
    uint32_t Time_Stamp;
    float j;
    float j_limit;
    float a;
    float a_max;
    float a_limit;
    float v;
    float v_max;
    float v_limit;
    float error_s;
    float position_initial;
    float direction_flag;
    float s;
    float s_accel_used;
    float s2;
    float s1;
    float v1;
    Speed_Plan_State_type Speed_Plan_State;

}Speed_Plan_Handle_t;


typedef struct
{
    float Motor_Position_Actual;    // 单位为rad
    float Motor_Position_Target;    // 单位为rad
    float Motor_Velocity_Actual;    // 单位为rad/s
    float Motor_Velocity_Target;    // 单位为rad/s
    float Motor_Torque_Feedforward; // 单位为N*m
    float Motor_Torque_Friction;
    float MIT_Kp;
    float MIT_Kd;
    float Output;
} Motor_MIT_Control_Handle_t;

typedef struct
{
    float Motor_Position_Actual; //范围:0~2pi,单位为rad
    volatile float Motor_Position_Target; // 范围:0~2pi,单位为rad
} Motor_Position_PID_Control_Handle_t; //供云台电机使用

typedef struct
{
    Motor_Type_t Motor_Type;
    FDCAN_HandleTypeDef *Motor_FDCAN_Handle; //FDCAN通信句柄
    uint16_t Motor_ID;
    uint8_t Motor_Status;
    float Motor_Torque_Actual;   //单位为N*m
    float Motor_Position_Target; //单位为rad
    Motor_MIT_Control_Handle_t Motor_MIT_Control_Handle;
    Speed_Plan_Handle_t Motor_Speed_Plan_Handle;
    uint64_t Wait_Count;
} DMJ4310_Motor_Handle_t;

typedef struct
{
    Motor_Type_t Motor_Type;
    FDCAN_HandleTypeDef *Motor_FDCAN_Handle;
    uint16_t Motor_ID;
    float Motor_Position_Target;                         // 单位为rad
    Motor_MIT_Control_Handle_t Motor_MIT_Control_Handle[2]; // Output范围:-33A~33A,单位为A
    Motor_Position_PID_Control_Handle_t Motor_Position_PID_Control_Handle;
    Speed_Plan_Handle_t Motor_Speed_Plan_Handle;
    uint64_t Wait_Count;
} LK4005_Motor_Handle_t;

#define PI 3.14159f
#define Servo_Motor_Number 2
#define LK4005_Motor_Number 3
#define Robotic_Arm_Mass_Gear1 0.05f // Gear1 摇臂质量,单位为kg
#define Robotic_Arm_Mass_L1 0.3f //大臂机械臂的质量,单位为kg
#define Robotic_Arm_Mass_Gear2 0.05f // Gear2 摇臂质量,单位为kg
#define Robotic_Arm_Mass_L2 0.3f // 小臂机械臂的质量,单位为kg
#define Robotic_Arm_Mass_End 0.09f   // 末端舵机,摄像头等的总质量,单位为kg
#define Robotic_Arm_Length_L1 0.44f // 大臂机械臂的长度,单位为m
#define Robotic_Arm_Length_L2 0.30f // 小臂机械臂的长度,单位为m
#define Robotic_Arm_Length_Connect 0.35f //大臂与云台连接杆的长度,单位为m   
#define Robotic_Arm_Angle_Offset 1.571f //整体机械臂(云台)相对于地面的夹角,单位为rad 
#define Robotic_Arm_Length_End 0.073f
#define Robotic_Arm_Length_Gear1 0.054f //第一个齿轮连接杆的长度
#define Robotic_Arm_Length_Gear2 0.026f // 第二个齿轮连接杆的长度
#define g 9.7913f

#endif
