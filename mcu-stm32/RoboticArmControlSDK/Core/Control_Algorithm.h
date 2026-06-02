#ifndef __CONTROL_ALGORITHM_H
#define __CONTROL_ALGORITHM_H

#include "Robotic_Arm_Config.h"

float Normalize_Angle(float Angle);
void Motor_MIT_Control(Motor_MIT_Control_Handle_t *Motor_MIT_Control_Handle);
float Upperarm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo);
float Forearm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo);
void Speed_Plan_Update(Speed_Plan_Handle_t *Speed_Plan_Handle, float position_actual, float position_target);
void Coordinate_Inverse_Settlement(float X, float Y, float Z, float phi_servo, float *Gimbal_Angle, float *Joint_Upper_Angle, float *Joint_Fore_Angle);

#endif
