#include "Control_Algorithm.h"
#include "LK4005_Motor_Driver.h"

// 弧度归一化到0~2pi
float Normalize_Angle(float Angle)
{
    float Res = fmodf(Angle, 2.0f * PI);
    if (Res < 0)
    {
        Res += 2.0f * PI;
    }
    return Res;
}

/*
//达妙电机由于其特殊的CAN报文控制用不到这个函数
*/
void Motor_MIT_Control(Motor_MIT_Control_Handle_t *Motor_MIT_Control_Handle)
{
    Motor_MIT_Control_Handle->Output = Motor_MIT_Control_Handle->MIT_Kp * (Motor_MIT_Control_Handle->Motor_Position_Target - Motor_MIT_Control_Handle->Motor_Position_Actual) + Motor_MIT_Control_Handle->MIT_Kd * (Motor_MIT_Control_Handle->Motor_Velocity_Target - Motor_MIT_Control_Handle->Motor_Velocity_Actual) + Motor_MIT_Control_Handle->Motor_Torque_Feedforward;

    if (fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) >= 0.001f)
    {
        Motor_MIT_Control_Handle->Output += (Motor_MIT_Control_Handle->Motor_Velocity_Actual) / fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) * Motor_MIT_Control_Handle->Motor_Torque_Friction;
    }
}

static float Calc_V1(float a_max, float j)
{
    return (a_max * a_max) / (2.0f * j);
}

static float Calc_Decel_Dist(float v, float a_max, float j)
{
    return (v * v) / (2.0f * a_max) + (v * a_max) / (2.0f * j);
}

void Speed_Plan_Update(Speed_Plan_Handle_t *Speed_Plan_Handle, float position_actual, float position_target,Motor_Type_t Motor_Type)
{
    uint32_t Current_Time = HAL_GetTick();
    float dt = (Current_Time - Speed_Plan_Handle->Time_Stamp) * 0.001f;
    float v1 = Calc_V1(Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

    switch (Speed_Plan_Handle->Speed_Plan_State)
    {
    case idle:
    {
        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->v = 0;
        Speed_Plan_Handle->s = 0;
        break;
    }
    case init:
    {
        float old_direction = Speed_Plan_Handle->direction_flag;
        Speed_Plan_Handle->error_s = position_target - position_actual;
        if(Motor_Type != Gimbal)
        {
            if (position_actual > 0.0f)
            {
                Speed_Plan_Handle->position_initial = position_actual + 0.03f;
            }
            else if (position_actual < 0.0f)
            {
                Speed_Plan_Handle->position_initial = position_actual - 0.03f;
            }
            else
            {
                Speed_Plan_Handle->position_initial = position_actual;
            }
        }
        else
        {
            Speed_Plan_Handle->position_initial = position_actual;
        }

        if (fabsf(Speed_Plan_Handle->error_s) <= 0.05f)
        {
            Speed_Plan_Handle->a = 0;
            Speed_Plan_Handle->v = 0;
            Speed_Plan_Handle->s = 0;
            Speed_Plan_Handle->Speed_Plan_State = idle;
            break;
        }

        if (Speed_Plan_Handle->error_s >= 0)
        {
            Speed_Plan_Handle->direction_flag = 1.0f;
        }
        else
        {
            Speed_Plan_Handle->direction_flag = -1.0f;
        }

        /* Direction reversal: attenuate inherited velocity to reduce Kd shock */
        if (old_direction * Speed_Plan_Handle->direction_flag < 0.0f)
        {
            Speed_Plan_Handle->v *= 0.3f;
        }

        /* Adaptive v_limit: pre-compute peak speed for this displacement */
        {
            float S = fabsf(Speed_Plan_Handle->error_s);
            float v1 = Calc_V1(Speed_Plan_Handle->a_max, Speed_Plan_Handle->j);
            float b = (Speed_Plan_Handle->a_max * Speed_Plan_Handle->a_max) / Speed_Plan_Handle->j;
            float discriminant = b * b + 4.0f * S * Speed_Plan_Handle->a_max;
            float v_peak = (-b + sqrtf(discriminant)) / 2.0f;

            if (v_peak < 2.0f * v1)
            {
                /* Triangle S-curve: cannot even reach a_max */
                v_peak = powf(0.5f * Speed_Plan_Handle->j * S * S, 1.0f / 3.0f);
            }

            /* Short-distance attenuation: further reduce v_limit for small moves */
            float scale = 1.0f;
            if (S < 0.08f)
            {
                scale = 0.30f;   /* tiny step: 30% -> 60% (relaxed for speed) */
            }
            else if (S < 0.12f)
            {
                scale = 0.40f;   /* small step: 40% -> 80% (relaxed for speed) */
            }
            else if (S < 0.30f)
            {
                scale = 0.60f;   /* medium step: 60% -> 100% (relaxed for speed) */
            }
            v_peak *= scale;

            Speed_Plan_Handle->v_limit = fminf(v_peak, Speed_Plan_Handle->v_max);
            if (Speed_Plan_Handle->v_limit < 0.0f)
            {
                Speed_Plan_Handle->v_limit = 0.0f;
            }

            /* Dynamic j_limit for smoother short-distance motion */
            if (S < 0.08f)
            {
                Speed_Plan_Handle->j_limit = 12.0f;
            }
            else if (S < 0.12f)
            {
                Speed_Plan_Handle->j_limit = 16.0f;
            }
            else if (S < 0.30f)
            {
                Speed_Plan_Handle->j_limit = 20.0f;
            }
            else
            {
                Speed_Plan_Handle->j_limit = Speed_Plan_Handle->j;
            }

            /* Compute effective max accel: ensure v_limit >= 2*v1' */
            float v1_limit = Calc_V1(Speed_Plan_Handle->a_max, Speed_Plan_Handle->j_limit);
            if (Speed_Plan_Handle->v_limit < 2.0f * v1_limit)
            {
                Speed_Plan_Handle->a_limit = sqrtf(Speed_Plan_Handle->j_limit * Speed_Plan_Handle->v_limit);
            }
            else
            {
                Speed_Plan_Handle->a_limit = Speed_Plan_Handle->a_max;
            }
        }

        /* ========== Velocity safety & smart phase jump on re-planning ========== */
        float v_abs = fabsf(Speed_Plan_Handle->v);

        /* Prediction cmd (0x00): use original init logic for smooth preview motion */
        if (Speed_Plan_Handle->cmd_type == 0x00)
        {
            Speed_Plan_Handle->a = 0.0f;
            Speed_Plan_Handle->s = 0.0f;
            Speed_Plan_Handle->Speed_Plan_State = phase1;
            break;
        }

        /* Confirm cmd (0x01): use v9 safety logic for final execution */
        /* 1. Clamp inherited velocity to new v_limit (with 20% overshoot margin) */
        if (v_abs > Speed_Plan_Handle->v_limit * 1.2f)
        {
            float v_clamp = Speed_Plan_Handle->v_limit * 1.2f;
            if (v_clamp > Speed_Plan_Handle->v_max)
            {
                v_clamp = Speed_Plan_Handle->v_max;
            }
            Speed_Plan_Handle->v = (Speed_Plan_Handle->v >= 0.0f ? 1.0f : -1.0f) * v_clamp;
            v_abs = v_clamp;
        }

        /* 2. Brake-distance safety: if we can't stop in error_s, force lower speed */
        float decel_needed = Calc_Decel_Dist(v_abs,
                                              Speed_Plan_Handle->a_limit,
                                              Speed_Plan_Handle->j_limit);
        if (decel_needed >= fabsf(Speed_Plan_Handle->error_s))
        {
            float v_safe = v_abs;
            /* Iteratively reduce speed until we can stop within the distance,
               or floor at a minimal crawl speed */
            while (decel_needed >= fabsf(Speed_Plan_Handle->error_s) && v_safe > 0.10f)
            {
                v_safe *= 0.92f;
                decel_needed = Calc_Decel_Dist(v_safe,
                                                Speed_Plan_Handle->a_limit,
                                                Speed_Plan_Handle->j_limit);
            }
            Speed_Plan_Handle->v = (Speed_Plan_Handle->v >= 0.0f ? 1.0f : -1.0f) * v_safe;
            Speed_Plan_Handle->a = 0.0f;
            Speed_Plan_Handle->s = 0.0f;
            Speed_Plan_Handle->Speed_Plan_State = phase3_end;
            break;
        }

        /* 3. Smart phase jump: if already fast, skip acceleration phase */
        if (v_abs > Speed_Plan_Handle->v_limit * 0.6f)
        {
            Speed_Plan_Handle->a = 0.0f;
            Speed_Plan_Handle->s = 0.0f;
            Speed_Plan_Handle->Speed_Plan_State = phase3_end;
            break;
        }

        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->s = 0;

        Speed_Plan_Handle->Speed_Plan_State = phase1;
        break;
    }
    case phase1:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= Speed_Plan_Handle->a_limit)
        {
            Speed_Plan_Handle->a = Speed_Plan_Handle->a_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase2;
        }
        /* RUNTIME SAFE: prevent accelerating past v_limit */
        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_limit)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        break;
    }
    case phase2:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_limit - v1)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        /* RUNTIME SAFE: prevent accelerating past v_limit */
        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_limit)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        break;
    }
    case phase3:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        /* RUNTIME SAFE: clamp v during P3 to prevent overshoot past v_limit */
        if (Speed_Plan_Handle->v > Speed_Plan_Handle->v_limit)
        {
            Speed_Plan_Handle->v = Speed_Plan_Handle->v_limit;
        }
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a <= 0)
        {
            Speed_Plan_Handle->a = 0;
            if (Speed_Plan_Handle->v > Speed_Plan_Handle->v_limit)
                Speed_Plan_Handle->v = Speed_Plan_Handle->v_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase3_end;
        }
        break;
    }
    case phase3_end:
    {
        float decel_dist = Calc_Decel_Dist(Speed_Plan_Handle->v, Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

        if (Speed_Plan_Handle->s >= fabsf(Speed_Plan_Handle->error_s) - decel_dist)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase5;
        }
        else
        {
            Speed_Plan_Handle->Speed_Plan_State = phase4;
        }
        break;
    }
    case phase4:
    {
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        float decel_dist = Calc_Decel_Dist(Speed_Plan_Handle->v, Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

        if (Speed_Plan_Handle->s >= fabsf(Speed_Plan_Handle->error_s) - decel_dist)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase5;
        }
        break;
    }
    case phase5:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        /* RUNTIME SAFE: hard protection if not decelerating fast enough */
        float remaining = fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s;
        if (remaining > 0.001f && Speed_Plan_Handle->v > 0.1f)
        {
            float min_a_needed = -(Speed_Plan_Handle->v * Speed_Plan_Handle->v) / (2.0f * remaining);
            if (Speed_Plan_Handle->a > min_a_needed)
            {
                Speed_Plan_Handle->a = min_a_needed;
            }
        }

        if (Speed_Plan_Handle->a <= -Speed_Plan_Handle->a_limit)
        {
            Speed_Plan_Handle->a = -Speed_Plan_Handle->a_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase6;
        }
        break;
    }
    case phase6:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v <= v1)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase7;
        }
        break;
    }
    case phase7:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= 0 || Speed_Plan_Handle->v <= 0)
        {
            Speed_Plan_Handle->a = 0;
            Speed_Plan_Handle->v = 0;
            Speed_Plan_Handle->Speed_Plan_State = idle;
        }
        break;
    }
    }
    if (fabsf(Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s)) <= 0.003f || Speed_Plan_Handle->v < 0 || Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s) >= 0)
    {
        Speed_Plan_Handle->v = 0;
        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->s = 0;
        Speed_Plan_Handle->Speed_Plan_State = idle;
    }
    Speed_Plan_Handle->Time_Stamp = HAL_GetTick();
}

/*
 * 大臂电机重力补偿
 * 按 Mechanism_Spec.md 势能法推导（新 θ2 定义：0=反向共线），返回电机输出轴应施加的补偿力矩（N·m）
 * upper_motor_angle: 大臂电机输出轴实际角度（rad），即 LK4005 Motor_Position_Actual
 * fore_motor_angle : 小臂电机输出轴实际角度（rad）
 * phi_servo        : 末端相对小臂延长线的偏角（rad），0 表示共线，π/2 表示垂直向上
 */
float Upperarm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo)
{
    /* 关节角换算（见 Mechanism_Spec.md 4.1 / 4.2） */
    float theta1 = -upper_motor_angle / 4.0f;  /* 大臂与竖直向下的夹角 */
    float theta2 =  fore_motor_angle / 2.0f;   /* 小臂展开角（新定义：0=反向共线） */
    float phi    =  phi_servo;                 /* 直接就是末端相对小臂的偏角 */

    /* 预计算三角函数，避免重复调用 */
    float c1h  = cosf(theta1 / 2.0f);                  /* cos(θ1/2)          */
    float s1   = sinf(theta1);                         /* sin(θ1)            */
    float s_g2 = sinf(theta1 - theta2 / 2.0f);         /* sin(θ1 - θ2/2)     */
    float s12  = sinf(theta1 + theta2);                /* sin(θ1 + θ2)       */
    float s_end = sinf(theta1 + theta2 + phi);         /* sin(θ1+θ2+φ_servo) */

    /* 预计算长度×三角函数，保持公式可读性 */
    float half_Lg1_c1h = (Robotic_Arm_Length_Gear1 / 2.0f) * c1h;
    float half_L1_s1   = (Robotic_Arm_Length_L1    / 2.0f) * s1;
    float half_Lg2_sg2 = (Robotic_Arm_Length_Gear2 / 2.0f) * s_g2;
    float half_L2_s12  = (Robotic_Arm_Length_L2    / 2.0f) * s12;

    float L1_s1  = Robotic_Arm_Length_L1 * s1;
    float Lg2_sg2 = Robotic_Arm_Length_Gear2 * s_g2;
    float L2_s12 = Robotic_Arm_Length_L2 * s12;
    float Lend_s_end = Robotic_Arm_Length_End * s_end;

    /* ∂V/∂θ1 ：势能对大臂关节角的偏导（按 Mechanism_Spec.md 关节偏置杆模型，新 θ2 定义） */
    float dV_dtheta1 =
          Robotic_Arm_Mass_Gear1 * g * (Robotic_Arm_Length_Gear1 / 4.0f) * c1h
        + Robotic_Arm_Mass_L1    * g * (half_Lg1_c1h + half_L1_s1)
        + Robotic_Arm_Mass_Gear2 * g * (half_Lg1_c1h + L1_s1 + half_Lg2_sg2)
        + Robotic_Arm_Mass_L2    * g * (half_Lg1_c1h + L1_s1 + Lg2_sg2 - half_L2_s12)
        + Robotic_Arm_Mass_End   * g * (half_Lg1_c1h + L1_s1 + Lg2_sg2 - L2_s12 - Lend_s_end);

    /* Q1 = -∂V/∂θ1 ；电机补偿力矩 τ_m1 = Q1 / 4 = -(∂V/∂θ1) / 4 */
    return -dV_dtheta1 / 4.0f;
}

/*
 * 小臂电机重力补偿
 * 按 Mechanism_Spec.md 势能法推导（新 θ2 定义：0=反向共线），返回电机输出轴应施加的补偿力矩（N·m）
 * phi_servo: 末端相对小臂延长线的偏角（rad）
 */
float Forearm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo)
{
    float theta1 = -upper_motor_angle / 4.0f;
    float theta2 =  fore_motor_angle / 2.0f;   /* 新 θ2 定义：0=反向共线 */
    float phi    =  phi_servo;                 /* 直接就是末端相对小臂的偏角 */

    float s_g2  = sinf(theta1 - theta2 / 2.0f);
    float s12   = sinf(theta1 + theta2);
    float s_end = sinf(theta1 + theta2 + phi);

    /* 预计算长度×三角函数 */
    float half_Lg2_sg2 = (Robotic_Arm_Length_Gear2 / 2.0f) * s_g2;
    float half_L2_s12  = (Robotic_Arm_Length_L2    / 2.0f) * s12;
    float L2_s12 = Robotic_Arm_Length_L2 * s12;
    float Lend_s_end = Robotic_Arm_Length_End * s_end;

    /* ∂V/∂θ2 ：势能对小臂展开角的偏导（按 Mechanism_Spec.md 关节偏置杆模型，新 θ2 定义） */
    float dV_dtheta2 =
        - Robotic_Arm_Mass_Gear2 * g * (Robotic_Arm_Length_Gear2 / 4.0f) * s_g2
        - Robotic_Arm_Mass_L2    * g * (half_Lg2_sg2 + half_L2_s12)
        - Robotic_Arm_Mass_End   * g * (half_Lg2_sg2 + L2_s12 + Lend_s_end);

    /* τ_m2 = Q2 / 2 = -(∂V/∂θ2) / 2 */
    return -dV_dtheta2 / 2.0f;
}

/*
 * 标准二连杆逆解（忽略 Gear 杆，新 θ2 定义：0=反向共线）
 * 输入：目标水平距离 R、目标高度 Z、等效杆参数 L_eq/delta
 * 输出：大臂角 theta1、小臂角 theta2
 *
 * 几何模型：
 *   第一杆（大臂）方向角 = θ1（从竖直向下顺时针）
 *   第二杆（等效 J2→E）绝对方向角 = θ1 + π + β，其中 β = θ2 + delta
 *   R = L1·sin θ1 - L_eq·sin(θ1 + β)
 *   Z - L_connect = -L1·cos θ1 + L_eq·cos(θ1 + β)
 */
static void Standard_TwoLink_IK(float R, float Z, float L1, float L_eq, float delta,
                                float *theta1, float *theta2)
{
    float dZ = Z - Robotic_Arm_Length_Connect;
    float L_J1E_sq = R * R + dZ * dZ;
    float L_J1E    = sqrtf(L_J1E_sq);

    /* β = θ2 + delta，由余弦定理求 cos(β) */
    float cos_beta = (L1 * L1 + L_eq * L_eq - L_J1E_sq) / (2.0f * L1 * L_eq);
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);

    /* 大臂角 θ1 = α + J1_angle */
    float alpha = atan2f(R, -dZ);
    float cos_J1 = (L1 * L1 + L_J1E_sq - L_eq * L_eq) / (2.0f * L1 * L_J1E);
    if (cos_J1 > 1.0f)  cos_J1 = 1.0f;
    if (cos_J1 < -1.0f) cos_J1 = -1.0f;
    float J1_angle = acosf(cos_J1);
    *theta1 = alpha + J1_angle;

    /* 小臂展开角 */
    *theta2 = beta - delta;
}

/*
 * 坐标逆解算
 * 输入：目标位置 (X, Y, Z) 和末端偏角 phi_servo（rad，0 表示末端在小臂延长线上）
 * 输出：云台角、大臂关节角 θ1、小臂展开角 θ2（均为 rad）
 *
 * 注意：定位点已改为小臂末端 J3（不再是摄像头光心 E）。
 *   J1 -> Gear1 -> P1(大臂根部) -> 大臂 L1(theta1) -> J2 -> Gear2 -> P2(小臂根部) -> 小臂 L2(theta2) -> J3
 * phi_servo 仅用于舵机控制，不再影响臂部逆解算。
 *
 * 由于本函数在串口中断中仅执行一次，不能迭代。
 * 策略：固定点迭代——用标准二连杆求初值，计算 Gear 偏置修正目标，再求解一次。
 *   第1次：求解忽略 Gear 杆的标准模型 -> theta1_0, theta2_0
 *   第2次：用 theta1_0, theta2_0 计算 Gear 偏置 dy, dz，修正目标后重新求解 -> theta1, theta2
 * 单次修正可将误差从 ~10deg 降到 ~1deg 量级。
 */
void Coordinate_Inverse_Settlement(float X, float Y, float Z, float phi_servo,
                                   float *Gimbal_Angle, float *Joint_Upper_Angle, float *Joint_Fore_Angle)
{
    /* ---------- 1. 云台水平角（保持与当前同圈，并取最近方向） ---------- */
    float current_gimbal = LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual;
    float base_turns = floorf(current_gimbal / (2.0f * PI)) * 2.0f * PI;
    float gimbal_raw = atan2f(X, Y) + base_turns;
    /* 约束到实际位置 ±π 范围内，避免 floorf 在 0/2π 边界导致跨圈跳变 */
    float gimbal_delta = gimbal_raw - current_gimbal;
    if (gimbal_delta > PI)
        gimbal_raw -= 2.0f * PI;
    else if (gimbal_delta < -PI)
        gimbal_raw += 2.0f * PI;
    *Gimbal_Angle = gimbal_raw;

    /* ---------- 2. 常数参数 ---------- */
    float R  = sqrtf(X * X + Y * Y);                  /* 水平径向距离 */
    float L1   = Robotic_Arm_Length_L1;
    float L2   = Robotic_Arm_Length_L2;
    float Lg1  = Robotic_Arm_Length_Gear1;
    float Lg2  = Robotic_Arm_Length_Gear2;

    /* 定位点改为 J3（小臂末端），等效长度仅含 L2，不再叠加 L_end */
    float L_eq = L2;
    float delta = 0.0f;

    /* phi_servo 不再影响臂部逆解算，仅保留参数用于兼容上层调用 */
    (void)phi_servo;

    /* ---------- 3. 第一次求解：忽略 Gear 杆 ---------- */
    float theta1_0, theta2_0;
    Standard_TwoLink_IK(R, Z, L1, L_eq, delta, &theta1_0, &theta2_0);

    /* ---------- 4. 计算 Gear 杆引起的末端偏置 ---------- */
    float half_t1 = theta1_0 * 0.5f;

    /* 新 θ2 定义下：
     *   n̂_g2 = (0, sin(θ1 + θ2/2), -cos(θ1 - θ2/2))
     *   dy_gear = Lg1·cos(θ1/2) + Lg2·sin(θ1 + θ2/2)
     *   dz_gear = Lg1·sin(θ1/2) - Lg2·cos(θ1 - θ2/2)
     */
    float dy_gear = Lg1 * cosf(half_t1) + Lg2 * sinf(theta1_0 + theta2_0 * 0.5f);
    float dz_gear = Lg1 * sinf(half_t1) - Lg2 * cosf(theta1_0 - theta2_0 * 0.5f);

    /* ---------- 5. 第二次求解：修正 Gear 偏置后的目标 ---------- */
    float theta1, theta2;
    Standard_TwoLink_IK(R - dy_gear, Z - dz_gear, L1, L_eq, delta, &theta1, &theta2);

    *Joint_Upper_Angle = theta1;
    *Joint_Fore_Angle  = theta2;
}
