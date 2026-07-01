#include "Control_Algorithm.h"
#include "LK4005_Motor_Driver.h"

float Normalize_Angle(float Angle)
{
    float Res = fmodf(Angle, 2.0f * PI);
    if (Res < 0)
    {
        Res += 2.0f * PI;
    }
    return Res;
}

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

/* 正确的S曲线减速距离 */
static float Calc_Decel_Dist(float v, float a_max, float j)
{
    float v_jerk = a_max * a_max / (2.0f * j);

    if (v <= 2.0f * v_jerk)
    {
        /* 三角S曲线：phase5 + phase7，无匀减速段 */
        return powf(v, 1.5f) / sqrtf(j);
    }

    /* 完整7段S曲线减速：phase5 + phase6 + phase7 */
    return (v * v) / (2.0f * a_max) + (v * a_max) / (2.0f * j);
}

/* 单步子步积分，dt 必须很小（如 1ms） */
static void Speed_Plan_One_Step(Speed_Plan_Handle_t *sp, float dt)
{
    float v1 = Calc_V1(sp->a_limit, sp->j_limit);

    switch (sp->Speed_Plan_State)
    {
    case phase1:
    {
        sp->a += sp->j_limit * dt;
        if (sp->a >= sp->a_limit)
        {
            sp->a = sp->a_limit;
            sp->Speed_Plan_State = phase2;
        }
        sp->v += sp->a * dt;
        if (sp->v >= sp->v_limit)
        {
            sp->v = sp->v_limit;
            sp->a = 0.0f;
            sp->Speed_Plan_State = phase3_end;
        }
        sp->s += sp->v * dt;
        break;
    }

    case phase2:
    {
        sp->v += sp->a * dt;
        if (sp->v >= sp->v_limit - v1)
        {
            sp->Speed_Plan_State = phase3;
        }
        sp->s += sp->v * dt;
        break;
    }

    case phase3:
    {
        sp->a -= sp->j_limit * dt;
        sp->v += sp->a * dt;
        if (sp->v > sp->v_limit)
        {
            sp->v = sp->v_limit;
            sp->a = 0.0f;
            sp->Speed_Plan_State = phase3_end;
        }
        sp->s += sp->v * dt;

        if (sp->a <= 0.0f)
        {
            sp->a = 0.0f;
            sp->Speed_Plan_State = phase3_end;
        }
        break;
    }

    case phase3_end:
    {
        float decel_dist = Calc_Decel_Dist(sp->v, sp->a_limit, sp->j_limit);
        if (sp->s >= fabsf(sp->error_s) - decel_dist)
            sp->Speed_Plan_State = phase5;
        else
            sp->Speed_Plan_State = phase4;
        break;
    }

    case phase4:
    {
        sp->s += sp->v * dt;
        float decel_dist = Calc_Decel_Dist(sp->v, sp->a_limit, sp->j_limit);
        if (sp->s >= fabsf(sp->error_s) - decel_dist)
        {
            sp->Speed_Plan_State = phase5;
        }
        break;
    }

    case phase5:
    {
        sp->a -= sp->j_limit * dt;
        if (sp->a <= -sp->a_limit)
        {
            sp->a = -sp->a_limit;
            sp->Speed_Plan_State = phase6;
        }
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;
        break;
    }

    case phase6:
    {
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;
        if (sp->v <= v1)
        {
            sp->Speed_Plan_State = phase7;
        }
        break;
    }

    case phase7:
    {
        sp->a += sp->j_limit * dt;
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;

        if (sp->a >= 0.0f || sp->v <= 0.0f)
        {
            sp->a = 0.0f;
            sp->v = 0.0f;
            sp->s = fabsf(sp->error_s);
            sp->Speed_Plan_State = idle;
        }
        break;
    }

    default:
        break;
    }
}

void Speed_Plan_Update(Speed_Plan_Handle_t *sp, float position_actual, float position_target, Motor_Type_t Motor_Type)
{
    uint32_t now = HAL_GetTick();
    float dt_total = (now - sp->Time_Stamp) * 0.001f;
    sp->Time_Stamp = now;

    if (dt_total <= 0.0f)
        return;

    /* ---------- idle：已到位，保持静止，不清零 s（保留 error_s 供观察） ---------- */
    if (sp->Speed_Plan_State == idle)
    {
        sp->a = 0.0f;
        sp->v = 0.0f;
        return;
    }

    /* ---------- init：外部触发的新目标启动或打断重规划 ---------- */
    if (sp->Speed_Plan_State == init)
    {
        float err = position_target - position_actual;
        if (fabsf(err) <= 0.03f)
        {
            sp->Speed_Plan_State = idle;
            return;
        }

        float old_v   = sp->v;
        float old_dir = sp->direction_flag;

        sp->error_s = err;
        sp->direction_flag = (err >= 0.0f) ? 1.0f : -1.0f;

        /* 方向反转：衰减继承速度 */
        if (old_dir * sp->direction_flag < 0.0f)
        {
            old_v *= 0.3f;
        }

        if (Motor_Type != Gimbal)
        {
            if (position_actual > 0.0f)
            {
                sp->position_initial = position_actual + 0.02f;
            }
            else if (position_actual < 0.0f)
            {
                sp->position_initial = position_actual - 0.02f;
            }
            else
            {
                sp->position_initial = position_actual;
            }
        }
        else
        {
            sp->position_initial = position_actual;
        }

        sp->s = 0.0f;
        sp->a = 0.0f;

        float S = fabsf(sp->error_s);
        float v1 = Calc_V1(sp->a_max, sp->j);
        float b  = sp->a_max * sp->a_max / sp->j;
        float discriminant = b * b + 4.0f * S * sp->a_max;
        float v_peak = (-b + sqrtf(discriminant)) / 2.0f;

        if (v_peak < 2.0f * v1)
        {
            v_peak = powf(0.5f * sp->j * S * S, 1.0f / 3.0f);
        }

        float scale = 1.0f;
        if (S < 0.08f)
            scale = 0.15f;
        else if (S < 0.20f)
            scale = 0.35f;
        else if (S < 0.60f)
            scale = 0.65f;
        else if (S < 1.20f)
            scale = 0.90f;
        v_peak *= scale;

        sp->v_limit = fminf(v_peak, sp->v_max);
        if (sp->v_limit < 0.0f) sp->v_limit = 0.0f;

        if (S < 0.08f)
            sp->j_limit = 8.0f;
        else if (S < 0.15f)
            sp->j_limit = 12.0f;
        else if (S < 0.30f)
            sp->j_limit = 16.0f;
        else
            sp->j_limit = sp->j;

        float v1_limit = Calc_V1(sp->a_max, sp->j_limit);
        if (sp->v_limit < 2.0f * v1_limit)
            sp->a_limit = sqrtf(sp->j_limit * sp->v_limit);
        else
            sp->a_limit = sp->a_max;

        /* ========== 打断智能处理 ========== */
        if (old_dir * sp->direction_flag > 0.0f)  /* 同向打断 */
        {
            float decel_needed = Calc_Decel_Dist(old_v, sp->a_limit, sp->j_limit);

            if (S >= decel_needed * 1.1f)
            {
                /* 距离充裕：根据当前速度与新峰值的差距决定是继承、减速还是重新加速 */
                if (old_v > sp->v_limit * 1.05f)
                {
                    /* 当前速度超过新峰值，需要减速适配 */
                    sp->v = fminf(old_v, sp->v_limit * 1.05f);
                    sp->Speed_Plan_State = phase3_end;
                }
                else if (old_v < sp->v_limit * 0.85f)
                {
                    /* 速度还低，继续加速到新的 v_limit */
                    sp->v = old_v;
                    sp->a = 0.0f;
                    sp->Speed_Plan_State = phase1;
                }
                else
                {
                    /* 速度接近峰值，丝滑继承匀速 */
                    sp->v = old_v;
                    sp->Speed_Plan_State = phase3_end;
                }
            }
            else
            {
                /* 距离不够：强制降到安全速度 */
                float v_safe = old_v;
                for (int i = 0; i < 15 && v_safe > 0.05f; i++)
                {
                    v_safe *= 0.82f;
                    decel_needed = Calc_Decel_Dist(v_safe, sp->a_limit, sp->j_limit);
                    if (decel_needed <= S)
                        break;
                }
                sp->v = v_safe;
                sp->Speed_Plan_State = phase3_end;
            }
        }
        else  /* 反向打断 */
        {
            /* 制动距离安全检查 */
            float v_safe = old_v;
            if (v_safe > 0.05f)
            {
                float decel_needed = Calc_Decel_Dist(v_safe, sp->a_limit, sp->j_limit);
                if (decel_needed > S)
                {
                    for (int i = 0; i < 15 && v_safe > 0.05f; i++)
                    {
                        v_safe *= 0.82f;
                        decel_needed = Calc_Decel_Dist(v_safe, sp->a_limit, sp->j_limit);
                        if (decel_needed <= S)
                            break;
                    }
                }
            }

            if (v_safe > 0.10f)
            {
                sp->v = v_safe;
                sp->Speed_Plan_State = phase3_end;
            }
            else
            {
                sp->v = 0.0f;
                sp->Speed_Plan_State = phase1;
            }
        }

        sp->Time_Stamp = now;   /* 重置时间戳，防止本次 dt 爆炸 */
        return;                 /* 本次只做初始化，下次正常周期再开始积分 */
    }

    /* ---------- 子步精密积分：1ms 步长 ---------- */
    const float dt_step = 0.001f;
    int n_steps = (int)(dt_total / dt_step);
    float remainder = dt_total - n_steps * dt_step;

    for (int i = 0; i < n_steps && sp->Speed_Plan_State != idle; i++)
    {
        Speed_Plan_One_Step(sp, dt_step);
    }
    if (sp->Speed_Plan_State != idle && remainder > 1e-6f)
    {
        Speed_Plan_One_Step(sp, remainder);
    }

    /* ---------- 唯一硬保护 ---------- */
    if (sp->Speed_Plan_State != idle)
    {
        if (sp->s >= fabsf(sp->error_s) - 0.001f || sp->v < 0.0f)
        {
            sp->a = 0.0f;
            sp->v = 0.0f;
            sp->s = fabsf(sp->error_s);
            sp->Speed_Plan_State = idle;
        }
    }
}

float Upperarm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo)
{
    float theta1 = -upper_motor_angle / 4.0f;
    float theta2 =  fore_motor_angle / 2.0f;
    float phi    =  phi_servo;

    float c1h  = cosf(theta1 / 2.0f);
    float s1   = sinf(theta1);
    float s_g2 = sinf(theta1 - theta2 / 2.0f);
    float s12  = sinf(theta1 + theta2);
    float s_end = sinf(theta1 + theta2 + phi);

    float half_Lg1_c1h = (Robotic_Arm_Length_Gear1 / 2.0f) * c1h;
    float half_L1_s1   = (Robotic_Arm_Length_L1    / 2.0f) * s1;
    float half_Lg2_sg2 = (Robotic_Arm_Length_Gear2 / 2.0f) * s_g2;
    float half_L2_s12  = (Robotic_Arm_Length_L2    / 2.0f) * s12;

    float L1_s1  = Robotic_Arm_Length_L1 * s1;
    float Lg2_sg2 = Robotic_Arm_Length_Gear2 * s_g2;
    float L2_s12 = Robotic_Arm_Length_L2 * s12;
    float Lend_s_end = Robotic_Arm_Length_End * s_end;

    float dV_dtheta1 =
          Robotic_Arm_Mass_Gear1 * g * (Robotic_Arm_Length_Gear1 / 4.0f) * c1h
        + Robotic_Arm_Mass_L1    * g * (half_Lg1_c1h + half_L1_s1)
        + Robotic_Arm_Mass_Gear2 * g * (half_Lg1_c1h + L1_s1 + half_Lg2_sg2)
        + Robotic_Arm_Mass_L2    * g * (half_Lg1_c1h + L1_s1 + Lg2_sg2 - half_L2_s12)
        + Robotic_Arm_Mass_End   * g * (half_Lg1_c1h + L1_s1 + Lg2_sg2 - L2_s12 - Lend_s_end);

    return -dV_dtheta1 / 4.0f;
}

float Forearm_Gravity_Compensation(float upper_motor_angle, float fore_motor_angle, float phi_servo)
{
    float theta1 = -upper_motor_angle / 4.0f;
    float theta2 =  fore_motor_angle / 2.0f;
    float phi    =  phi_servo;

    float s_g2  = sinf(theta1 - theta2 / 2.0f);
    float s12   = sinf(theta1 + theta2);
    float s_end = sinf(theta1 + theta2 + phi);

    float half_Lg2_sg2 = (Robotic_Arm_Length_Gear2 / 2.0f) * s_g2;
    float half_L2_s12  = (Robotic_Arm_Length_L2    / 2.0f) * s12;
    float L2_s12 = Robotic_Arm_Length_L2 * s12;
    float Lend_s_end = Robotic_Arm_Length_End * s_end;

    float dV_dtheta2 =
        - Robotic_Arm_Mass_Gear2 * g * (Robotic_Arm_Length_Gear2 / 4.0f) * s_g2
        - Robotic_Arm_Mass_L2    * g * (half_Lg2_sg2 + half_L2_s12)
        - Robotic_Arm_Mass_End   * g * (half_Lg2_sg2 + L2_s12 + Lend_s_end);

    return -dV_dtheta2 / 2.0f;
}

static void Standard_TwoLink_IK(float R, float Z, float L1, float L_eq, float delta,
                                float *theta1, float *theta2)
{
    float dZ = Z - Robotic_Arm_Length_Connect;
    float L_J1E_sq = R * R + dZ * dZ;
    float L_J1E    = sqrtf(L_J1E_sq);

    float cos_beta = (L1 * L1 + L_eq * L_eq - L_J1E_sq) / (2.0f * L1 * L_eq);
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);

    float alpha = atan2f(R, -dZ);
    float cos_J1 = (L1 * L1 + L_J1E_sq - L_eq * L_eq) / (2.0f * L1 * L_J1E);
    if (cos_J1 > 1.0f)  cos_J1 = 1.0f;
    if (cos_J1 < -1.0f) cos_J1 = -1.0f;
    float J1_angle = acosf(cos_J1);
    *theta1 = alpha + J1_angle;

    *theta2 = beta - delta;
}

void Coordinate_Inverse_Settlement(float X, float Y, float Z, float phi_servo,
                                   float *Gimbal_Angle, float *Joint_Upper_Angle, float *Joint_Fore_Angle)
{
    float current_gimbal = LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual;
    float base_turns = floorf(current_gimbal / (2.0f * PI)) * 2.0f * PI;
    float gimbal_raw = atan2f(X, Y) + base_turns;
    float gimbal_delta = gimbal_raw - current_gimbal;
    if (gimbal_delta > PI)
        gimbal_raw -= 2.0f * PI;
    else if (gimbal_delta < -PI)
        gimbal_raw += 2.0f * PI;
    *Gimbal_Angle = gimbal_raw;

    float R  = sqrtf(X * X + Y * Y);
    float L1   = Robotic_Arm_Length_L1;
    float L2   = Robotic_Arm_Length_L2;
    float Lg1  = Robotic_Arm_Length_Gear1;
    float Lg2  = Robotic_Arm_Length_Gear2;

    float L_eq = L2;
    float delta = 0.0f;

    (void)phi_servo;

    float theta1_0, theta2_0;
    Standard_TwoLink_IK(R, Z, L1, L_eq, delta, &theta1_0, &theta2_0);

    float half_t1 = theta1_0 * 0.5f;
    float dy_gear = Lg1 * cosf(half_t1) + Lg2 * sinf(theta1_0 + theta2_0 * 0.5f);
    float dz_gear = Lg1 * sinf(half_t1) - Lg2 * cosf(theta1_0 - theta2_0 * 0.5f);

    float theta1, theta2;
    Standard_TwoLink_IK(R - dy_gear, Z - dz_gear, L1, L_eq, delta, &theta1, &theta2);

    *Joint_Upper_Angle = theta1;
    *Joint_Fore_Angle  = theta2;
}
