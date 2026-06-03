#include "Robotic_Arm_Control_API.h"

static uint8_t Gimbal_Start_Complete = 0;
static uint8_t Upper_Lock_Done = 0;
static uint8_t Fore_Lock_Done = 0;

/* 初始化完成后置 1，控制舵机何时开始跟踪目标值 */
uint8_t Servo_Control_Active = 0;

/* ========== FF初始化命令顺序状态机 ========== */
typedef enum
{
    INIT_SEQ_IDLE,
    INIT_SEQ_UPPER,
    INIT_SEQ_FORE,
    INIT_SEQ_GIMBAL,
    INIT_SEQ_DONE
} Init_Sequence_State_t;

static Init_Sequence_State_t Init_Sequence_State = INIT_SEQ_IDLE;
uint8_t Init_Sequence_Trigger = 0;
/* =========================================== */

void Servo_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    for (i = 0; i < Servo_Motor_Number; i++)
    {
        if (Servo_Motor_Handle[i].Motor_Type == Servo)
        {
            Servo_Motor_Set_Angle(&Servo_Motor_Handle[i]);
        }
        HAL_Delay(Servo_Control_Cycle);
    }
}

void LK4005_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    
    /* 预先读取大臂、小臂电机当前输出轴角度（由 FDCAN 中断持续更新） */
    float upper_motor_angle = LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
    float fore_motor_angle  = LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        LK4005_Motor_Read_Position(LK4005_Motor_Handle[i]);
        HAL_Delay(1); /* 微小延迟完成FDCAN报文发送 */
        LK4005_Motor_Read_Velocity(LK4005_Motor_Handle[i]);
        HAL_Delay(1); /* 微小延迟完成FDCAN报文发送 */
        if (LK4005_Motor_Handle[i].Wait_Count >= 20)
        {
            /* ---------- 大臂 (Joint_Upper) ---------- */
            if (LK4005_Motor_Handle[i].Motor_Type == Joint_Upper)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle,
                                  LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual,
                                  LK4005_Motor_Handle[i].Motor_Position_Target, LK4005_Motor_Handle[i].Motor_Type);
                if (LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial +
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag *
                            LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.s;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Velocity_Target =
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag *
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.v;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Torque_Feedforward =
                        Upperarm_Gravity_Compensation(upper_motor_angle, fore_motor_angle, Servo_Motor_Handle[1].Motor_Position);
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);
                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);
                }
                else
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Position_Target;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Velocity_Target = 0.0f;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Torque_Feedforward =
                        Upperarm_Gravity_Compensation(upper_motor_angle, fore_motor_angle, Servo_Motor_Handle[1].Motor_Position);
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);
                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);
                }
            }
            /* ---------- 小臂 (Joint_Fore) ---------- */
            if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle,
                                  LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual,
                                  LK4005_Motor_Handle[i].Motor_Position_Target, LK4005_Motor_Handle[i].Motor_Type);
                if (LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial +
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag *
                            LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.s;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Velocity_Target =
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag *
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.v;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Torque_Feedforward =
                        Forearm_Gravity_Compensation(upper_motor_angle, fore_motor_angle, Servo_Motor_Handle[1].Motor_Position);
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);
                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);
                }
                else
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Position_Target;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Velocity_Target = 0.0f;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Torque_Feedforward =
                        Forearm_Gravity_Compensation(upper_motor_angle, fore_motor_angle, Servo_Motor_Handle[1].Motor_Position);
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);
                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);
                }
            }
            /* ---------- 云台 (Gimbal) ---------- */
            if (LK4005_Motor_Handle[i].Motor_Type == Gimbal)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle,
                                  LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual,
                                  LK4005_Motor_Handle[i].Motor_Position_Target, LK4005_Motor_Handle[i].Motor_Type);
                if (LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial +
                        LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag *
                            LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.s;
                }
                else
                {
                    LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Target =
                        LK4005_Motor_Handle[i].Motor_Position_Target;
                }
                LK4005_Motor_Position_Control(LK4005_Motor_Handle[i]);
            }
            HAL_Delay(LK4005_Motor_Control_Cycle);
        }
    }
}

static uint8_t Is_Motor_Arrived(uint8_t idx, float pos_thr)
{
    if (idx == 0) /* 云台 */
    {
        return (LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State == idle) &&
               (fabsf(LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual -
                      LK4005_Motor_Handle[0].Motor_Position_Target) <= pos_thr);
    }
    else /* 大臂或小臂 */
    {
        return (LK4005_Motor_Handle[idx].Motor_Speed_Plan_Handle.Speed_Plan_State == idle) &&
               (fabsf(LK4005_Motor_Handle[idx].Motor_MIT_Control_Handle[0].Motor_Position_Actual -
                      LK4005_Motor_Handle[idx].Motor_Position_Target) <= pos_thr);
    }
}

static void Check_And_Send_Feedback(void)
{
    extern uint8_t Feedback_Pending;
    
    if (Feedback_Pending == 0) return;
    
    if (Is_Motor_Arrived(0, 0.05f) && Is_Motor_Arrived(1, 0.05f) && Is_Motor_Arrived(2, 0.05f))
    {
        if (Feedback_Pending == 2)
        {
            Communication_Send_Move_Success();
        }
        Feedback_Pending = 0;
    }
}

void Robotic_Arm_Control_Init(void)
{
    Motor_Control_Init();
    HAL_Delay(1500);
    /* LK4005电机无需专门使能帧，直接开始发控制帧即可 */
    Communication_Usart_Init();
}

void Robotic_Arm_Control(void)
{
    static uint8_t System_Init_Done = 0;
    
    /* 上电初始化：大臂和小臂保持位置，云台转到90° */
    if (!System_Init_Done)
    {
        /* 尽早设置云台目标为90°，不等大臂小臂锁定，避免云台先向0°跑 */
        if (Gimbal_Start_Complete == 0 && LK4005_Motor_Handle[0].Wait_Count >= 15)
        {
            LK4005_Motor_Handle[0].Motor_Position_Target = PI / 2.0f; /* 云台转到90° */
            LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            Gimbal_Start_Complete = 1;
        }
        
        if (!Upper_Lock_Done && LK4005_Motor_Handle[1].Wait_Count >= 15)
        {
            LK4005_Motor_Handle[1].Motor_Position_Target = LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
            Upper_Lock_Done = 1;
        }
        if (!Fore_Lock_Done && LK4005_Motor_Handle[2].Wait_Count >= 15)
        {
            LK4005_Motor_Handle[2].Motor_Position_Target = LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].Motor_Position_Actual;
            Fore_Lock_Done = 1;
        }
        
        if (Gimbal_Start_Complete == 1)
        {
            /* 必须云台到达90° 且 大臂小臂都锁定后，才算初始化完成 */
            if (Is_Motor_Arrived(0, 0.1f) && Upper_Lock_Done && Fore_Lock_Done)
            {
                Gimbal_Start_Complete = 2;
                System_Init_Done = 1;
                Servo_Control_Active = 1;  /* 初始化完成后，允许舵机跟踪目标值 */
            }
        }
    }
    
    /* FF初始化命令顺序控制：先大臂 → 再小臂 → 最后云台 */
    if (Init_Sequence_Trigger)
    {
        Init_Sequence_Trigger = 0;
        Init_Sequence_State = INIT_SEQ_UPPER;
    }
    
    if (Init_Sequence_State != INIT_SEQ_IDLE)
    {
        switch (Init_Sequence_State)
        {
        case INIT_SEQ_UPPER:
            if (Is_Motor_Arrived(1, 0.05f))
            {
                /* 大臂到达，启动小臂到3.5rad */
                LK4005_Motor_Handle[2].Motor_Position_Target = 3.5f;
                LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
                Init_Sequence_State = INIT_SEQ_FORE;
            }
            break;
        case INIT_SEQ_FORE:
            if (Is_Motor_Arrived(2, 0.2f))
            {
                /* 小臂到达，启动云台归位到0° */
                LK4005_Motor_Handle[0].Motor_Position_Target = 0.0f;
                LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
                Init_Sequence_State = INIT_SEQ_GIMBAL;
            }
            break;
        case INIT_SEQ_GIMBAL:
            if (Is_Motor_Arrived(0, 0.05f))
            {
                Init_Sequence_State = INIT_SEQ_DONE;
            }
            break;
        case INIT_SEQ_DONE:
            Communication_Send_Init_Success();
            Init_Sequence_State = INIT_SEQ_IDLE;
            break;
        default:
            break;
        }
    }
    
    if (Servo_Control_Active)
    {
        Servo_Motor_Handle_Update();
    }
    LK4005_Motor_Handle_Update();
    
    Check_And_Send_Feedback();
}
