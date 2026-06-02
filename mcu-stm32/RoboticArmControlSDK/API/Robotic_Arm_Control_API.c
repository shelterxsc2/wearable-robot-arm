#include "Robotic_Arm_Control_API.h"

static uint8_t Gimbal_Start_Complete = 0;
static uint8_t Joint_Upper_Start_Complete = 0;
static uint8_t Joint_Fore_Start_Complete = 0;
static uint8_t Upper_Lock_Done = 0;
static uint8_t Fore_Lock_Done = 0;

/* 小臂启动完成后置 1，控制舵机何时开始跟踪目标值 */
uint8_t Servo_Control_Active = 0;

/* ========== 自动测试状态机 ========== */
uint8_t Test_Mode_Active = 0;

/* 测试点：距离较远的四个角（单位：m，舵机固定在小臂延长线上 phi_servo=0） */
static const float Test_Point[4][4] = {
    {0.02f, 0.60f, 0.45f, 0.0f},   /* 右上  [0] */
    {0.02f, 0.60f, 0.47f, 0.0f},   /* 右下  [1] */
    {-0.02f, 0.60f, 0.47f, 0.0f},  /* 左下  [2] */
    {-0.02f, 0.60f, 0.45f, 0.0f}   /* 左上  [3] */
};

#define TEST_POS_THR 0.05f
#define TEST_CYCLES 3

typedef enum
{
    TEST_IDLE,
    TEST_MOVE,
    TEST_WAIT,
    TEST_DONE
} Test_State_t;

static Test_State_t Test_State = TEST_IDLE;
static uint8_t Test_Point_Idx = 0;
static uint8_t Test_Cycle_Count = 0;
static uint32_t Test_Wait_Tick = 0;

static void Test_Set_Target(uint8_t idx)
{
    float gimbal, upper, fore;
    Coordinate_Inverse_Settlement(Test_Point[idx][0], Test_Point[idx][1],
                                  Test_Point[idx][2], Test_Point[idx][3],
                                  &gimbal, &upper, &fore);
    LK4005_Motor_Handle[0].Motor_Position_Target = gimbal;        /* 云台：直接相连 */
    LK4005_Motor_Handle[1].Motor_Position_Target = -4.0f * upper; /* 大臂：电机轴 = -4×关节角 */
    LK4005_Motor_Handle[2].Motor_Position_Target =  2.0f * fore;  /* 小臂：电机轴 =  2×关节角 */
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
    LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
}

static uint8_t Test_Is_All_Stopped(void)
{
    uint8_t upper_done =
        (LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State == idle) &&
        (fabsf(LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Actual -
               LK4005_Motor_Handle[1].Motor_Position_Target) <= TEST_POS_THR);
    uint8_t fore_done =
        (LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State == idle) &&
        (fabsf(LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].Motor_Position_Actual -
               LK4005_Motor_Handle[2].Motor_Position_Target) <= TEST_POS_THR);
    uint8_t gimbal_done =
        (LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State == idle) &&
        (fabsf(LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual -
               LK4005_Motor_Handle[0].Motor_Position_Target) <= TEST_POS_THR);
    return upper_done && fore_done && gimbal_done;
}

static void Test_Sequence_Run(void)
{
    if (!Test_Mode_Active)
        return;
    if (Joint_Fore_Start_Complete != 2)
        return;
    switch (Test_State)
    {
    case TEST_IDLE:
        Test_Point_Idx = 0;
        Test_Cycle_Count = 0;
        Test_State = TEST_MOVE;
        break;
    case TEST_MOVE:
        Test_Set_Target(Test_Point_Idx);
        Test_State = TEST_WAIT;
        break;
    case TEST_WAIT:
        if (Test_Is_All_Stopped())
        {
            Test_Wait_Tick = 0;
            Test_Point_Idx++;
            if (Test_Point_Idx >= 4)
            {
                Test_Point_Idx = 0;
                Test_Cycle_Count++;
                if (Test_Cycle_Count >= TEST_CYCLES)
                {
                    Test_State = TEST_DONE;
                    break;
                }
            }
            Test_State = TEST_MOVE;
        }
        break;
    case TEST_DONE:
        Test_Mode_Active = 0;
        Test_State = TEST_IDLE;
        break;
    default:
        break;
    }
}
/* =================================== */

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
                                  LK4005_Motor_Handle[i].Motor_Position_Target);
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
                if (fabsf(LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual -
                          LK4005_Motor_Handle[i].Motor_Position_Target) <= 0.1f &&
                    Joint_Upper_Start_Complete == 0 && Gimbal_Start_Complete == 2)
                {
                    Joint_Upper_Start_Complete = 1;
                }
            }
            /* ---------- 小臂 (Joint_Fore) ---------- */
            if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle,
                                  LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual,
                                  LK4005_Motor_Handle[i].Motor_Position_Target);
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
                if (fabsf(LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Actual -
                          LK4005_Motor_Handle[i].Motor_Position_Target) <= 0.1f &&
                    Joint_Fore_Start_Complete == 0 && Joint_Upper_Start_Complete == 2)
                {
                    Joint_Fore_Start_Complete = 1;
                }
            }
            /* ---------- 云台 (Gimbal) ---------- */
            if (LK4005_Motor_Handle[i].Motor_Type == Gimbal)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle,
                                  LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual,
                                  LK4005_Motor_Handle[i].Motor_Position_Target);
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
                if (fabsf(LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Actual -
                          LK4005_Motor_Handle[i].Motor_Position_Target) <= 0.1f &&
                    Gimbal_Start_Complete == 0)
                {
                    Gimbal_Start_Complete = 1;
                }
            }
            HAL_Delay(LK4005_Motor_Control_Cycle);
        }
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
    /* 阶段0: 云台先启动，大臂和小臂锁定当前位置 */
    if (Gimbal_Start_Complete == 0)
    {
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
    }
    else if (Gimbal_Start_Complete == 1)
    {
        LK4005_Motor_Handle[1].Motor_Position_Target = -6.28f; /* 大臂电机轴角度目标 */
        LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
        Gimbal_Start_Complete = 2;
    }
    /* 阶段1: 大臂启动，小臂锁定当前位置 */
    if (Joint_Upper_Start_Complete == 0)
    {
        if (!Fore_Lock_Done && LK4005_Motor_Handle[2].Wait_Count >= 15)
        {
            LK4005_Motor_Handle[2].Motor_Position_Target = LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].Motor_Position_Actual;
            Fore_Lock_Done = 1;
        }
    }
    else if (Joint_Upper_Start_Complete == 1)
    {
        LK4005_Motor_Handle[2].Motor_Position_Target = 3.14f; /* 小臂电机轴角度目标 */
        LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
        Joint_Upper_Start_Complete = 2;
    }
    /* 阶段2: 小臂启动 */
    if (Joint_Fore_Start_Complete == 1)
    {
        Joint_Fore_Start_Complete = 2;
        Servo_Control_Active = 1;  /* 小臂启动完成后，才允许舵机跟踪目标值 */
        Test_Mode_Active = 1;      /* 小臂启动完成后，才启动测试模式 */
    }

    /* ---------- 四角循环测试 ---------- */
    Test_Sequence_Run();

    if (Servo_Control_Active)
    {
        Servo_Motor_Handle_Update();
    }
    LK4005_Motor_Handle_Update();
    Communication_Test();
}
