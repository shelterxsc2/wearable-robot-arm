# STM32 机械臂下位机

## 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32H723VGTx (Cortex-M7, 550MHz) |
| 开发环境 | Keil MDK + STM32CubeMX HAL |
| 电机总线 | FDCAN1 + FDCAN2 (经典 CAN 模式) |
| 上位机通信 | USART1 + DMA Idle 中断 |
| 舵机 PWM | TIM1_CH3 + TIM2_CH3 |
| IMU | SPI2 + BMI088 (驱动待实现) |

## 机械臂构型

```
基座(J1/背部)
  │
  │ 连接杆 L_connect = 0.44m
  ▼
云台(Gimbal) ── LK4005 ×1 (位置 PID)
  │
大臂(Joint_Upper) ── DMJ4310 ×1 (MIT 力矩控制)
  │  L1 = 0.45m
  ▼
小臂(Joint_Fore) ── LK4005 ×1 (MIT 力矩控制)
  │  L2 = 0.461m
  ▼
末端法兰 ── 舵机1 ── 末端杆 L_end = 0.08245m ── 舵机0 ── 相机
```

## 工程结构

```
mcu-stm32/
├── Core/                          # CubeMX 生成
│   ├── Src/main.c                 # HAL_Init → 外设初始化 → Robotic_Arm_Control_Init → while(1)
│   └── Inc/                       # HAL 头文件
├── Drivers/                       # CMSIS + STM32H7xx_HAL_Driver
├── MDK-ARM/                       # Keil 工程文件 (.uvprojx)
└── RoboticArmControlSDK/          # 用户业务代码
    ├── API/
    │   └── Robotic_Arm_Control_API.c    # 顶层控制逻辑、启动归位流程
    ├── Core/
    │   ├── Control_Algorithm.c          # S曲线、MIT控制、重力补偿、逆运动学
    │   ├── DMJ4310_Motor_Driver.c/h     # 达妙电机驱动
    │   ├── LK4005_Motor_Driver.c/h      # 瓴控电机驱动
    │   ├── LFD01M_Motor_Driver.c/h      # 飞特舵机PWM驱动
    │   └── BMI088_IMU_Driver.c/h        # IMU驱动（目前只有空壳头文件）
    ├── Config/
    │   └── Robotic_Arm_Config.h         # 结构体定义、机械参数、宏
    └── Port/
        ├── Robotic_Arm_Motor_HAL_STM32_Port.c      # FDCAN发送、Float/Uint转换、FIFO中断
        └── Robotic_Arm_Communication_HAL_STM32_Port.c  # UART DMA初始化、Idle中断解析
```

## 控制算法

| 算法 | 文件 | 说明 |
|------|------|------|
| **S 曲线速度规划** | `Control_Algorithm.c` | 7 阶段状态机（idle→init→phase1~7），含加加速度 j、最大加速度 a_max、最大速度 v_max |
| **MIT 力矩控制** | `Control_Algorithm.c` | `Output = Kp·Δpos + Kd·Δvel + Feedforward + Friction` |
| **重力补偿** | `Control_Algorithm.c` | 根据大臂/小臂角度实时计算重力矩，补偿给关节电机 |
| **逆运动学** | `Control_Algorithm.c` | `Coordinate_Inverse_Settlement(X, Y, Z, Servo_Angle, ...)` |

## 启动归位流程

1. 大臂 DMJ4310 使能，Wait_Count ≥ 20 后进入闭环，目标 `-0.2 rad`
2. 到位后自动切目标 `1.57 rad`
3. 小臂 LK4005 锁定当前位置 → 到位后切 `1.57 rad` + 圈数对齐
4. 云台 LK4005 锁定当前位置 → 到位后切目标角度
5. `Gimbal_Start_Complete = 1`，舵机开始响应 PWM

## 电机配置

### DMJ4310（大臂）
- FDCAN2，ID=1
- MIT 控制：`Kp=62.8, Kd=1.3`
- S 曲线：`j=15.5, a_max=1.5, v_max=0.45`

### LK4005（云台 + 小臂）
- 云台：FDCAN1，ID=0x14C，位置 PID 控制（`0xA3`）
- 小臂：FDCAN2，ID=0x149，MIT 力矩控制（`0xA1`）
- 小臂 MIT：`Kp=100.5/105.0, Kd=3.95/1.255`

### LFD01M（舵机 ×2）
- 舵机0（上方，连摄像头）：TIM1_CH3
- 舵机1（下方，连小臂）：TIM2_CH3
- PWM 范围：500~2500 μs

## 通信接口

当前实现：**USART1 DMA Idle 中断**，接收 **10 字节简化 raw 格式**：

```
[Byte0~1] X  (int16, cm, 小端)
[Byte2~3] Y  (int16, cm, 小端)
[Byte4~5] Z  (int16, cm, 小端)
[Byte6~7] Servo1 (int16, °, 小端)
[Byte8~9] Servo2 (int16, °, 小端)
```

> ⚠️ 此格式与 `shared/protocol.h` 中定义的标准帧格式 `[0xAA][0x55][LEN][CMD][payload][CRC8]` **不一致**。详见根目录 [`docs/architecture.md`](../docs/architecture.md)。

## 导入说明（从 Windows Keil 工程）

若后续在 Windows 上修改后需要同步回仓库：
1. 复制 `Core/Src`、`Core/Inc`、`RoboticArmControlSDK/` 下的改动文件
2. 不要提交 `Debug/`、`Release/`、`*.uvguix.*` 等编译产物
3. 更新本 README，补充新的编译/烧录说明
