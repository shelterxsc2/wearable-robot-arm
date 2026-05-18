# STM32 机械臂下位机

## 占位说明

本目录用于存放 STM32 裸机开发的机械臂控制代码。

## 预期内容

- 电机驱动（PWM / 步进 / 舵机）
- LQR 控制算法实现
- UART 通信（接收上位机目标位姿，回传当前位姿）
- 编码器读取 / 状态估计
- 安全限位、急停逻辑

## 如何从 Windows 导入

1. 在 Windows 上找到 Keil / STM32CubeIDE 工程目录
2. 将整个工程目录（建议去掉 `Debug/`、`Release/`、`*.uvguix.*` 等编译产物）复制到本目录
3. 建议保留原始目录结构，例如：

```
mcu-stm32/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── Middlewares/ (if any)
├── *.ioc
├── Makefile / *.uvprojx
└── README.md
```

4. 导入后更新本 README，补充编译和烧录说明

## 通信接口

下位机需实现 `shared/protocol.h` 中定义的 UART 帧格式：

```
[0xAA][0x55][LEN][CMD][payload...][CRC8]
```

- **接收**: `CMD_TARGET_POSE` (0x10) — 上位机下发的目标位姿
- **发送**: `CMD_CURRENT_POSE` (0x20) — 下位机回传的当前末端位姿

