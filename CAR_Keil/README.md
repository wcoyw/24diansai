# CAR — MSPM0G3507 移植工程 (逐飞库 + Keil MDK)

2024 电赛 H 题 STM32F103 → MSPM0G3507 | 已配置: 32MHz SYSOSC, Keil MDK 5.37

## 一分钟上手

1. **双击打开**: `project\mdk\CAR.uvprojx`
2. **编译**: 按 F7 或点击 Build
3. **下载**: 点击 Download (F8)

> 工程已包含所有逐飞库源文件, Include Path 已配置, 不需要额外操作。

## 文件结构

```
CAR_Keil/
├── libraries/                         # 逐飞开源库 (已拷贝, 已改32MHz)
│   ├── zf_common/                     # 时钟/中断/调试
│   ├── zf_driver/                     # GPIO/PWM/UART/PIT/EXTI/SoftIIC
│   ├── zf_device/                     # 设备驱动
│   └── sdk/                           # TI SDK + DriverLib + CMSIS
├── project/
│   ├── mdk/
│   │   ├── CAR.uvprojx                # ← 双击打开!
│   │   ├── CAR.uvoptx                 # Keil 项目配置
│   │   └── mspm0g3507.sct             # 链接脚本
│   └── user/
│       ├── inc/                       # 头文件 (9 个)
│       │   ├── main.h                 # 32引脚定义 + 所有外设宏
│       │   ├── motor.h, track.h, pid.h
│       │   ├── button.h, led.h, buzzer.h
│       │   └── icm42688.h, imu.h
│       └── src/                       # 源文件 (10 个)
│           ├── main.c                 # 主状态机 + 25Hz PIT 回调
│           ├── motor.c                # TIMA0 PWM + EXTI 编码器
│           ├── track.c                # 7路循迹加权控制
│           ├── pid.c                  # 位置式 PID (与STM32一致)
│           ├── button.c, led.c, buzzer.c
│           ├── icm42688.c             # ICM-42688-P 软I2C驱动
│           ├── imu.c                  # 互补滤波姿态解算
│           └── isr.c                  # 逐飞库 ISR 向量表
└── README.md
```

## 已完成配置

| 项目 | 配置 |
|------|------|
| **时钟** | 32MHz SYSOSC, 无 PLL (已改 `ti_msp_dl_config.c` + `zf_common_clock.c`) |
| **PWM** | TIMA0 CH0(PA21左) + CH1(PA07右), 20kHz |
| **编码器** | 4× GPIO EXTI 双边沿 (PA28/29, PB14/18) |
| **串口** | UART0(PA10/11 调试) + UART1(PA08/09 视觉), 115200 |
| **IMU** | ICM-42688-P I2C(PA15/30), 地址 0x69, 互补滤波 |
| **控制周期** | PIT_TIM_A1 → TIMA1 回调, 25Hz (40ms) |
| **按键** | 非阻塞扫描, 4路独立状态机消抖 |
| **所有引脚** | 严格按 `CAR_引脚配置总表.md` (32/32) |

## ISR 调度链

```
硬件中断               → 逐飞 isr.c            → 回调                    → 本工程处理
─────────────────────────────────────────────────────────────────────────────
TIMA1 (PIT 25Hz)      → TIMA1_IRQHandler     → pit_callback_list[1]   → control_pit_callback()
GPIOA/B EXTI          → GROUP1_IRQHandler    → exti_callback_list[]   → enc_l/r_a/b_callback()
UART0/1               → UART0/1_IRQHandler   → uart_callback_list[]   → (仅用阻塞发送)
```

## 移植关键变化

| 维度 | STM32 原版 | → | MSPM0 新版 |
|------|-----------|---|-----------|
| IMU | JY61P (UART) | → | ICM-42688-P (I2C, 0x69) |
| 编码器 | TIM3/TIM4 QEI | → | 4× GPIO EXTI 双边沿 |
| PWM | TIM2 CH3/CH4 | → | TIMA0 CH0/CH1 (20kHz) |
| 控制定时器 | TIM5 100Hz → 软4分频 | → | PIT_TIM_A1 回调 25Hz |
| 按键 | 主循环阻塞 HAL_Delay | → | 非阻塞状态机 (ISR 中) |
| 库 | STM32 HAL | → | 逐飞 zf_driver |

## 调试顺序建议

1. 编译下载 → LED 是否闪 (启动指示)
2. 串口助手打开 COM 口 (115200) → 应输出 "CAR MSPM0G3507 ready"
3. 按键测试 → 按 SW1~SW4 看串口有无任务触发
4. 电机测试 → 确认 PWM 频率 20kHz, 方向正确
5. 编码器测试 → 转轮子看数值变化
6. IMU 测试 → 串口打印 Roll/Pitch/Yaw

## 注意

- **上电时保持静止**: IMU 初始化需要 100 次采样校准陀螺仪零偏
- **IMU 异常**: 红灯快闪 = ICM-42688-P I2C 通信失败 (检查 PA15/PA30 接线)
- **M0+ 无 FPU**: 浮点运算靠软件, 25Hz 下性能足够, 无需改为定点数
- **编码器方向**: `encoder_get_right()` 已取反, 如需调方向改 `motor.c:200`
