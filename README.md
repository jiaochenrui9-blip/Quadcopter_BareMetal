# Quadcopter_BareMetal — ICM42688-P

独立裸机工程，原 RTOS 工程在 C:/Project/Quadcopter。当前版本用 ICM42688-P
替换 MPU6050，使用独立 SPI2 DMA。旧 mpu6050.c/.h 保留作参考，不参与构建。

## 接线

| 模块 | 信号 | STM32 引脚 |
| --- | --- | --- |
| ICM42688-P | SCLK / SDO / SDI | PB13 / PB14 / PB15（SPI2） |
| ICM42688-P | CS / INT1 | PB12 / PB0（EXTI0，上升沿） |
| nRF24 | SCK / MISO / MOSI | PA5 / PA6 / PA7（SPI1） |
| nRF24 | CSN / CE / IRQ | PA4 / PB10 / PB11（EXTI11，下降沿） |
| OLED | SCL / SDA | PB8 / PB9（I2C1 重映射） |
| 四路电机 | TIM2 CH1/2/3/4 | PA0 / PA1 / PA2 / PA3 |

取消 I2C2，以释放 PB10/PB11。SPI2 RX/TX 分别使用 DMA1 Channel4/5，
OLED TX 使用 Channel6。SPI2 总线时钟为 4.5 MHz（APB1 36 MHz / 8），
它与传感器的 1000 Hz 采样率是不同参数。

ICM42688 的 VDD/VDDIO 接 3.3 V，供电去耦、保留脚和未用 FSYNC 按
DS-000347 v1.9 第 20–22 页处理。当前直接使用芯片坐标，PCB 定稿后必须
按实际安装方向核对机体坐标映射。旧 MPU6050 的加速度计偏移未移植。

## 数据与控制流程

1. 上电先清零电机，检查 WHO_AM_I=0x47，软复位并核对寄存器写入。
2. 加速度计 ±8 g、陀螺仪 ±500 dps，两者 1 kHz、低噪声模式；
   UI 低通带宽 100 Hz，FIFO bypass，INT1 高有效推挽脉冲。
   按手册清除 INT_ASYNC_RESET。上电启动等待后才开放 DRDY。
3. DRDY EXTI 只置 ICM42688_Ready，主循环取走标志并启动 SPI2 双向 DMA。
   命令加 14 字节数据总计 15 字节；第一字节接收结果丢弃。
   状态寄存器检查使用短轮询，传感器数据使用 DMA。
4. DMA 完成回调释放 CS 并标记完成；主循环消费完整缓冲区。
   DMA 活动或上一帧未消费时不能覆盖缓冲区。错误或超过 5 ms 的帧被丢弃并中止传输。
5. 前 1000 个有效完整帧求陀螺仪零偏均值，后续逐帧扣除；校准期间应保持静止。
   用加速度初始化姿态，再平均 500 帧获得 roll/pitch 水平零点。
   没有额外的 500 帧等待阶段。两项校准完成前禁止电机控制。
6. 后续完整帧执行既有 Mahony 姿态融合、飞控 PID、混控、电机输出；
   遥控必须满足现有连接及低油门解锁条件。PID 和混控公式未改。
7. 遥控 EXTI 置 Remote_Ready，主循环调用原接收解析流程。
8. OLED 使用 100 ms 周期，校准后更新参数页。绘图和传输缓冲分离，
   OLED_Poll 推进 I2C1 DMA，不等待整屏传输，失败或超时重置显示链路。
9. 无遥控时停机；IMU 传输错误、异常数据或 20 ms 无有效帧时停机。
   HAL TIM4 tick 优先级设为 4，高于 SPI/I2C 的 5/6，避免 HAL 中断内异常
   等待时 tick 无法推进。

未用 FIFO，延迟期间的历史样本无法补回。融合与既有控制仍以 1 ms 设计，
上板必须检查持续运行时能否跟上 1 kHz。调试计数器：
g_imu_drdy_count（中断）、g_imu_frame_count（有效帧）、
g_imu_coalesced_count（标志未消费时的新中断）、g_imu_error_count（读取/消费错误）。
这些计数器用于诊断，不是实测实时性保证。

## 验证

2026-09-09：Debug/Release 清理编译通过，无编译警告。
Debug RAM 5624 B；Release RAM 5616 B。Flash 占用见最新 build-icm-*.log。

主机测试命令：powershell -ExecutionPolicy Bypass -File verification/run_icm_tests.ps1
（可用 -Compiler 指定本机 GCC。）

测试使用实际 ICM42688 驱动、主循环、遥控解析、姿态/控制和 OLED 代码，
模拟寄存器和 HAL 外设。覆盖身份/寄存器错误、1 kHz 配置、DMA 缓冲所有权、
校准样本计数、零偏消除、未完成/过期帧拒绝、传输错误和超时、
IMU 停更、遥控失联/重新解锁以及 OLED 快照/完成/错误/超时流程。
原 test_baremetal.c、test_zero.c 是 MPU6050 版本历史测试，不是本版本入口。

未烧录、未接新板。软件测试不能证明 SPI 信号质量、DRDY 实际频率、
安装轴向、控制执行时间或实飞效果。新板先拆桨核对这些项目。
