# Quadcopter_BareMetal — MPU6050

独立裸机工程，原 RTOS 工程在 C:/Project/Quadcopter_FreeRTOS。当前版本使用板上的
MPU6050（I2C2 + DMA）。此前的 ICM42688-P 版本因芯片无法正常使用已撤回，驱动不再留在
树里，可在提交 f89e0b6 找到。

## 接线

以《飞控原理图》2026-09-01 的网表（netlist.enet）为准：

| 模块 | 信号 | STM32 引脚 |
| --- | --- | --- |
| MPU6050 | SCL / SDA | PB10 / PB11（I2C2，板上 2.2 kΩ 上拉，AD0 接地，地址 0x68） |
| MPU6050 | INT | PB0（`GYRO_INT`，EXTI0，上升沿） |
| nRF24 | SCK / MISO / MOSI | PA5 / PA6 / PA7（SPI1） |
| nRF24 | CSN / CE / IRQ | PA4 / PB14 / PB15（EXTI15_10，下降沿） |
| OLED | SCL / SDA | PB8 / PB9（I2C1 重映射） |
| 四路电机 | TIM2 CH1/2/3/4 | PA0 / PA1 / PA2 / PA3 |
| 电池电压分压 | ADC | PB1（只配成模拟输入，应用代码暂未使用） |

PB12、PB13 悬空。ICM42688 版本曾把 nRF 的 CE/IRQ 挪到 PB10/PB11，并用 PB12~PB15 做
SPI2；换回 MPU6050 后全部还原为原理图的接法。如果实物上还留着那次的飞线，需要先恢复，
否则 PB10/PB11 上 I2C2 会与 nRF 冲突。

电机插座的标号和固件里的电机编号不一致：板上插座 M1/M2/M3/M4 依次由 PWM1/PWM4/PWM2/
PWM3 驱动；固件里的 M1..M4 指 PWM1..PWM4（左前、右前、右后、左后，见 mixer.c）。接电机
和拆桨核对时按 PWM 通道对应，不要按插座标号。每路 MOS 管栅极有 100 Ω 串联电阻和
10 kΩ 下拉。

I2C2 RX 使用 DMA1 Channel5，OLED 的 I2C1 TX 使用 Channel6。I2C2 与 I2C1 是两个外设，
OLED 整屏 DMA 传输不会占用 IMU 总线。I2C 时钟 400 kHz。

外设、引脚、DMA、中断优先级只在 Quadcopter_BareMetal.ioc 里配置，由 CubeMX 生成 Core/
下的代码；应用代码通过生成的 main.h 标签（GYRO_INT_Pin、NRF_CE_Pin、NRF_IRQ_Pin 等）
引用引脚，改引脚只需要改 .ioc 并重新生成。用 CubeMX 6.15 命令行重新生成时会丢弃旧的 ADC1
配置（原 RTOS 工程的 .ioc 也一样），所以 ADC1 目前没有生成；需要电池电压时在 CubeMX
里重新启用 ADC1 的 IN9。

加速度计零位偏移沿用旧 MPU6050 驱动里在机体水平静止时测得的值（X +0.104 g、
Y +0.028 g、Z −0.068 g）；换过模块需要重测。当前直接使用芯片坐标，PCB 的安装方向
必须按实际核对机体坐标映射（见下面第 11 条）。

## 数据与控制流程

1. 上电先清零电机，MPU6050 设备复位并等待 100 ms，检查 WHO_AM_I=0x68；随后逐项写入并
   回读核对：时钟（PLL，陀螺仪 X）、SMPLRT_DIV=0、CONFIG=3（DLPF 42 Hz，陀螺仪输出
   1 kHz）、陀螺仪 ±500 dps、加速度计 ±8 g。
2. 数据就绪中断：INT_PIN_CFG=0x10（推挽、高有效、50 µs 脉冲、任意读清状态），
   INT_ENABLE 打开 DATA_RDY；开放 DRDY 前先读一次 INT_STATUS 清掉陈旧位。
3. DRDY EXTI 只置 MPU6050_Ready，主循环取走标志并启动 I2C2 DMA：从 INT_STATUS 起连续
   读 15 字节（状态 1、加速度 6、温度 2、陀螺仪 6）。状态放进同一次突发读，不需要额外的
   阻塞读就能判断样本是否新鲜：DATA_RDY 位没置位说明没有新样本，该帧既不计数也不算错误。
4. DMA 完成回调标记完成；主循环消费完整缓冲区。DMA 活动或上一帧未消费时不能覆盖缓冲区。
   错误或超过 5 ms 的帧被丢弃，并通过 HAL_I2C_DeInit/Init 复位 I2C2（同时关掉 DMA）。
   加速度或陀螺仪出现 −32768 的帧按异常数据丢弃。
5. 前 1000 个有效完整帧求陀螺仪零偏均值，后续逐帧扣除；校准期间应保持静止。
   用加速度初始化姿态，再平均 500 帧获得 roll/pitch 水平零点。
   没有额外的 500 帧等待阶段。两项校准完成前禁止电机控制。
6. 后续完整帧执行既有 Mahony 姿态融合、飞控 PID、混控、电机输出；
   遥控必须满足现有连接及低油门解锁条件。PID 公式未改；俯仰混控符号与速率环 Ki 见第 10、11 条。
7. 遥控 EXTI 置 Remote_Ready，主循环调用原接收解析流程。
8. OLED 使用 100 ms 周期，校准后更新参数页。绘图和传输缓冲分离，
   OLED_Poll 推进 I2C1 DMA，不等待整屏传输，失败或超时重置显示链路。
9. 无遥控时停机；IMU 传输错误、异常数据或 20 ms 无有效帧时停机。
   HAL TIM4 tick 优先级设为 4，高于 SPI/I2C 的 5/6，避免 HAL 中断内异常
   等待时 tick 无法推进。

10. 控制器：外环姿态 P（Kp=15），内环速率环 PID（Kp=3，Ki=5，Kd=0）。速率环积分
    限幅 ±100 计数，油门处于怠速（≤20）时每帧清零，避免在地上积分饱和。电机不一致
    （如某个电机弱 20%）需要稳定的力矩补偿，纯 P 会留下稳态倾斜和偏航漂移，Ki 用于消除。
    仍可用遥控参数槽在线改增益，改增益时对应积分清零。
11. 俯仰混控符号：机体系 x 前、y 左、z 上，右手系，所以 pitch>0 表示机头下俯，
    pitch 修正为正时应加速后电机（与 roll 列方向相反）。混控符号必须使每个环的输出
    把自己的测量角度向上推，否则是正反馈。此项已用闭环仿真验证，但 IMU 安装方向未定，
    **拆桨后必须手动核对**：机头下倾时前两个电机（PWM1、PWM2）应加速，一侧下沉时
    下沉侧电机应加速。原 RTOS 工程里俯仰列的符号与此相反，两版必有一版与实物不符，
    以这项台架核对为准；不符就翻转 mixer.c 的 mixer_pitch_sign。
12. 安全：独立看门狗 IWDG（内部 LSI，不占用定时器，约 120 ms 超时，调试暂停时冻结），
    主循环每圈喂狗，主循环卡死会复位。Error_Handler 和各 Fault 异常处理先直接清零
    TIM2 比较寄存器再停机，复位后 TIM2 也随之复位。复位期间 PA0..PA3 为浮空输入，
    原理图里每路 MOS 管栅极都有 10 kΩ 下拉，不会误导通。

未用 FIFO，延迟期间的历史样本无法补回。融合与既有控制仍以 1 ms 设计，
上板必须检查持续运行时能否跟上 1 kHz。调试计数器：
g_imu_drdy_count（中断）、g_imu_frame_count（有效帧）、
g_imu_coalesced_count（标志未消费时的新中断）、g_imu_error_count（读取/消费错误）。
这些计数器用于诊断，不是实测实时性保证。

## 验证

2026-10-03：Debug/Release 清理编译通过，无编译警告（CMake + arm-none-eabi-gcc）。
Debug RAM 5488 B、Flash 48 KB；Release RAM 5488 B、Flash 31152 B。

主机测试命令：powershell -ExecutionPolicy Bypass -File verification/run_mpu_tests.ps1
（可用 -Compiler 指定本机 GCC。）

测试使用实际 MPU6050 驱动、主循环、遥控解析、姿态/控制和 OLED 代码，
模拟 MPU6050 寄存器和 HAL 外设。覆盖身份/寄存器错误、回读失败、1 kHz 配置、
换算系数和轴序、DMA 缓冲所有权、校准样本计数、零偏消除、陈旧 INT_STATUS 帧忽略、
异常/过期帧拒绝、传输错误和超时后的 I2C2 复位、IMU 停更、遥控失联/重新解锁以及
OLED 快照/完成/错误/超时流程。运行脚本还会核对：main.c 的回调与测试用的副本一致，
测试桩里的引脚标签与 CubeMX 生成的 main.h 一致。
test_control_sign.c 是控制方向的闭环仿真：用真实的姿态、飞控和混控代码，
配刚体模型，检查首次修正方向、三轴扰动衰减、电机不一致被 Ki 补偿、怠速不积分。
惯量和推力是小型有刷机的示意值，只验证符号和收敛，不是整定依据。
原 test_baremetal.c、test_zero.c 是更早的 MPU6050 版本历史测试，不是本版本入口。

未烧录、未上板。软件测试不能证明 I2C 信号质量、DRDY 实际频率、安装轴向、
控制执行时间或实飞效果。上板先拆桨核对这些项目。
