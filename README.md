<div align="center">

# 🌊 Hmini 315 遥控链路

**将 315 MHz 接收器的遥控数据，经 Arduino Mega 2560 转换为标准 SBUS，并送入 PX4。**

![Status](https://img.shields.io/badge/通信链路-已打通-2ea44f?style=for-the-badge)
![Arduino](https://img.shields.io/badge/Arduino-Mega%202560-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-SBUS-8A2BE2?style=for-the-badge)
![RF](https://img.shields.io/badge/RF-315%20MHz-F59E0B?style=for-the-badge)

</div>

---

## 📖 项目简介

本项目实现了一条完整的遥控数据传输链路：

```mermaid
flowchart LR
    A["🎮 遥控手柄"] -->|"315 MHz"| B["📡 接收器"]
    B -->|"自定义 UART<br/>9600 baud · 8N1"| C["🧠 Arduino Mega 2560"]
    C -->|"标准 SBUS<br/>100000 baud · 8E2"| D["✈️ PX4"]
    D --> E["🖥️ QGroundControl"]

    style A fill:#fff3cd,stroke:#f59e0b,color:#1f2937
    style B fill:#dbeafe,stroke:#3b82f6,color:#1f2937
    style C fill:#ccfbf1,stroke:#00979d,color:#1f2937
    style D fill:#ede9fe,stroke:#8b5cf6,color:#1f2937
    style E fill:#dcfce7,stroke:#22c55e,color:#1f2937
```

Arduino 负责接收并校验接收器发出的 24 字节数据帧，完成解扰、10 路通道解析和数值映射，再以标准 SBUS 持续转发给 PX4。链路断开时，Arduino 会主动发送带 `failsafe` 标志的安全帧。

> [!IMPORTANT]
> 当前代码默认运行在**台架测试模式**：接收器 → Arduino → PX4 的通信链路已经打通，但水桨、ESC、舵机、IMU、深度计和 SD 日志仍保持禁用。完成动力映射及安全检查前，请勿关闭台架模式。

## ✅ 当前进度

| 模块 | 状态 | 说明 |
|---|:---:|---|
| 315 MHz 接收器 UART 接收 | ✅ | `Serial3`，9600 baud，8N1 |
| 帧同步、CRC-8 校验与 BD/DB 解扰 | ✅ | 坏帧不会覆盖最近一次有效通道 |
| 10 路遥控通道解析 | ✅ | 4 路 11 位摇杆 + 6 路 6 位辅助通道 |
| Arduino → PX4 标准 SBUS | ✅ | `Serial2`，100000 baud，8E2，约 71.4 Hz |
| PX4 / QGroundControl 通道验证 | ✅ | 可通过 `input_rc` 观察通道和失控状态 |
| 200 ms 断联检测与 SBUS Failsafe | ✅ | 油门及 AUX 置低，姿态通道回中 |

## 🔌 硬件连接

| 起点 | 终点 | 信号 / 参数 |
|---|---|---|
| 315 接收器 `OUT` | Mega `D15 / RX3` | UART，9600 baud，8N1 |
| Mega `D16 / TX2` | 板卡 `SOUT` | 未反相 SBUS UART，100000 baud，8E2 |
| 板卡 `SOUT` | PX4 `RC/SBUS IN` | 经过反相与 3.3 V 电平适配的标准 SBUS |
| Arduino `GND` | 接收器及 PX4 `GND` | 必须共地 |
| Mega USB | 电脑 | 调试串口，115200 baud |

> [!CAUTION]
> 标准 SBUS 使用**反相逻辑**。Arduino Mega 的硬件 UART 不会自动反相，因此 `D16 / TX2` 应经过板卡的 `SOUT` 反相和电平适配电路后再连接 PX4，不能默认将裸 UART TX 直接接入 PX4 的 SBUS 引脚。

## 📦 数据链路

### 接收器 → Arduino

接收器输出固定 24 字节帧：

```text
00 | AD AD AD AD AD | FE | 10 | CRC | Data'[12] | BD DB BD
```

- CRC：初值 `0x00`，多项式 `0x31`，仅覆盖线上 `Data'[12]`
- 解扰：`Data'[i] XOR {BD, DB, BD, DB, ...}`
- 通道：`CH1～CH4` 使用 11 位精度，`CH5～CH10` 使用 6 位精度
- 附加字段：11 位帧序号和 5 位保留字段

### Arduino → PX4

- 输出标准 25 字节 SBUS 帧
- `CH1～CH10` 保持接收器通道原顺序
- 除诊断用 `CH14` 外，`CH11～CH16` 默认置于补偿后的安全中位
- `CH14` 用作 11 位循环帧计数，仅供链路诊断
- 超过 200 ms 未收到有效 315 数据帧时，置位 `lost_frame` 和 `failsafe`

更完整的帧结构、位域、映射算法和状态机说明请参阅：

📘 [Hmini_315 代码架构与 315 接收说明](docs/Hmini_315_代码架构与315接收说明.md)

## 🎛️ 通道映射

| PX4 / SBUS | Arduino | 手柄操作 | 数值范围 |
|---|---|---|---:|
| CH1 — Roll | `ch[0]` | 右摇杆左右 | 1000～2000 |
| CH2 — Pitch | `ch[1]` | 右摇杆上下 | 1000～2000 |
| CH3 — Throttle | `ch[2]` | 左摇杆上下 | 1000～2000 |
| CH4 — Yaw | `ch[3]` | 左摇杆左右 | 1000～2000 |
| CH5 | `ch[4]` | 左扳机 SA | 1000 / 2000 |
| CH6 | `ch[5]` | 左三档开关 SB | 1000 / 1492 / 2000 |
| CH7 | `ch[6]` | 右三档开关 SC | 1000 / 1492 / 2000 |
| CH8 | `ch[7]` | 右扳机 SD | 1000 / 2000 |
| CH9 | `ch[8]` | 左扳机 SE | 1000 / 2000 |
| CH10 | `ch[9]` | 右旋钮 SI | 1000～2000 |

> `1492` 是 6 位辅助通道映射后的正常物理中位；发送 SBUS 时会单独补偿，使 PX4 端显示约为 `1500`。

## 🚀 快速开始

### 1. 准备环境

- Arduino Mega 2560
- Arduino IDE 与 Arduino AVR Boards 支持包
- Bolder Flight Systems SBUS 库（代码使用 `sbus.h`、`bfs::SbusTx`）
- 315 MHz 接收器与带 SBUS 输入的 PX4 飞控
- QGroundControl

`Wire`、`SPI`、`SD` 和 `Servo` 为 Arduino 内置库；MS5837 驱动已包含在仓库中。

### 2. 打开并上传草图

1. 使用 Arduino IDE 打开 `Hmini_315/Hmini_315.ino`。
2. 开发板选择 **Arduino Mega or Mega 2560**。
3. 确认主草图中保持：

   ```cpp
   #define RX315_BENCH_TEST_ONLY 1
   ```

4. 选择正确串口并编译、上传。
5. 打开 115200 baud 串口监视器。

正常连接后可以看到类似输出：

```text
RX315_FLAG,CONNECTED,frames=...,seq=...
RX315_VALID,frames=...,seq=...,ch=...
RX315_STATUS,bytes=...,valid=...,crc_errors=...
SBUS_TX_STATUS,frames=...,seq=...,failsafe=0
```

### 3. 在 PX4 / QGC 中验证

1. 将 PX4 通过 USB 连接到电脑。
2. 在 QGroundControl 中进入 `Vehicle Setup → Radio` 观察通道。
3. 也可以在 MAVLink Console 中运行：

   ```text
   listener input_rc -n 20
   ```

预期结果：

- 操作手柄时，CH1～CH10 随对应控件变化；
- 正常连接时 `rc_failsafe=false`；
- 关闭手柄超过 200 ms 后 `rc_failsafe=true`；
- 恢复连接后通道与状态自动恢复。

> [!NOTE]
> 请使用 QGC 的 **Radio** 页面验证飞控收到的 RC 数据；**Joystick** 页面用于电脑 USB/HID 手柄，不适用于本链路。

## 🛡️ 失控保护

当接收器噪声仍存在但没有通过 CRC 和帧尾校验的有效帧时，系统不会误判为在线。最后一帧有效数据超过 200 ms 后，Arduino 将持续发送安全 SBUS 帧：

- Roll、Pitch、Yaw：回中
- Throttle：最低
- CH5～CH10：最低
- `lost_frame=true`
- `failsafe=true`

重新收到有效帧后，系统会自动恢复实时通道输出。

## 🗂️ 仓库结构

```text
Hmini_315/
├─ Hmini_315.ino                  主程序入口与台架模式
├─ RX315.ino                      接收、校验、解扰和通道解析
├─ sbus.ino                       标准 SBUS 输出与 Failsafe
├─ WaterControl315.ino            保留的水下动力控制逻辑
├─ YawCtrl.ino                    姿态、深度 PID 与跟踪微分器
├─ MS5837.*                       Bar30 深度传感器驱动
├─ imu_data_decode.* / packet.*   IMU 串口协议解析
├─ tools/RX315_Diagnostics/       原始脉冲与 UART 诊断草图
├─ docs/                          详细代码与协议文档
├─ 315/                           STM32 发送端与 PX4 直连驱动参考
└─ README.md
```

Arduino IDE 会自动合并 `Hmini_315/` 目录中的所有 `.ino` 文件进行编译；`tools/` 和 `legacy/` 中的文件不会进入主草图构建。

## 🔍 诊断提示

- **完全没有输入字节**：检查接收器 `OUT → D15/RX3`、供电和共地。
- **持续出现 CRC 错误**：检查波特率、电平、干扰和接收器协议版本。
- **Arduino 通道正常但 PX4 无输入**：重点检查 SBUS 反相、电平适配和 PX4 的 RC/SBUS 端口配置。
- **需要抓取原始信号**：使用 `Hmini_315/tools/RX315_Diagnostics/RX315_Diagnostics.ino`。
- **PX4 中位或端点偏移**：根据实际硬件重新标定 `SBUS_PX4_MIN/MID/MAX`。

> [!WARNING]
> `CH14` 是诊断帧计数，不是 PWM 控制通道。请勿将其分配给飞行模式、AUX passthrough、参数调节或任何执行器。

## 🧭 下一阶段

- [ ] 完成 315 通道到水桨控制变量的映射
- [ ] 确认通道反向、中位死区和开关阈值
- [ ] 完成执行器上电与断联安全检查
- [ ] 恢复并联调 IMU、Bar30 和 SD 日志
- [ ] 开展整机水下控制测试

---

<div align="center">

**通信先可靠，动力再上电。** ⚡🛟

</div>
