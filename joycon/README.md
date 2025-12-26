# Joycon - ESP32 遥控车控制器

这是一个基于 ESP32 的智能遥控车控制器项目。它集成了摇杆输入、LCD 显示交互、WiFi 连接以及阿里云 IoT 远程控制功能。

## 🛠 技术栈 (Tech Stack)

### 硬件环境
- **主控芯片**: Espressif ESP32 (Xtensa® Dual-Core 32-bit LX6)
- **显示设备**: ST7567 128x64 单色 LCD (SPI 接口)
- **输入设备**:
  - 双轴模拟摇杆 (ADC 采集)
  - 物理按键 (GPIO 输入)
- **通信模块**: ESP32 内置 WiFi (2.4GHz)

### 软件环境
- **开发框架**: ESP-IDF (Espressif IoT Development Framework)
- **编程语言**: C (C99/C11 Standard)
- **构建系统**: CMake + Ninja
- **操作系统**: FreeRTOS (实时操作系统)

### 关键库与组件
- **FreeRTOS**: 任务调度、队列管理 (用于输入事件和显示刷新)。
- **ESP-IDF Drivers**: SPI Master (LCD), ADC (摇杆), GPIO, WiFi Station。
- **Network Protocols**:
  - **HTTP Client**: 用于设备绑定、解绑和查询设备列表。
  - **MQTT Client**: 集成阿里云 IoT SDK，用于发布控制指令。
- **cJSON**: 用于解析 HTTP 响应和构建 MQTT JSON 消息。
- **NVS (Non-Volatile Storage)**: 用于持久化存储 WiFi 配置和设备信息。

## 📂 项目结构

```
joycon/
├── CMakeLists.txt          # 项目级构建脚本
├── sdkconfig               # 项目配置文件 (Kconfig)
├── partitions.csv          # 分区表定义
├── main/                   # 核心应用源码
│   ├── main.c              # 程序入口，系统初始化，任务创建
│   ├── bsp_display.c       # LCD 驱动与 UI 绘制逻辑 (ST7567)
│   ├── bsp_joystick.c      # 摇杆 ADC 采集与校准
│   ├── bsp_key_fsm.c       # 按键状态机 (消抖、长按检测)
│   ├── bsp_http.c          # HTTP 网络请求封装
│   ├── bsp_dns.c           # DNS 解析服务
│   └── include/            # 头文件
├── components/             # 自定义组件
│   └── ali-iot/            # 阿里云 IoT 连接组件
└── README.md               # 项目说明文档
```

## 🚀 编译与烧录 (Build & Flash)

本项目使用标准的 ESP-IDF 编译链路。

### 1. 环境准备
确保已安装 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/index.html) 开发环境。

### 2. 配置项目
```bash
idf.py set-target esp32
idf.py menuconfig
```
*在 menuconfig 中配置串口、WiFi 凭据等（如果项目使用了 Kconfig 自定义配置）。*

### 3. 编译
```bash
idf.py build
```

### 4. 烧录与监控
连接 ESP32 开发板，执行以下命令进行烧录并打开串口监视器：
```bash
idf.py -p PORT flash monitor
```
*(将 `PORT` 替换为实际串口号，如 `COM3` 或 `/dev/ttyUSB0`)*

## ✨ 功能特性

1.  **仪表盘模式**: 实时显示双摇杆状态、电池电量、WiFi 信号强度。
2.  **菜单系统**:
    - 支持多级菜单导航。
    - 设备绑定/解绑流程。
    - 系统设置与重置。
3.  **智能配网**: 支持通过 NVS 存储和读取 WiFi 配置。
4.  **云端互联**:
    - 通过 HTTP 获取可绑定的车辆列表。
    - 通过 MQTT 实时发送摇杆控制数据 (JSON 格式)。
5.  **低功耗设计**: (待实现) 支持空闲自动息屏等功能。

## 📝 开发注意事项

- **LCD 驱动**: 修改 `bsp_display.c` 中的 `lcd_spi_init` 可适配不同的引脚定义。
- **摇杆校准**: 首次使用可能需要校准 ADC 范围，相关逻辑位于 `bsp_joystick.c`。
- **阿里云配置**: 请确保 `components/ali-iot` 中的三元组信息正确配置。
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
