# Joycon 项目 - AI 编码指南

## 项目概览
这是一个用于遥控车控制器 ("Joycon") 的 ESP-IDF 项目。它运行在 ESP32 上，具有摇杆、LCD 显示屏、WiFi 连接功能，并集成了阿里云 IoT。

## 架构与结构
- **入口点**: main/main.c 初始化系统、NVS、WiFi，并启动主应用程序任务。
- **板级支持包 (BSP)**: 硬件抽象层位于 main/bsp_*.c 和 main/include/bsp_*.h。
  - bsp_display.c: SPI LCD 驱动和 UI 渲染逻辑。
  - bsp_joystick.c: 基于 ADC 的摇杆输入处理及校准。
  - bsp_key_fsm.c: 按键输入的有限状态机。
  - bsp_dns.c / bsp_http.c: 网络服务。
- **组件**:
  - components/ali-iot: 用于阿里云 IoT MQTT 认证和通信的自定义组件。
- **配置**: 系统配置（WiFi 凭据、设备信息）存储在 NVS（非易失性存储）中。

## 关键约定
- **全局变量**: 全局状态变量以 g_ 为前缀（例如：g_wifi_connected, g_dev_config）。
- **任务管理**: 使用 FreeRTOS 任务和队列。示例：main/main.c 中的 beep_task 通过队列处理蜂鸣器输出。
- **内存管理**: bsp_config.h 中的自定义宏 user_malloc 和 user_iram_malloc 封装了 heap_caps_malloc。
- **引脚定义**:
  - LCD 引脚定义在 main/include/bsp_display.h 中。
  - 其他 GPIO（如 BEEP_GPIO）定义在各自的源文件或 main.c 中。

## 构建与工作流
- **构建系统**: 基于 CMake 的 ESP-IDF 构建系统。
- **配置**: sdkconfig 保存项目配置 (Kconfig)。
- **烧录与监控**: 使用标准的 ESP-IDF 命令 (idf.py flash monitor) 或 VS Code 扩展任务。
- **依赖管理**: 通过 main/CMakeLists.txt 中的 idf_component_register 进行管理。

## 开发指南
- **添加功能**:
  1.  在 main/include/bsp_*.h 中定义硬件接口。
  2.  在 main/bsp_*.c 中实现驱动逻辑。
  3.  更新 main/main.c 以初始化新组件并创建必要的任务。
- **UI 更新**: 修改 bsp_display.c 以更新帧缓冲区 (lcd_frame_buffer) 和菜单结构 (disp_menu_t)。
- **网络逻辑**: 在尝试网络操作之前，确保 WiFi 已连接 (g_wifi_connected)。

## 常用模式
- **NVS 访问**: 使用
vs_open,
vs_get_*,
vs_set_*,
vs_commit 序列进行配置持久化。
- **ADC 校准**: bsp_joystick.c 演示了 adc_cali_scheme_line_fitting 的用法。
- **JSON 解析**: 使用 cJSON 库解析 MQTT 和 HTTP 响应。
