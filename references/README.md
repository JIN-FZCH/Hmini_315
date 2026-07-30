# 平台参考代码

本目录中的代码不参与 `Hmini_315` Arduino 草图编译，仅用于保存早期链路方案和其他平台的参考实现。

```text
references/
├─ stm32-sbus-bridge/
│  └─ main.c             STM32 HAL 工程中的 SBUS 接收与协议转换参考
└─ px4-custom-module/
   └─ uart_telem3.cpp    PX4 自定义串口接收模块参考
```

这些文件不是完整的独立工程，缺少各自平台的工程配置、生成文件或 PX4 消息定义。需要重新使用时，应放回对应的 STM32CubeIDE 或 PX4-Autopilot 工程，并补齐构建配置后再编译。
