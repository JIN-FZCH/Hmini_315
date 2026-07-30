# PX4 固件归档

本目录保存与项目联调时使用或留存的 PX4 飞控固件，不参与 Arduino Mega 草图的编译与烧录。

## `px4_fmu-v6c_default-v1.13.3.px4`

- 目标：PX4FMUv6C
- Board ID：56
- PX4 版本标识：v1.13.3
- Git 哈希：`1c8ab2a0d7db2d14a6f320ebd8766b5ffaea28fa`
- 用途：通过 QGroundControl 的自定义固件功能刷写兼容的 FMUv6C 飞控

刷写前必须确认实际飞控硬件确实兼容 FMUv6C，并备份参数。该文件与 `references/px4-custom-module/uart_telem3.cpp` 分开归档；后者只是 PX4 自定义模块的参考源码，不能仅凭文件名认定本固件已经包含该模块。
