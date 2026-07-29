2026-07-29待办事项

1、[已实现] 增加AIR / SAFE / WATER模式隔离。AIR禁用本地水桨和舵机变化；SAFE/WATER禁止实时SBUS进入PX4。
2、[已实现] 水下解锁后舵机使用：
        servo_left.writeMicroseconds(angle);
        servo_right.writeMicroseconds(3000 - angle);
3、[已实现] 接入手柄通道：
        CH1 -> yaw_315
        CH2 -> pitch_315（实际为前后推进Surge）
        CH3 -> depth_d_315（1000→0.0 m，2000→1.0 m）
        CH7 -> AIR / SAFE / WATER
        CH10 -> angle

后续上机事项：
1、保持RX315_BENCH_TEST_ONLY=1验证模式日志、通道端点和入水联锁。
2、断桨验证推进方向、舵机方向和传感器超时保护。
3、确认安全后再将RX315_BENCH_TEST_ONLY改为0。
