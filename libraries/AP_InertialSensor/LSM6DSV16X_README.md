# LSM6DSV16X 驱动使用说明

## 概述
LSM6DSV16X 是 STMicroelectronics 的高性能 6 轴 IMU，支持高达 8kHz 的陀螺仪采样率。

## 特性
- 陀螺仪: ±2000 dps, 8kHz ODR
- 加速度计: ±16G, 1kHz ODR
- SPI 接口, 最高 10MHz
- 内置数字滤波器

## 硬件配置

在板级 hwdef.dat 文件中添加:

```
# SPI 设备定义
SPIDEV lsm6dsv16x SPI1 DEVID1 LSM6DSV16X_CS MODE3 10*MHZ 10*MHZ

# IMU 定义
IMU LSM6DSV16X SPI:lsm6dsv16x ROTATION_NONE
```

## 参数说明

- `SPI:lsm6dsv16x`: SPI 设备名称
- `ROTATION_NONE`: 传感器安装方向，可选值:
  - ROTATION_NONE
  - ROTATION_YAW_90
  - ROTATION_YAW_180
  - ROTATION_YAW_270
  - ROTATION_ROLL_180
  - 等等

## 驱动特点

1. **高采样率**: 陀螺仪 8kHz, 加速度计 1kHz
2. **低延迟**: 脉冲中断模式
3. **数字滤波**: 内置 LPF1 滤波器 (288Hz)
4. **高精度**: 高精度 ODR 模式

## 寄存器配置

- 陀螺仪: 2000 dps, 8kHz, 高精度模式
- 加速度计: 16G, 1kHz, 高精度模式
- 滤波器: LPF1 使能, 带宽 288Hz
- 中断: 脉冲模式, 陀螺仪数据就绪

## 调试

在 `AP_InertialSensor_LSM6DSV16X.h` 中设置:
```cpp
#define LSM6DSV16X_DEBUG 1
```

重新编译后会在启动时打印寄存器转储。

## 移植说明

本驱动从 Betaflight 移植，主要改动:
1. 适配 ArduPilot 的 HAL 接口
2. 简化为基本功能 (无 FIFO, 无 EIS)
3. 使用 ArduPilot 的数据处理流程
4. 添加寄存器检查和错误处理
