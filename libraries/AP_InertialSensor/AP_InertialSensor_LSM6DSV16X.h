#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

// 调试开关
#define LSM6DSV16X_DEBUG 0

class AP_InertialSensor_LSM6DSV16X : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM6DSV16X() { }
    void start(void) override;
    bool update() override;

    // 探测传感器
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

private:
    // 构造函数
    AP_InertialSensor_LSM6DSV16X(AP_InertialSensor &imu,
                                 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                 int drdy_pin_num,
                                 enum Rotation rotation);

    // 初始化
    bool _init_sensor();
    bool _hardware_init();

    // 陀螺仪和加速度计初始化
    void _gyro_init();
    void _accel_init();

    // 寄存器读写
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    // 数据读取
    void _poll_data();
    void _read_gyro();
    void _read_accel();

#if LSM6DSV16X_DEBUG
    void _dump_registers();
#endif

    // 成员变量
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;
    AP_HAL::DigitalSource *_drdy_pin;
    float _gyro_scale;
    float _accel_scale;
    int _drdy_pin_num;
    enum Rotation _rotation;
};
