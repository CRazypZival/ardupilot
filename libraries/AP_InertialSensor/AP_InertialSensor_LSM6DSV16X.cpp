/*
 *  LSM6DSV16X 驱动 - 移植自 Betaflight
 */
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DSV16X.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;

// WHO_AM_I 值
#define LSM6DSV16X_WHO_AM_I_CONST       0x70
#define LSM6DSV16X_DRY_PIN              -1

// 10 MHz max SPI 频率
#define LSM6DSV16X_MAX_SPI_CLK_HZ       10000000

// 寄存器定义
#define LSM6DSV_WHO_AM_I                0x0F
#define LSM6DSV_CTRL1                   0x10
#define LSM6DSV_CTRL2                   0x11
#define LSM6DSV_CTRL3                   0x12
#define LSM6DSV_CTRL3_SW_RESET          0x01
#define LSM6DSV_CTRL3_IF_INC            0x04
#define LSM6DSV_CTRL3_BDU               0x40

#define LSM6DSV_CTRL4                   0x13
#define LSM6DSV_CTRL4_DRDY_PULSED       0x02

#define LSM6DSV_CTRL6                   0x15
#define LSM6DSV_CTRL7                   0x16
#define LSM6DSV_CTRL7_LPF1_G_EN         0x01
#define LSM6DSV_CTRL8                   0x17

#define LSM6DSV_HAODR_CFG               0x62

#define LSM6DSV_INT1_CTRL               0x0D
#define LSM6DSV_INT1_CTRL_INT1_DRDY_G   0x02

#define LSM6DSV_STATUS_REG              0x1E
#define LSM6DSV_STATUS_REG_GDA          0x02
#define LSM6DSV_STATUS_REG_XLDA         0x01

// 陀螺仪数据寄存器
#define LSM6DSV_OUTX_L_G                0x22
#define LSM6DSV_OUTX_H_G                0x23
#define LSM6DSV_OUTY_L_G                0x24
#define LSM6DSV_OUTY_H_G                0x25
#define LSM6DSV_OUTZ_L_G                0x26
#define LSM6DSV_OUTZ_H_G                0x27

// 加速度计数据寄存器
#define LSM6DSV_OUTX_L_A                0x28
#define LSM6DSV_OUTX_H_A                0x29
#define LSM6DSV_OUTY_L_A                0x2A
#define LSM6DSV_OUTY_H_A                0x2B
#define LSM6DSV_OUTZ_L_A                0x2C
#define LSM6DSV_OUTZ_H_A                0x2D

// ODR 和量程配置
#define LSM6DSV_CTRL1_ODR_XL_1000HZ     0x09
#define LSM6DSV_CTRL2_ODR_G_8000HZ      0x0C
#define LSM6DSV_CTRL6_FS_G_2000DPS      0x04
#define LSM6DSV_CTRL8_FS_XL_16G         0x03
#define LSM6DSV_CTRL6_LPF1_G_BW_288HZ   0x00

// 构造函数
AP_InertialSensor_LSM6DSV16X::AP_InertialSensor_LSM6DSV16X(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                           int drdy_pin_num,
                                                           enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _drdy_pin_num(drdy_pin_num)
    , _rotation(rotation)
{
}

// 探测传感器
AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSV16X::probe(AP_InertialSensor &_imu,
                                                               AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSV16X *sensor =
        NEW_NOTHROW AP_InertialSensor_LSM6DSV16X(_imu, std::move(dev),
                                                 LSM6DSV16X_DRY_PIN,
                                                 rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// 初始化传感器
bool AP_InertialSensor_LSM6DSV16X::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    if (_drdy_pin_num >= 0) {
        _drdy_pin = hal.gpio->channel(_drdy_pin_num);
        if (_drdy_pin == nullptr) {
            AP_HAL::panic("LSM6DSV16X: null data-ready GPIO channel\n");
        }
        _drdy_pin->mode(HAL_GPIO_INPUT);
    }

    bool success = _hardware_init();

#if LSM6DSV16X_DEBUG
    _dump_registers();
#endif
    return success;
}

// 硬件初始化
bool AP_InertialSensor_LSM6DSV16X::_hardware_init()
{
    _spi_sem->take_blocking();

    // 设置读标志
    _dev->set_read_flag(0x80);

    // 检查 WHO_AM_I
    uint8_t whoami = _register_read(LSM6DSV_WHO_AM_I);
    if (whoami != LSM6DSV16X_WHO_AM_I_CONST) {
        DEV_PRINTF("LSM6DSV16X: unexpected WHOAMI 0x%x\n", whoami);
        _spi_sem->give();
        return false;
    }

    // 软复位
    _register_write(LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);
    hal.scheduler->delay(10);

    // 等待复位完成
    uint8_t tries = 0;
    while (_register_read(LSM6DSV_CTRL3) & LSM6DSV_CTRL3_SW_RESET) {
        hal.scheduler->delay(1);
        if (++tries > 100) {
            DEV_PRINTF("LSM6DSV16X: reset timeout\n");
            _spi_sem->give();
            return false;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // 配置寄存器自增和块数据更新
    _register_write(LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU);
    hal.scheduler->delay(1);

    // 配置高精度 ODR 模式
    _register_write(LSM6DSV_HAODR_CFG, 0x01);
    hal.scheduler->delay(1);

    // 初始化陀螺仪和加速度计
    _gyro_init();
    _accel_init();

    // 配置中断为脉冲模式
    _register_write(LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED);
    hal.scheduler->delay(1);

    // 使能陀螺仪数据就绪中断
    _register_write(LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G);
    hal.scheduler->delay(1);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _spi_sem->give();
    return true;
}

// 陀螺仪初始化
void AP_InertialSensor_LSM6DSV16X::_gyro_init()
{
    // 配置陀螺仪: 8kHz ODR, 高精度模式
    _register_write(LSM6DSV_CTRL2, LSM6DSV_CTRL2_ODR_G_8000HZ | (0x01 << 4));
    hal.scheduler->delay(1);

    // 配置陀螺仪: 2000 dps 量程, LPF1 带宽 288Hz
    _register_write(LSM6DSV_CTRL6, (LSM6DSV_CTRL6_LPF1_G_BW_288HZ << 4) | LSM6DSV_CTRL6_FS_G_2000DPS);
    hal.scheduler->delay(1);

    // 使能 LPF1 滤波器
    _register_write(LSM6DSV_CTRL7, LSM6DSV_CTRL7_LPF1_G_EN);
    hal.scheduler->delay(1);

    // 设置陀螺仪刻度: 70 mdps/LSB for ±2000 dps
    _gyro_scale = 0.070f * DEG_TO_RAD;
}

// 加速度计初始化
void AP_InertialSensor_LSM6DSV16X::_accel_init()
{
    // 配置加速度计: 1kHz ODR, 高精度模式
    _register_write(LSM6DSV_CTRL1, LSM6DSV_CTRL1_ODR_XL_1000HZ | (0x01 << 4));
    hal.scheduler->delay(1);

    // 配置加速度计: 16G 量程
    _register_write(LSM6DSV_CTRL8, LSM6DSV_CTRL8_FS_XL_16G);
    hal.scheduler->delay(1);

    // 设置加速度计刻度: 16G 量程, 0.488 mg/LSB
    _accel_scale = 0.488f * GRAVITY_MSS / 1000.0f;
}

// 启动传感器
void AP_InertialSensor_LSM6DSV16X::start(void)
{
    // 注册陀螺仪和加速度计
    if (!_imu.register_gyro(gyro_instance, 8000, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV16X)) ||
        !_imu.register_accel(accel_instance, 1000, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV16X))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);

    _set_accel_max_abs_offset(accel_instance, 5.0f);

    // 启动定时器读取数据
    _dev->register_periodic_callback(125, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSV16X::_poll_data, void));
}

// 读取寄存器
uint8_t AP_InertialSensor_LSM6DSV16X::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

// 写入寄存器
void AP_InertialSensor_LSM6DSV16X::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

// 轮询数据
void AP_InertialSensor_LSM6DSV16X::_poll_data()
{
    // 读取陀螺仪和加速度计数据
    _read_gyro();
    _read_accel();

    // 检查寄存器值
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

// 读取陀螺仪数据
void AP_InertialSensor_LSM6DSV16X::_read_gyro()
{
    struct PACKED {
        int16_t x;
        int16_t y;
        int16_t z;
    } raw_data;

    const uint8_t reg = LSM6DSV_OUTX_L_G | 0x80;

    if (!_dev->transfer(&reg, 1, (uint8_t *)&raw_data, sizeof(raw_data))) {
        DEV_PRINTF("LSM6DSV16X: error reading gyroscope\n");
        return;
    }

    Vector3f gyro_data(raw_data.x, raw_data.y, raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_data);
}

// 读取加速度计数据
void AP_InertialSensor_LSM6DSV16X::_read_accel()
{
    struct PACKED {
        int16_t x;
        int16_t y;
        int16_t z;
    } raw_data;

    const uint8_t reg = LSM6DSV_OUTX_L_A | 0x80;

    if (!_dev->transfer(&reg, 1, (uint8_t *)&raw_data, sizeof(raw_data))) {
        DEV_PRINTF("LSM6DSV16X: error reading accelerometer\n");
        return;
    }

    Vector3f accel_data(raw_data.x, raw_data.y, raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(accel_instance, accel_data);
    _notify_new_accel_raw_sample(accel_instance, accel_data);
}

// 更新
bool AP_InertialSensor_LSM6DSV16X::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    return true;
}

#if LSM6DSV16X_DEBUG
// 调试: 转储寄存器
void AP_InertialSensor_LSM6DSV16X::_dump_registers(void)
{
    hal.console->println("LSM6DSV16X registers:");

    const uint8_t first = 0x0F;
    const uint8_t last = 0x2D;
    for (uint8_t reg = first; reg <= last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif
