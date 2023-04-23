#include <thread>
#include <chrono>
#include "malloc.h"
#include "string.h"
#include "bmp280_app.hpp"
#include "i2c.h"

static struct bmp280_dev bmp;
static struct bmp280_config conf;
static struct bmp280_uncomp_data ucomp_data;
static i2c_t *i2c = NULL;

static void bmp280_delay_ms(uint32_t period_ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
}

static int8_t i2c_rw_init(const char* i2cdev)
{
    i2c = i2c_new();
    return (int8_t)i2c_open(i2c, i2cdev);
}

static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    if(reg_data == NULL) return -1;

    uint8_t* pdata = (uint8_t*)malloc(length + 1);
    pdata[0] = reg_addr;
    memcpy(&pdata[1], reg_data, length);

    struct i2c_msg msg =
    {
            .addr = (unsigned short)i2c_addr,
            .flags = 0,
            .len = (unsigned short)(length + 1),
            .buf = pdata
    };

    return (int8_t)i2c_transfer(i2c, &msg, 1);
}
static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    if(reg_data == NULL) return -1;

    struct i2c_msg msgs[2] = 
    {
        {
            .addr = (unsigned short)i2c_addr,
            .flags = 0,
            .len = 1,
            .buf = &reg_addr
        },
        {
            .addr = (unsigned short)i2c_addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = reg_data
        }
    };

    return i2c_transfer(i2c, msgs, 2);
}

int8_t BMP280_Init(std::string i2cdev)
{
    bmp.delay_ms = bmp280_delay_ms;
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;
    bmp.intf = BMP280_I2C_INTF;
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    int8_t rslt = (int8_t)i2c_rw_init(i2cdev.c_str());
    if (rslt != 0)
        return rslt;

    rslt = bmp280_init(&bmp);
    if (rslt != BMP280_OK)
        return rslt;

    rslt = bmp280_get_config(&conf, &bmp);
    if (rslt != BMP280_OK)
        return rslt;

    conf.filter = BMP280_FILTER_COEFF_2;
    conf.os_temp = BMP280_OS_4X;
    conf.os_pres = BMP280_OS_4X;
    conf.odr = BMP280_ODR_1000_MS;

    rslt = bmp280_set_config(&conf, &bmp);
    if (rslt != BMP280_OK)
        return rslt;

    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    return rslt;
}

bool BMP280_Measure()
{
    return bmp280_get_uncomp_data(&ucomp_data, &bmp) == BMP280_OK;
}
double BMP280_GetTemperature()
{
    double temp;
    bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
    return temp;
}
double BMP280_GetPressure()
{
    double pressure;
    bmp280_get_comp_pres_double(&pressure, ucomp_data.uncomp_press, &bmp);
    return pressure;
}