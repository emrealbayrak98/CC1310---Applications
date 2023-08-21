/*
 * bmp180.h
 *
 *  Created on: 11 Aðu 2023
 *      Author: emre
 */

#ifndef BMP180_DRIVER_BMP180_H_
#define BMP180_DRIVER_BMP180_H_

#include <stdint.h>
#include <ti/drivers/I2C.h>
#include <math.h>

// Slave address
#define BMP180_7_BIT_ADDRESS 0b1110111
#define BMP180_ADDRESS      (BMP180_7_BIT_ADDRESS << 1)

// Registers
#define CALIB_ADDR     0xAA
#define ID_ADDR        0xD0
#define SOFT_ADDR      0xE0
#define CTRL_MEAS_ADDR 0xF4
#define OUT_MSB_ADDR   0xF6
#define OUT_LSB_ADDR   0xF7
#define OUT_XLSB_ADDR  0xF8

// BMP180 Commands and values
#define SOFT_RESET_VAL 0xB6
#define ID_VAL         0x55
#define RED_UT_VAL     0x2E
#define RED_UP_VAL     0x34

// Private defines
#define convert8bitto16bit(high, low) (((uint16_t)(high) << 8) | (low))
#define powerof2(x)                 (1 << (x))
#define MILLI_SEC      1000

/*
 * Oversampling settings
 */
enum bmp180_oversampling_settings{
    ultra_low_power,
    standard,
    high_resolution,
    ultra_high_resolution
};

/*
 * Holds sensor data, sensor settings and calibration values.
 */
typedef struct {
    /* I2C handle structures */
    I2C_Handle      *hi2cx;
    I2C_Transaction *i2cTransaction;

    float temperature;
    int32_t pressure;
    float altitude;
    int32_t  sea_pressure;

    enum bmp180_oversampling_settings oversampling_setting;
    uint8_t  oss;

    /* BMP180 Registers */
    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t  B1;
    int16_t  B2;
    int32_t  B3;
    uint32_t B4;
    int32_t  B5;
    int32_t  B6;
    uint32_t B7;
    int16_t  MB;
    int16_t  MC;
    int16_t  MD;


}bmp180_t;


// Private function prototypes
uint8_t bmp180_init(I2C_Handle* i2c,I2C_Transaction* i2cTransaction, bmp180_t *bmp180);
void bmp180_get_all(I2C_Handle* i2c,bmp180_t *bmp180);
void bmp180_get_temperature(I2C_Handle* i2c,bmp180_t *bmp180);
void bmp180_get_pressure(I2C_Handle* i2c,bmp180_t *bmp180);
void bmp180_get_altitude(bmp180_t *bmp180);
static void bmp180_read(I2C_Handle* i2c,bmp180_t *bmp180, uint8_t reg, uint8_t *rbuffer, uint8_t size);
static void bmp180_write(I2C_Handle* i2c,bmp180_t *bmp180, uint8_t reg, uint8_t *buffer, size_t size);
static int32_t _bmp180_read_ut(I2C_Handle* i2c,bmp180_t *bmp180);
static int32_t _bmp180_read_up(I2C_Handle* i2c,bmp180_t *bmp180);
static uint8_t get_calibration(I2C_Handle* i2c,bmp180_t *bmp180, uint8_t* rbuffer);
#endif /* BMP180_DRIVER_BMP180_H_ */
