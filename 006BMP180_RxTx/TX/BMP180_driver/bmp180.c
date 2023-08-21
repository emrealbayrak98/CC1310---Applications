/*
 * bmp180.c
 *
 *  Created on: 11 Aðu 2023
 *      Author: emre
 */

#include "bmp180.h"
#include <unistd.h>

/*
 * Buffers used for getting calibration registers values
 */


/* FUNCTION DECLARATIONS ========*/

/**
 * @brief Reading values by first sending the register address, using I2C Lib.
 * @retval none
 * @param i2c* I2C = handle pointer.
 * @param bmp180* bmp180_t = pointer to the initialize structure.
 * @param uint8_t reg = Register address.
 * @param uint8_t *rbuffer = pointer to the read buffer.
 * @param uint8_t size = number of reads that will be executed.
 * */
static void bmp180_read(I2C_Handle* i2c,bmp180_t *bmp180, uint8_t reg, uint8_t *rbuffer, uint8_t size)
{
    uint8_t buffer = reg;
    bmp180->i2cTransaction->readBuf = rbuffer;
    bmp180->i2cTransaction->writeBuf = &buffer;
    bmp180->i2cTransaction->readCount = size;
    bmp180->i2cTransaction->writeCount = 1;
    bmp180->i2cTransaction->slaveAddress = BMP180_7_BIT_ADDRESS;
    if(!(I2C_transfer(*(i2c), bmp180->i2cTransaction))){
        while(1);
    }
}
/**
 * @brief Writing values by first sending the register address, using I2C Lib.
 * @retval none
 * @param i2c* I2C = handle pointer.
 * @param bmp180* bmp180_t = pointer to the initialize structure.
 * @param uint8_t reg = Register address.
 * @param uint8_t *buffer = pointer to the write buffer.
 * @param uint8_t size = number writes that will be executed.
 * @note Bit '0' must be left empty before initializing
 * */
static void bmp180_write(I2C_Handle* i2c,bmp180_t *bmp180, uint8_t reg, uint8_t *buffer, size_t size)
{
    buffer[0] = reg;
    bmp180->i2cTransaction->writeBuf = buffer;
    bmp180->i2cTransaction->writeCount = size;
    bmp180->i2cTransaction->readCount = 0;
    bmp180->i2cTransaction->slaveAddress = BMP180_7_BIT_ADDRESS;
    if(!(I2C_transfer(*(i2c), bmp180->i2cTransaction))){
            while(1);
        }
}

/**
 * @brief Initialize sensor and get calibration values.
 * @retval 0 on success, 1 on sensor error, 2 on wrong ID.
 * @param hi2cx I2C handle.
 * @param bmp180 `bmp180_t` struct to initialize.
 * */
uint8_t bmp180_init(I2C_Handle* i2c, I2C_Transaction* i2cTransaction,bmp180_t *bmp180)
{
    static uint8_t rbuffer[22];
    static uint8_t buffer[2];

    bmp180->i2cTransaction = i2cTransaction;
    /* Perform Soft Reset*/
    buffer[1] = SOFT_RESET_VAL;
    bmp180_write(i2c, bmp180, SOFT_ADDR , buffer, 2);
    usleep(10*MILLI_SEC);

    // Check if device ID is correct
    bmp180_read(i2c,bmp180, ID_ADDR , rbuffer, 1);
    if (rbuffer[0] != ID_VAL) {
        return 2;
    }

    // Set hardware oversampling setting
    switch (bmp180->oversampling_setting) {
        case ultra_low_power:
            bmp180->oss = 0;
            break;
        case standard:
            bmp180->oss = 1;
            break;
        case high_resolution:
            bmp180->oss = 2;
            break;
        case ultra_high_resolution:
            bmp180->oss = 3;
            break;
        default:
            bmp180->oversampling_setting = standard;
            bmp180->oss = 1;
            break;
    }

    /* Get Calib values */
    if(!get_calibration(i2c, bmp180, rbuffer)){
        return 1;
    }

    return 0;
}

/**
 * @brief Get all sensor data at once.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_all(I2C_Handle* i2c,bmp180_t *bmp180){
//    get_calibration(i2c, i2cTransaction, bmp180);
    bmp180_get_temperature(i2c,bmp180);
    bmp180_get_pressure(i2c,bmp180);
    bmp180_get_altitude(bmp180);
}

/**
 * @brief Read ut values from sensor.
 * @param hi2cx I2C handle.
 * @param bmp180 `bmp180_t` struct to initialize.
 * @retval uncompensated temperature value.
 * */
static int32_t _bmp180_read_ut(I2C_Handle* i2c,bmp180_t *bmp180)
{
    uint8_t write_data[2] = {0,RED_UT_VAL}, ut_data[2];

    bmp180_write(i2c,bmp180, CTRL_MEAS_ADDR , write_data, 2);
    usleep(5*MILLI_SEC);
    bmp180_read(i2c,bmp180, OUT_MSB_ADDR , ut_data, 2);

    return (convert8bitto16bit(ut_data[0], ut_data[1]));
}

/**
 * @brief Read up values from sensor.
 * @retval uncompensated pressure value.
 * @param hi2cx I2C handle.
 * @param bmp180 `bmp180_t` struct to initialize.
 * */
static int32_t _bmp180_read_up(I2C_Handle* i2c,bmp180_t *bmp180)
{
    uint8_t write_data[2] = {0,RED_UP_VAL + (bmp180->oss << 6)},up_data[3];
    bmp180_write(i2c,bmp180, CTRL_MEAS_ADDR , write_data, 2);
    uint8_t wait = 0;

    /* Sleep accordingly */
    switch (bmp180->oversampling_setting) {
        case ultra_low_power:
            wait = 5;
            break;
        case standard:
            wait = 8;
            break;
        case high_resolution:
            wait = 14;
            break;
        case ultra_high_resolution:
            wait = 26;
            break;
        default:
            wait = 5;
            break;
    }
    usleep(wait*MILLI_SEC);
    bmp180_read(i2c,bmp180, OUT_MSB_ADDR , up_data, 3);

    return ((up_data[0] << 16) + (up_data[1] << 8) + up_data[2]) >> (8 - bmp180->oss);
}

/**
 * @brief Get temperature data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_temperature(I2C_Handle* i2c,bmp180_t *bmp180)
{
    int32_t ut = _bmp180_read_ut(i2c,bmp180);
    int32_t X1, X2;

    X1 = (ut - bmp180->AC6) * bmp180->AC5 / powerof2(15);
    X2 = bmp180->MC * powerof2(11) / (X1 + bmp180->MD);
    bmp180->B5 = X1 + X2;
    bmp180->temperature = (float)(((bmp180->B5 + 8) / powerof2(4)) / 10.0);
}

/**
 * @brief Get pressure data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_pressure(I2C_Handle* i2c,bmp180_t *bmp180)
{
    int32_t X1, X2, X3, up = _bmp180_read_up(i2c,bmp180), p;
    bmp180->B6 = bmp180->B5 - 4000;
    X1 = (bmp180->B2 * (bmp180->B6 * bmp180->B6 / powerof2(12))) / powerof2(11);
    X2 = bmp180->AC2 * bmp180->B6 / powerof2(11);
    X3 = X1 + X2;
    bmp180->B3 = ((((uint32_t)(bmp180->AC1 * 4 + X3)) << bmp180->oss) + 2) / 4;
    X1 = bmp180->AC3 * bmp180->B6 / powerof2(13);
    X2 = (bmp180->B1 * (bmp180->B6 * bmp180->B6 / powerof2(12))) / powerof2(16);
    X3 = ((X1 + X2) + 2) / powerof2(2);
    bmp180->B4 = bmp180->AC4 * (uint32_t)(X3 + 32768) / powerof2(15);
    bmp180->B7 = ((uint32_t)up - bmp180->B3) * (50000 >> bmp180->oss);
    if (bmp180->B7 < 0x80000000) {
        p = (bmp180->B7 * 2) / bmp180->B4;
    }
    else {
        p = (bmp180->B7 / bmp180->B4) * 2;
    }
    X1 = (p / powerof2(8)) * (p / powerof2(8));
    X1 = (X1 * 3038) / powerof2(16);
    X2 = (-7357 * p) / powerof2(16);
    p = p + (X1 + X2 + 3791) / powerof2(4);
    bmp180->pressure = p;
}

/**
 * @brief Calculate altitude from pressure data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_altitude(bmp180_t *bmp180)
{
    bmp180->altitude = 44330 * (1 - pow(((float)bmp180->pressure / (float)bmp180->sea_pressure), (1 / 5.255)));
}

/**
 * @brief Get calibration values of the sensor.
 * @param hi2cx I2C handle.
 * @param bmp180 `bmp180_t` struct to initialize.
 * @retval 1 for success, 0 for sensor damage.
 * */
static uint8_t get_calibration(I2C_Handle* i2c, bmp180_t *bmp180, uint8_t* rbuffer){
    // Get calibration data
    /* Read from 0xAA to 0xBF*/
    bmp180_read(i2c,bmp180, CALIB_ADDR, rbuffer, 22);


    // If any of the calibration data is 0x00 or 0xFF, sensor is damaged
    uint8_t i = 0;
    for (i = 0; i < 22; i += 2) {
        uint16_t combined_calibration_data = convert8bitto16bit(rbuffer[i], rbuffer[i + 1]);
        if (combined_calibration_data == 0x00 || combined_calibration_data == 0XFF) {
            return 0;
        }
    }

    // Save calibration data
    bmp180->AC1 = convert8bitto16bit(rbuffer[0],  rbuffer[1]);
    bmp180->AC2 = convert8bitto16bit(rbuffer[2],  rbuffer[3]);
    bmp180->AC3 = convert8bitto16bit(rbuffer[4],  rbuffer[5]);
    bmp180->AC4 = convert8bitto16bit(rbuffer[6],  rbuffer[7]);
    bmp180->AC5 = convert8bitto16bit(rbuffer[8],  rbuffer[9]);
    bmp180->AC6 = convert8bitto16bit(rbuffer[10], rbuffer[11]);
    bmp180->B1  = convert8bitto16bit(rbuffer[12], rbuffer[13]);
    bmp180->B2  = convert8bitto16bit(rbuffer[14], rbuffer[15]);
    bmp180->B3  = 0;
    bmp180->B4  = 0;
    bmp180->B5  = 0;
    bmp180->B6  = 0;
    bmp180->B7  = 0;
    bmp180->MB  = convert8bitto16bit(rbuffer[16], rbuffer[17]);
    bmp180->MC  = convert8bitto16bit(rbuffer[18], rbuffer[19]);
    bmp180->MD  = convert8bitto16bit(rbuffer[20], rbuffer[21]);
    return 1;
}
