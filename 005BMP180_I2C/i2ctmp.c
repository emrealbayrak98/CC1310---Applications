/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Author : Nihat Emre Albayrak
 */

/*
 *  ======== i2ctmp116.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "BMP180_driver/bmp180.h"

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

uint32_t  pressure;
float  temp;
float  alt;


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Initialize I2C objects */
    I2C_Handle      i2c;
    I2C_Params      i2cParams;

    /* Initialize BMP180 object */
    bmp180_t bmp180;


    /* Call driver init functions */
    GPIO_init();
    I2C_init();

    /* Configure the LED */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_3400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Set parameters of BMP180*/
    bmp180.sea_pressure = 101325;
    bmp180.oversampling_setting =high_resolution;
    if(bmp180_init(&i2c, (bmp180_t*)&bmp180)!=0){
        while(1); // Init Failed.
    }
    while(1){
        bmp180_get_all(&i2c,&bmp180);
        temp = bmp180.temperature;
        pressure = bmp180.pressure;
        alt = bmp180.altitude;
    }

}
