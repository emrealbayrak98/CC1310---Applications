
/*
 *  ======== LED_BUTTON.c ========
 *  Register Level Implementation
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "cc1310.h"

#define IE_BIT 29
#define DIO6   6
#define DIO7   6
#define DIO13  13
#define DIO14  14
#define DOUT_16 16
#define DOUT_24 24

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    GPIO->DOUT23_20 = 1293912;
    uint32_t   time = 250000;                      /* < Some Delay > */
    IOC->IOCFG6   &= ~(1 << IE_BIT);               /* < Input Disabled > */
    IOC->IOCFG7   &= ~(1 << IE_BIT);               /* < Input Disabled > */
    GPIO->DOE31_0 |= (1 << DIO6);                  /* < Output Enabled > */
    GPIO->DOE31_0 |= (1 << DIO7);                  /* < Output Enabled > */
    IOC->IOCFG14  |= (1 << IE_BIT);                /* < Input Enabled For BTN2 > */
    IOC->IOCFG13  |= (1 << IE_BIT);                /* < Input Enabled For BTN1 > */
    while(1){
        if(!((GPIO->DIN31_0 >> DIO14 ) & 1)){      /* < Test Button Press > */
            usleep(time);                          /* < Avoid Debouncing > */
            GPIO->DOUT7_4 ^= (1 << DOUT_16);       /* < Toggle Output > */
        }
        if(!((GPIO->DIN31_0 >> DIO13 ) & 1)){      /* < Test Button Press > */
            usleep(time);                          /* < Avoid Debouncing > */
            GPIO->DOUT7_4 ^= (1 << DOUT_24);       /* < Toggle Output > */
        }
    }
}
