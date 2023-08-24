/*
 *  ======== LED_BUTTON.c ========
 *  Driver Test Application
 */

/* Include GPIO Driver */
#include "cc1310_GPIO.h"

/* Include STD Libraries */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Shortcut Definitions */
#define MS      1000
#define BUTTON1 DIO13
#define BUTTON2 DIO14
#define LED1    DIO6
#define LED2    DIO7

/* Button ISR Flags */
uint8_t Button1_Flag = RESET;

/* ISR Function (Will be passed to GPIO callbackFxn) */
void Button1_ISR()
{
    Button1_Flag = SET;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Create GPIO Handle object */
    GPIO_Handle_t xGPIO;

    /* Initialize Structure Addresses */
    xGPIO.GPIO_RegDef = GPIO;
    xGPIO.IOC_RegDef = IOC;

    /* Reset parameters to default, Set new parameters */
    GPIO_Params_Reset(&xGPIO);
    xGPIO.Params.GPIO_MODE = Input_IE;
    xGPIO.Params.EDGE_DET = POSEDGE;
    xGPIO.Params.INT_PINS = BUTTON1;
    xGPIO.Params.PULL_CTL = PULLUP;
    xGPIO.Params.callbackFxn1 = &Button1_ISR;

    /* Initialize GPIO with new parameters */
    GPIO_Init(&xGPIO);

    /* Reset parameters to default, Set new parameters */
    GPIO_Params_Reset(&xGPIO);
    xGPIO.Params.GPIO_MODE = Input;
    xGPIO.Params.IN_PINS = BUTTON2;
    xGPIO.Params.PULL_CTL = PULLUP;

    /* Initialize GPIO with new parameters */
    GPIO_Init(&xGPIO);

    /* Reset parameters to default, Set new parameters */
    GPIO_Params_Reset(&xGPIO);
    xGPIO.Params.GPIO_MODE = Output;
    xGPIO.Params.OUT_PINS = LED1 | LED2;
    xGPIO.Params.PULL_CTL = NOPULL;

    /* Initialize GPIO with new parameters */
    GPIO_Init(&xGPIO);

    /* Led operations with buttons inf loop */
    while(1)
    {
        if(!GPIO_Input(&xGPIO, BUTTON2))
        {
            while(!GPIO_Input(&xGPIO, BUTTON2))
            {
                if(Button1_Flag == SET){
                    break;
                }
                GPIO_Toggle(&xGPIO, LED2);
                usleep(100*MS); // Blink period
            }

        }

        if(Button1_Flag == SET)
        {
            usleep(50*MS);
            GPIO_Toggle(&xGPIO, LED1);
            Button1_Flag = RESET;
        }

    }

}

