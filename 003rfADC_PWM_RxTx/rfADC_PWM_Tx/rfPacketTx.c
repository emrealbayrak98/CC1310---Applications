/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
//#include <math.h>
#include <stdint.h>
#include <stddef.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>

/* Driverlib Header files */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/
#define ADCBUFFERSIZE    (2)
#define ADC0              0
#define ADC1              1
#define MODE_BIT          3
/* Do power measurement */
//#define POWER_MEASUREMENT

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      7
#define PACKET_INTERVAL     500  /* Set packet interval to 500us */

/***** Prototypes *****/
void mainThreadADC(ADC_Handle* adcBuf,uint8_t mode);
int number_of_digits_16bit_integer(uint16_t num);
void separate_uint16_to_uint8(uint16_t num, uint8_t result_array[2]);
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);
void *setGPIO();

static uint8_t sampleBufferOne[ADCBUFFERSIZE];
static uint8_t sampleBufferTwo[ADCBUFFERSIZE];
static uint8_t ADC0len=0;
static uint8_t ADC1len=0;
static uint16_t adc0Value;
static uint16_t adc1Value;
static uint8_t clearFlag=0;
static uint8_t mode=0;


/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH];

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#ifdef POWER_MEASUREMENT
#if defined(Board_CC1350_LAUNCHXL)
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#endif
    PIN_TERMINATE
};

/***** Function definitions *****/


void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    setGPIO();

    ADC_Handle adcBuf;
    ADC_Handle adcBuf2;
    ADC_Params adcBufParams;
    ADC_Params adcBufParams2;

    /* Call driver init functions */
    ADC_init();

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADC_Params_init(&adcBufParams);
    adcBuf = ADC_open(Board_ADC0, &adcBufParams);

    if (adcBuf == NULL){
        /* ADCBuf failed to open. */
        while(1);
    }

    ADC_Params_init(&adcBufParams2);
    adcBuf2 = ADC_open(Board_ADC1, &adcBufParams2);

    if (adcBuf2 == NULL){
        /* ADCBuf failed to open. */
        while(1);
    }

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }


    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    while(1)
    {

        /* Send packet */
//        packet[0] = PAYLOAD_LENGTH;
        int counter = 0;
        mainThreadADC(&adcBuf,ADC0);
        mainThreadADC(&adcBuf2,ADC1);
        ADC0len = number_of_digits_16bit_integer(adc0Value);
        ADC1len = number_of_digits_16bit_integer(adc1Value);
        separate_uint16_to_uint8(adc0Value,sampleBufferOne);
        separate_uint16_to_uint8(adc1Value,sampleBufferTwo);

        packet[0] = ADC0len;
        packet[MODE_BIT] = mode;
        if(clearFlag == 1){
            packet[6] = 1;
            clearFlag = 0;
        }
        else{
            packet[6] = 0;
        }

        for(counter=0 ; counter < 2 ; counter++){
            packet[counter+1] = sampleBufferOne[counter];
            packet[counter+4] = sampleBufferTwo[counter];
        }
        /* Send packet */
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);

        /* Power down the radio */
        RF_yield(rfHandle);

        /* Sleep for PACKET_INTERVAL us */
        usleep(PACKET_INTERVAL);

    }
}

void mainThreadADC(ADC_Handle* adcBuf,uint8_t mode)
{
    /* Start converting. */
    if(mode == ADC0){
        ADC_convert(*adcBuf, &adc0Value);
    }
    else{
        ADC_convert(*adcBuf, &adc1Value);
    }

}
/* Number of digits counted */
int number_of_digits_16bit_integer(uint16_t num)
{
    int count = 0;
    int absolute_num = abs(num);
    do {
        count++;
        absolute_num /= 10;
    } while (absolute_num != 0);
    return count;
}

void separate_uint16_to_uint8(uint16_t num, uint8_t result_array[2])
{
    result_array[0] = (uint8_t)(num & 0xFF);         // Lower 8 bits
    result_array[1] = (uint8_t)((num >> 8) & 0xFF);  // Upper 8 bits
}

void gpioButtonFxn0(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle a FLAG */
    clearFlag = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    /* Clear the GPIO interrupt and Change the MOD */
    if(mode < 3){
        mode++;
    }
    else{
        mode = 0;
    }
}

void *setGPIO()
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(Board_GPIO_BUTTON1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* install Button callback */
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);
    GPIO_setCallback(Board_GPIO_BUTTON1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(Board_GPIO_BUTTON0);
    GPIO_enableInt(Board_GPIO_BUTTON1);

    return (NULL);
}
