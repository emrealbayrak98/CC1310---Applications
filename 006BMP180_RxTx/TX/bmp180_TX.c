/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Author : Nihat Emre Albayrak
 */

/*
 *  ======== i2ctmp116.c ========
 */
/***** Includes *****/
/* Standard C Libraries */
#include <unistd.h>
//#include <stdint.h>
//#include <stddef.h>
//#include <stdio.h>


/* External Drivers */
#include "BMP180_driver/bmp180.h"

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Board.h>

/* Example/Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include "application_settings.h"

/***** Defines *****/
#define TIMEOUT_MS             1000  /* Time limit for the Watchdog timer to reset. Must be bigger
                                     * than the Packet_Interval */
#define SEA_LEVEL_PRESSURE     101325/* Sea level atm. pressure */
/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8    /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             13   /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2    /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     4    /* The Data Entries data field will contain:
                                     * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                     * Max 30 payload bytes
                                     * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      13
#define PACKET_INTERVAL     250000  /* Set packet interval to 250ms */

/***** Prototypes *****/
static void callbackRx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void *setGPIO();
void gpioButtonFxn0(uint_least8_t index);
static void floatToBytes(float value, uint8_t bytes[4]);

/***** Global Variables *****/
static uint32_t pressure;
static float temp;
static float alt;
static uint8_t packet_sent[PAYLOAD_LENGTH];
static uint8_t packet_received[MAX_LENGTH + NUM_APPENDED_BYTES - 1];  /* The length byte is stored in a separate variable */

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

/* Compiler Supports */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/***** Function definitions *****/

/*
 *  ======== watchdogCallback ========
 */
void watchdogCallback(uintptr_t watchdogHandle)
{
    /*
     * If the Watchdog Non-Maskable Interrupt (NMI) is called,
     * loop until the device resets. Some devices will invoke
     * this callback upon watchdog expiration while others will
     * reset. See the device specific watchdog driver documentation
     * for your device.
     */
    while (1) {}
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    setGPIO();

    /* Initialize I2C objects */
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Initialize BMP180 object */
    bmp180_t bmp180;

    /* Initialize RF objects */
    RF_Params rfParams;

    /* Initialize Watchdog objects */
    Watchdog_Handle watchdogHandle;
    Watchdog_Params WDparams;
    uint32_t        reloadValue;

    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    Watchdog_init();
    RF_Params_init(&rfParams);
    I2C_Params_init(&i2cParams);
    Watchdog_Params_init(&WDparams);

    /* Set parameters of I2C */
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;

    /* Open a I2C driver instance */
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Set parameters of BMP180*/
    bmp180.sea_pressure = SEA_LEVEL_PRESSURE;
    bmp180.oversampling_setting = standard;

    /* Open a BMP180 driver instance */
    if(bmp180_init(&i2c, &i2cTransaction, (bmp180_t*)&bmp180)!=0){
        while(1); // Init Failed.
    }

    /* Set parameters of Watchdog */
    WDparams.callbackFxn = (Watchdog_Callback) watchdogCallback;
    WDparams.debugStallMode = Watchdog_DEBUG_STALL_ON;
    WDparams.resetMode = Watchdog_RESET_ON;

    /* Open a Watchdog driver instance */
    watchdogHandle = Watchdog_open(Board_WATCHDOG0, &WDparams);
    if (watchdogHandle == NULL) {
       /* Error opening Watchdog */
       while (1) {}
    }

    /*
    * The watchdog reload value is initialized during the
    * Watchdog_open() call.
    *
    * Converts TIMEOUT_MS to watchdog clock ticks.
    */
    reloadValue = Watchdog_convertMsToTicks(watchdogHandle, TIMEOUT_MS);

    /*
    * A value of zero (0) indicates the converted value exceeds 32 bits
    * OR that the API is not applicable for this specific device.
    */
    if (reloadValue != 0) {
       Watchdog_setReload(watchdogHandle, reloadValue);
    }


    if( RFQueue_defineQueue(&dataQueue,
                                   rxDataEntryBuffer,
                                   sizeof(rxDataEntryBuffer),
                                   NUM_DATA_ENTRIES,
                                   MAX_LENGTH + NUM_APPENDED_BYTES)){
       /* Failed to allocate space for all data entries */
       while(1);
    }


    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet_sent;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;


    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropRx.pktConf.bRepeatOk = false;        /* Stop after receiving a single valid packet */
    RF_cmdPropRx.pktConf.bRepeatNok = false;

    /* Request access to the radio */
    #if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
    #else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    #endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    while(1){
        /* Get all values from the sensor */
        bmp180_get_all(&i2c,&bmp180);
        temp = bmp180.temperature;
        pressure = bmp180.pressure;
        alt = bmp180.altitude;

        /* Convert the values */
        floatToBytes(temp,packet_sent+1);
        floatToBytes(pressure,packet_sent+5);
        floatToBytes(alt,packet_sent+9);

        Power_disablePolicy();

        /* Packet send interval */
        usleep(PACKET_INTERVAL);
        /* Send packet */
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityHigh, NULL , 0);

        /* Receive packet */
        RF_EventMask terminationReason2 = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                               RF_PriorityNormal, &callbackRx,
                                                               RF_EventRxEntryDone);
        /* Clear watchdog, BOARD RESET if it is not cleared */
        Watchdog_clear(watchdogHandle);
        Power_enablePolicy();

    }

}

/*
 *  ======== callback ========
 */
void callbackRx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {

        /* Toggle pin to indicate RX */
        GPIO_toggle(Board_GPIO_LED0);

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet_received, packetDataPointer, (packetLength + 1));

        /* Write Application here if needed */
        RFQueue_nextEntry();

    }
}


void gpioButtonFxn0(uint_least8_t index)
{
    /* Configure Here */
}

void *setGPIO()
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* install Button callback */
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(Board_GPIO_BUTTON0);

    return (NULL);
}

/* Convert provided float to 4 bytes of integer */
void floatToBytes(float value, uint8_t* bytes)
{
    uint32_t intValue = *((uint32_t*)&value);

    bytes[0] = (intValue >> 24) & 0xFF;
    bytes[1] = (intValue >> 16) & 0xFF;
    bytes[2] = (intValue >> 8) & 0xFF;
    bytes[3] = intValue & 0xFF;
}
