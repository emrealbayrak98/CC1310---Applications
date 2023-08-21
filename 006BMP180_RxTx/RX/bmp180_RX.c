/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 *Author: Nihat Emre Albayrak
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
//#include <stddef.h>
//#include <unistd.h>
//#include <string.h>

/* TI Drivers */
//#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/NVS.h>

/* Board Header files */
#include "Board.h"

/* Driverlib Header files */
//#include <ti/devices/DeviceFamily.h>
//#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
//#include DeviceFamily_constructPath(driverlib/cpu.h)
//#include DeviceFamily_constructPath(driverlib/interrupt.h)

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include "application_settings.h"

/* Uncomment the line below if MAX Capacity is needed. */
#define MAX_CAPACITY                           0x1        // Use max CAPACITY ie. Write raw data to RAM


/***** Defines *****/
#define MAX_SIZE_OF_RAW_BUFFER 9
#define MAX_SIZE_OF_RAW_TEXT_BUFFER 21
#define SIZE_OF_UART_BUFFER    46
#define RAM_WRITE_SPEED_DIV    3       /* Write speed divider(write speed = receive speed / 3) */
#define INT_RAM_REGION_SIZE    0x4000U /* Max size = 0x4000 */

#ifdef MAX_CAPACITY
    #define INT_RAM_WRITE_NUM INT_RAM_REGION_SIZE/MAX_SIZE_OF_RAW_BUFFER
#else
    #define INT_RAM_WRITE_NUM INT_RAM_REGION_SIZE/MAX_SIZE_OF_RAW_TEXT_BUFFER
#endif

#define UART_BAUD_RATE         9600

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8    /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             13   /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2    /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     4    /* The Data Entries data field will contain:
                                     * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                     * Max 30 payload bytes
                                     * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/* Packet TX Configuration */
#define PAYLOAD_LENGTH         13   /* Packet Length */
#define PACKET_INTERVAL        500  /* Set packet interval to 500us */

/* Buffers placed in RAM to hold bytes received. */
static char textBufUart[SIZE_OF_UART_BUFFER];
static char textBufRam[MAX_SIZE_OF_RAW_TEXT_BUFFER];

#if defined(MAX_CAPACITY)
static uint8_t rawBufRam[MAX_SIZE_OF_RAW_BUFFER];
#endif
/***** Prototypes *****/
void convertValuesToString();
static void callbackRx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void *setGPIO();
void gpioButtonFxn0(uint_least8_t index);
static float bytesToFloat(const uint8_t* bytes);
static void convertAndMerge(float temperature, int pressure, float altitude, uint8_t *buffer);

/***** Global Variables *****/
static uint32_t pressure;
static float temperature;
static float altitude;
static uint8_t packet_sent[PAYLOAD_LENGTH];
static uint8_t packet_received[MAX_LENGTH + NUM_APPENDED_BYTES - 1];  /* The length byte is stored in a separate variable */

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

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

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

/***** Function definitions *****/

/*
 *  ======== txTaskFunction ========
 */
void *mainThread(void *arg0)
{
    /* Initialize RF object */
    RF_Params rfParams;

    /* Initialize UART objects */
    UART_Handle uart;
    UART_Params uartParams;

    /* Initialize NVS objects */
    NVS_Handle nvsHandle;
    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;

    /* Call driver init functions */
    UART_init();
    NVS_init();
    setGPIO();

    /* Call driver Param init functions */
    UART_Params_init(&uartParams);
    NVS_Params_init(&nvsParams);
    RF_Params_init(&rfParams);

    /* Create a UART with data processing off. */
    uartParams.stopBits = UART_STOP_TWO;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.dataLength = UART_LEN_8;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.baudRate = UART_BAUD_RATE;

    /* Open a UART driver instance */
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Create a NVS for Internal Flash. */
    nvsHandle = NVS_open(Board_NVSINTERNAL, &nvsParams);

    if (nvsHandle == NULL) {
        while (1);
    }

    /* Get Attributes */
    NVS_getAttrs(nvsHandle, &regionAttrs);

    /* Print information about Flash MEM region */
    char info[69];
    memset(info,0,sizeof(info));
    snprintf(info, sizeof(info), "Region Base Address: 0x%x Sector Size: 0x%x Region Size: 0x%x\r\n",
             regionAttrs.regionBase, regionAttrs.sectorSize, regionAttrs.regionSize);

    UART_write(uart, &info, sizeof(info));

    /* Clear previously written data to Flash MEM */
    NVS_erase(nvsHandle, 0, regionAttrs.regionSize);

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

    volatile uint8_t ramCounter=0;

    while(1)
    {
        /* Enter Receive mode until packet Receive */
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                           RF_PriorityNormal, &callbackRx,
                                                           RF_EventRxEntryDone);

        /* Convert Received packet into strings of data */
        convertValuesToString();

        #if defined(MAX_CAPACITY)
        convertAndMerge(temperature, pressure, altitude, rawBufRam);
        #endif

        /* Display Received values using UART */
        UART_write(uart, &textBufUart, sizeof(textBufUart));

        /* Set a boundary according to Region Size*/
        if(ramCounter < INT_RAM_WRITE_NUM * RAM_WRITE_SPEED_DIV){

            /* To decrease writing frequency, a divider is used*/
            if((ramCounter % RAM_WRITE_SPEED_DIV == 0)){

                #if defined(MAX_CAPACITY)
                /* Write to memory */
                NVS_write(nvsHandle, MAX_SIZE_OF_RAW_BUFFER*(ramCounter/RAM_WRITE_SPEED_DIV), (void *) rawBufRam,
                          sizeof(rawBufRam),/*NVS_WRITE_ERASE |*/ NVS_WRITE_POST_VERIFY);
                #else
                /* Write to memory */
                NVS_write(nvsHandle, MAX_SIZE_OF_RAW_TEXT_BUFFER*(ramCounter/RAM_WRITE_SPEED_DIV), (void *) textBufRam,
                                          sizeof(textBufRam),/*NVS_WRITE_ERASE |*/ NVS_WRITE_POST_VERIFY);
                #endif
            }
            ramCounter++;
        }

        /* Send an acknowledge packet, packet_sent can be configured */
        RF_EventMask terminationReason2 = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                                         RF_PriorityNormal, NULL, 0);
    }

}
/*
 *  ======== callback RX ========
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

        /* Convert Received values into float*/
        temperature = bytesToFloat(packetDataPointer);
        pressure = bytesToFloat(packetDataPointer+4);
        altitude = bytesToFloat(packetDataPointer+8);

        /* Move to next entry */
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

/* Convert 4 bytes of integer to 32 bit float */
float bytesToFloat(const uint8_t* bytes)
{
    uint32_t intValue = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
                        ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];

    return *((float*)&intValue);
}

/* Writes the String with the values */
void convertValuesToString()
{
    memset(textBufUart,0,sizeof(textBufUart));
    snprintf(textBufUart, sizeof(textBufUart), "temp: %6.2f\r\npres: %6d\r\nalt: %8.2f\r\n", temperature, pressure, altitude);

    memset(textBufRam,0,sizeof(textBufRam));
    snprintf(textBufRam, sizeof(textBufRam), "%6.2f%6d%8.2f", temperature, pressure, altitude);
}

/* Convert temperature, pressure, and altitude into the specified byte format */
static void convertAndMerge(float temperature, int pressure, float altitude, uint8_t *buffer)
{
    int16_t temp_int = (int16_t)(temperature); // Convert to 1 decimal point
    uint8_t temp_frac = (uint8_t)((temperature - temp_int) * 100);

    uint8_t pressure_bytes[3];
    pressure_bytes[0] = (pressure >> 16) & 0xFF;
    pressure_bytes[1] = (pressure >> 8) & 0xFF;
    pressure_bytes[2] = pressure & 0xFF;

    int16_t alt_int = (int16_t)(altitude); // Convert to 1 decimal point
    uint8_t alt_frac = (uint8_t)((altitude - alt_int) * 100);

    // Merge the converted values into a 9-byte array
    buffer[0] = (uint8_t)(temp_int >> 8);
    buffer[1] = (uint8_t)(temp_int & 0xFF);
    buffer[2] = temp_frac;

    memcpy(buffer + 3, pressure_bytes, 3);

    buffer[6] = (uint8_t)(alt_int >> 8);
    buffer[7] = (uint8_t)(alt_int & 0xFF);
    buffer[8] = alt_frac;
}
