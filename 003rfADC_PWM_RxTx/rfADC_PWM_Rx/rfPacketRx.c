/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/PWM.h>


/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8    /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             7    /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2    /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     4    /* The Data Entries data field will contain:
                                     * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                     * Max 30 payload bytes
                                     * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */
#define PWM_MODE_STD           0    /* Standard mode */
#define PWM_MODE_INC           1    /* Incrementing mode */
#define PWM_MODE_INC_ST        2    /* Steady Increment */
#define PWM_MODE_SPIN          3    /* Spinning mode */
#define PWM_MODE_INC_MAX       4    /* Maximum Increment */
#define pwmPeriod              20000


/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
uint16_t ASCIItoDec(uint8_t* pPacket,uint8_t pLen);
void merge_uint8_to_uint16(uint8_t* pPacket);
void mainThreadPWM(PWM_Handle* pwm1,PWM_Handle* pwm2,uint8_t mode);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;
PWM_Handle pwm1 = NULL;
PWM_Handle pwm2 = NULL;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
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


static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */
static uint16_t    duty[2] = {1500,2500};
static uint16_t    ControlBuf[2] = {0,0};
static uint8_t     resetFlag=0;
static uint8_t     XYmode=0;

/***** Function definitions *****/


void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);

    if (pwm1 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }

    PWM_start(pwm1);

    pwm2 = PWM_open(Board_PWM1, &params);
    if (pwm2 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }

    PWM_start(pwm2);



    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);

    while(1);

}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {

        /* Toggle pin to indicate RX */
//        PIN_setOutputValue(ledPinHandle, Board_PIN_LED0,
//                           !PIN_getOutputValue(Board_PIN_LED0));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));
        /* Check data[2] for XYmode */
        if(*(packetDataPointer+2) == 1){
            XYmode = 1;
        }
        else if(*(packetDataPointer+2) == 0){
            XYmode = 0;
        }
        else{
            XYmode = 2;
        }
        /* Merge an 8 bit integer to a 16 bit integer */
        merge_uint8_to_uint16(packetDataPointer);

        /* Check reset bit */
        if(*(packetDataPointer+5) == 1){
            resetFlag = 1;
        }

        /* PWM main function, Sets duty cycle */
        mainThreadPWM((PWM_Handle*)&pwm1,(PWM_Handle*)&pwm2,PWM_MODE_INC_MAX);

        /* Move to next entry */
        RFQueue_nextEntry();

    }
}
/* 8 bits integer to 16 bits integer */
void merge_uint8_to_uint16(uint8_t* pPacket) {
    uint8_t counter;
    uint8_t counter2;
    uint8_t dummyArr[4];
    uint16_t merged_value[2] = {0,0};

    for(counter=0 ; counter < 2 ; counter++){
        dummyArr[counter] = *(pPacket + counter);
        dummyArr[counter+2] = *(pPacket + counter + 3);
    }

    for(counter=0,counter2=0 ; counter < 2 ; counter++, counter2+=2){
        merged_value[counter] |= dummyArr[0+counter2];
        merged_value[counter] |= ((uint16_t)dummyArr[1+counter2] << 8);
    }
    for(counter=0 ; counter < 2 ; counter++){
        ControlBuf[counter] = merged_value[counter];
    }

}

//uint16_t ASCIItoDec(uint8_t* pPacket,uint8_t pLen){
//
//    uint16_t data = 0;
//    uint8_t byte[4] = {0,0,0,0};
//    uint8_t counter;
//    uint8_t offset = 4 - pLen;
//    uint16_t mulArray[4] = {1000,100,10,1};
//
//    for(counter=0 ; counter < pLen ; counter++){
//        byte[counter + offset] = (*(pPacket + counter) - 48);
//    }
//
//
//    for(counter=0 ; counter < 4 ; counter++ ){
//        data += byte[counter]*mulArray[counter];
//    }
//
//    return data;
//}

void mainThreadPWM(PWM_Handle* pwm1,PWM_Handle* pwm2,uint8_t mode)
{
    /* Period and duty in microseconds */
    /* Check reset bits */
    if(resetFlag == 1 && XYmode == 0){
        if(duty[0] > 1540){
            duty[0] -= 50;
        }
        else if(duty[0] < 1460){
            duty[0] += 50 ;
        }

        if(duty[0] == 1500){
            resetFlag = 0;
        }

    }
    else if(resetFlag == 1 && XYmode == 1){
            if(duty[1] > 0){
                duty[1] -= 100;
            }

            else if(duty[1] < 20000){
                duty[1] += 100 ;
            }

            if(duty[1]== 0){
                resetFlag = 0;
            }

        }
    else if(resetFlag == 1 && XYmode == 2){
        if(duty[0] > 1540){
            duty[0] -= 50;
        }

        else if(duty[0] < 1460){
            duty[0] += 50 ;
        }

        if(duty[1] > 50){
            duty[1] -= 100;
        }

        if(duty[1] == 0 && duty[0] == 1500){
            resetFlag = 0;
        }

    }
    /* X only mode */
    else if(XYmode == 0){
        if(mode == PWM_MODE_STD){
            if(ControlBuf[0] > 2250){
                duty[0] = 2500;
            }
            else if(ControlBuf[0] < 2250 & ControlBuf[0] > 750){
                duty[0] = 1500;
            }
            else{
                duty[0] = 500;
            }
        }
        else if(mode == PWM_MODE_INC){
            if(ControlBuf[0] > 2250 && duty[0] < 2470){
                duty[0] += 40;
            }
            else if(ControlBuf[0] < 750 && duty[0] > 530){
                duty[0] -= 40;
            }
            else if((ControlBuf[0] <= 2250 && ControlBuf[0] >= 750) ){
                if(ControlBuf[0] >= 1500 && duty[0] < 2490){
                    duty[0] += 20;
                }
                else if(ControlBuf[0] < 1500 && duty[0] > 1510){
                    duty[0] -= 20;
                }

            }
        }
        else if(mode == PWM_MODE_INC_ST){
            if(ControlBuf[0] > 2250 && duty[0] < 2490){
                duty[0] += 20;
            }
            else if(ControlBuf[0] < 750 && duty[0] > 510){
                duty[0] -= 20;
            }

        }
        else if(mode == PWM_MODE_SPIN){
            if(ControlBuf[0] > 2250){
                duty[0] = 2700;
            }
            else{
                duty[0] = 1500;
            }

        }
        else if(mode == PWM_MODE_INC_MAX){
            if(ControlBuf[0] > 2250 && duty[0] < 2460){
                duty[0] += 50;
            }
            else if(ControlBuf[0] < 750 && duty[0] > 540){
                duty[0] -= 50;
            }
        }
    }
    /* Y only mode */
    else if(XYmode == 1){
        if(mode == PWM_MODE_INC){
            if(ControlBuf[1] > 2250 && duty[1] < 19950){
                duty[1] += 100;
            }
            else if(ControlBuf[1] < 750 && duty[1] > 50){
                duty[1] -= 100;
            }
            else if((ControlBuf[1] <= 2250 && ControlBuf[1] >= 750) ){
                if(ControlBuf[1] >= 1600 && duty[1] < 20000){
                    duty[1] += 50;
                }
                else if(ControlBuf[1] < 1600 && duty[1] > 0){
                    duty[1] -= 50;
                }

            }
        }
        else if(mode == PWM_MODE_INC_ST){
            if(ControlBuf[1] > 2250 && duty[1] < 19950){
                duty[1] += 100;
            }
            else if(ControlBuf[1] < 750 && duty[1] > 50){
                duty[1] -= 100;
            }

        }
        else if(mode == PWM_MODE_INC_MAX){
            if(ControlBuf[1] > 2250 && duty[1] < 19850){
                duty[1] += 200;
            }
            else if(ControlBuf[1] < 750 && duty[1] > 150){
                duty[1] -= 200;
            }
        }
    }
    /* X and Y mode */
    else if(XYmode == 2){
        if(mode == PWM_MODE_INC_ST){
            if(ControlBuf[0] > 2250 && duty[0] < 2490){
                duty[0] += 20;
            }

            else if(ControlBuf[0] < 750 && duty[0] > 510){
                duty[0] -= 20;
            }

            if(ControlBuf[1] > 2250 && duty[1] < 19950){
                duty[1] += 100;
            }

            else if(ControlBuf[1] < 750 && duty[1] > 50){
                duty[1] -= 100;
            }

        }
        else if(mode == PWM_MODE_INC_MAX){
            if(ControlBuf[0] > 2250 && duty[0] < 2460){
                duty[0] += 50;
            }
            else if(ControlBuf[0] < 750 && duty[0] > 540){
                duty[0] -= 50;
            }

            if(ControlBuf[1] > 2250 && duty[1] < 19850){
                duty[1] += 200;
            }

            else if(ControlBuf[1] < 750 && duty[1] > 150){
                duty[1] -= 200;
            }

        }

    }
    /* pwm1 is set for X only mode */
    if(XYmode == 0){
        PWM_setDuty(*pwm1, duty[0]);
    }
    /* pwm2 is set for Y only mode */
    else if(XYmode == 1){
        PWM_setDuty(*pwm2, duty[1]);
    }
    /* both pwm1 and pwm2 are set for X and Y mode */
    else{
        PWM_setDuty(*pwm1, duty[0]);
        PWM_setDuty(*pwm2, duty[1]);
    }

}
