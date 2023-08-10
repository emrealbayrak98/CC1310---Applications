/***** Includes *****/
/* Standard C Libraries */
//#include <stdlib.h>
//#include <stdio.h>
//#include <math.h>
//#include <stdint.h>
//#include <stddef.h>
#include <unistd.h>

/* TI Drivers */
//#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

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

/***** Defines *****/
#define ADCBUFFERSIZE    (2)
#define ADC0              0
#define ADC1              1
#define MODE_BIT          3

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

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      7

#define PACKET_INTERVAL     500  /* Set packet interval to 500000us or 500ms */


/***** Prototypes *****/

uint32_t buffersCompletedCounter = 0;
static uint8_t ADC0len=0;
//static uint8_t ADC1len=0;
uint16_t adc0Value;
uint16_t adc1Value;
uint8_t clearFlag=0;
uint8_t mode=0;

/***** Prototypes *****/
static void callbackRx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void mainThreadPWM(PWM_Handle* pwm1,PWM_Handle* pwm2,uint8_t mode);
void merge_uint8_to_uint16(uint8_t* pPacket);
int number_of_digits_16bit_integer(short int num);
void separate_uint16_to_uint8(uint16_t num, uint8_t result_array[2]);
void *setGPIO();
void gpioButtonFxn1(uint_least8_t index);
void gpioButtonFxn0(uint_least8_t index);
void LoadPacketWithAdcValues(ADC_Handle* adcBuf,ADC_Handle* adcBuf2,
                             uint8_t sampleBufferOne[ADCBUFFERSIZE],uint8_t sampleBufferTwo[ADCBUFFERSIZE]);
void mainThreadADC(ADC_Handle* adcBuf,uint8_t mode);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

static uint8_t packet_sent[PAYLOAD_LENGTH];

static PWM_Handle pwm1 = NULL;
static PWM_Handle pwm2 = NULL;

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


static uint8_t     packet_received[MAX_LENGTH + NUM_APPENDED_BYTES - 1];  /* The length byte is stored in a separate variable */
static uint16_t    duty[2] = {1500,2500};                                 /* Duty Cycle Buffer */
static uint16_t    ControlBuf[2] = {0,0};
static uint8_t     resetFlag=0;                                           /* Sends Reset Flag */
static uint8_t     XYmode=1;                                              /* X only = 0 , Y only = 1 , X & Y both = 2 */


/***** Function definitions *****/

/*
 *  ======== txTaskFunction ========
 */
void *mainThread(void *arg0)
{
        uint8_t sampleBufferOne[ADCBUFFERSIZE];
        uint8_t sampleBufferTwo[ADCBUFFERSIZE];

        RF_Params rfParams;
        RF_Params_init(&rfParams);
        PWM_Params params;

        setGPIO();

        ADC_Handle adcBuf;
        ADC_Handle adcBuf2;
        ADC_Params adcBufParams;
        ADC_Params adcBufParams2;

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
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                           RF_PriorityNormal, &callbackRx,
                                                           RF_EventRxEntryDone);

        LoadPacketWithAdcValues(&adcBuf,&adcBuf2,sampleBufferOne,sampleBufferTwo);

        RF_EventMask terminationReason2 = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                                         RF_PriorityNormal, NULL, 0);
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
        memcpy(packet_received, packetDataPointer, (packetLength + 1));
//        if(*(packetDataPointer+2) == 1){
//            XYmode = 1;
//        }
//        else if(*(packetDataPointer+2) == 0){
//            XYmode = 0;
//        }
//        else{
//            XYmode = 2;
//        }
        merge_uint8_to_uint16(packetDataPointer);

        if(*(packetDataPointer+5) == 1){
            resetFlag = 1;
        }

        mainThreadPWM((PWM_Handle*)&pwm1,(PWM_Handle*)&pwm2,PWM_MODE_INC_MAX);


        RFQueue_nextEntry();

    }
}

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

int number_of_digits_16bit_integer(short int num) {
    if (num >= -32768 && num <= 32767) {
        int count = 0;
        int absolute_num = abs(num);
        do {
            count++;
            absolute_num /= 10;
        } while (absolute_num != 0);
        return count;
    } else {
        // Handle the case when the input is not a 16-bit signed integer.
        return -1;
    }
}

void mainThreadADC(ADC_Handle* adcBuf,uint8_t mode)
{
//    int_fast16_t res;
    /* Start converting. */
    if(mode == ADC0){
        ADC_convert(*adcBuf, &adc0Value);
    }
    else{
        ADC_convert(*adcBuf, &adc1Value);
    }

}

void LoadPacketWithAdcValues(ADC_Handle* adcBuf,ADC_Handle* adcBuf2,uint8_t sampleBufferOne[ADCBUFFERSIZE],uint8_t sampleBufferTwo[ADCBUFFERSIZE]){
//        packet[0] = PAYLOAD_LENGTH;
    int counter = 0;
    mainThreadADC(adcBuf,ADC0);
    mainThreadADC(adcBuf2,ADC1);
    ADC0len = number_of_digits_16bit_integer(adc0Value);
//    ADC1len = number_of_digits_16bit_integer(adc1Value);
    separate_uint16_to_uint8(adc0Value,sampleBufferOne);
    separate_uint16_to_uint8(adc1Value,sampleBufferTwo);

    packet_sent[0] = ADC0len;
    packet_sent[MODE_BIT] = mode;
    if(clearFlag == 1){
        packet_sent[6] = 1;
        clearFlag = 0;
    }
    else{
        packet_sent[6] = 0;
    }

    for(counter=0 ; counter < 2 ; counter++){
        packet_sent[counter+1] = sampleBufferOne[counter];
        packet_sent[counter+4] = sampleBufferTwo[counter];
    }
}
void separate_uint16_to_uint8(uint16_t num, uint8_t result_array[2]) {
    result_array[0] = (uint8_t)(num & 0xFF);         // Lower 8 bits
    result_array[1] = (uint8_t)((num >> 8) & 0xFF);  // Upper 8 bits
}

void gpioButtonFxn0(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
    clearFlag = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
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

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on Board_GPIO_BUTTON1.
     */


    return (NULL);
}
