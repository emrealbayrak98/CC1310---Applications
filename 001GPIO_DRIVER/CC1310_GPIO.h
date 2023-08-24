/*
 * CC1310_GPIO.h
 *
 *  Created on: 22 Aðu 2023
 *      Author: Nihat Emre Albayrak
 */

#ifndef CC1310_GPIO_H_
#define CC1310_GPIO_H_

/***** Includes *****/
/* Standard C Libraries */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* TI Drivers */
/* Interrupt Driver is used from TI Library, Can be implemented seperately */
#include "ti/devices/cc13x0/driverlib/interrupt.h"

/* Helping Macros */
#define __vo volatile
#define __weak __attribute__((weak))

/* Some Generic Macros */
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET            SET

/* Base addresses of IOC and GPIO Peripherals */
#define GPIO_BASEADDR       0x40022000U
#define IOC_BASEADDR        0x40081000U

/* Peripheral Definitions */
#define GPIO                ((GPIO_RegDef_t*)GPIO_BASEADDR)
#define IOC                 ((IOC_RegDef_t*)IOC_BASEADDR)

/* IOCFG Register Bit Numbers */
#define HYST_EN_ADDR           30
#define IE_ADDR                29
#define IOMODE_ADDR            24
#define EDGE_IRQ_EN_ADDR       18
#define EDGE_DET_ADDR          16
#define PULL_CTL_ADDR          13
#define SLEW_RED_ADDR          12
#define PORT_ID_ADDR           0

/* CC1310 Specific Parameters */
#define TOTAL_PIN_NUMBER       32           /*!< Total GPIO Pin Number >*/
#define INT_AON_GPIO_EDGE      16           /*!< IOC_GPIO Interrupt Number >*/
#define IOC_RESET_VALUE        0x6000U

/* BIT Masks */
#define IOC_REG_BITMASK        0x7F077F3FU  /*!< Reserved Bits Excluded >*/
#define ALL_BITMASK            0xFFFFFFFFU
#define BIT1                   0x1

/* BIT Numbers Mask Definitions */
#define DIO0           1 << 0               /*!< BIT Number 0 >*/
#define DIO1           1 << 1               /*!< BIT Number 1 >*/
#define DIO2           1 << 2               /*!< BIT Number 2 >*/
#define DIO3           1 << 3               /*!< BIT Number 3 >*/
#define DIO4           1 << 4               /*!< BIT Number 4 >*/
#define DIO5           1 << 5               /*!< BIT Number 5 >*/
#define DIO6           1 << 6               /*!< BIT Number 6 >*/
#define DIO7           1 << 7               /*!< BIT Number 7 >*/
#define DIO8           1 << 8               /*!< BIT Number 8 >*/
#define DIO9           1 << 9               /*!< BIT Number 9 >*/
#define DIO10          1 << 10              /*!< BIT Number 10 >*/
#define DIO11          1 << 11              /*!< BIT Number 11 >*/
#define DIO12          1 << 12              /*!< BIT Number 12 >*/
#define DIO13          1 << 13              /*!< BIT Number 13 >*/
#define DIO14          1 << 14              /*!< BIT Number 14 >*/
#define DIO15          1 << 15              /*!< BIT Number 15 >*/
#define DIO16          1 << 16              /*!< BIT Number 16 >*/
#define DIO17          1 << 17              /*!< BIT Number 17 >*/
#define DIO18          1 << 18              /*!< BIT Number 18 >*/
#define DIO19          1 << 19              /*!< BIT Number 19 >*/
#define DIO20          1 << 20              /*!< BIT Number 20 >*/
#define DIO21          1 << 21              /*!< BIT Number 21 >*/
#define DIO22          1 << 22              /*!< BIT Number 22 >*/
#define DIO23          1 << 23              /*!< BIT Number 23 >*/
#define DIO24          1 << 24              /*!< BIT Number 24 >*/
#define DIO25          1 << 25              /*!< BIT Number 25 >*/
#define DIO26          1 << 26              /*!< BIT Number 26 >*/
#define DIO27          1 << 27              /*!< BIT Number 27 >*/
#define DIO28          1 << 28              /*!< BIT Number 28 >*/
#define DIO29          1 << 29              /*!< BIT Number 29 >*/
#define DIO30          1 << 30              /*!< BIT Number 30 >*/
#define DIO31          1 << 31              /*!< BIT Number 31 >*/

/* Peripheral Register Definition Structures */

/* GPIO REGISTERS STRUCT */
typedef struct
{
    __vo uint32_t DOUT3_0;          /*!< Data Out 0 to 3 >*/
    __vo uint32_t DOUT7_4;          /*!< Data Out 4 to 7 >*/
    __vo uint32_t DOUT11_8;         /*!< Data Out 8 to 11 >*/
    __vo uint32_t DOUT15_12;        /*!< Data Out 12 to 15 >*/
    __vo uint32_t DOUT19_16;        /*!< Data Out 16 to 19 >*/
    __vo uint32_t DOUT23_20;        /*!< Data Out 20 to 23 >*/
    __vo uint32_t DOUT27_24;        /*!< Data Out 24 to 27 >*/
    __vo uint32_t DOUT31_28;        /*!< Data Out 28 to 31 >*/
    __vo uint32_t EMPTY[24];        /*!< Reserved >*/
    __vo uint32_t DOUT31_0;         /*!< Data Output for DIO 0 to 31 >*/
    __vo uint32_t EMPTY1[3];        /*!< Reserved >*/
    __vo uint32_t DOUTSET31_0;      /*!< Data Out Set >*/
    __vo uint32_t EMPTY2[3];        /*!< Reserved >*/
    __vo uint32_t DOUTCLR31_0;      /*!< Data Out Clear >*/
    __vo uint32_t EMPTY3[3];        /*!< Reserved >*/
    __vo uint32_t DOUTTGL31_0;      /*!< Data Out Toggle >*/
    __vo uint32_t EMPTY4[3];        /*!< Reserved >*/
    __vo uint32_t DIN31_0;          /*!< Data Input from DIO 0 to 31 >*/
    __vo uint32_t EMPTY5[3];        /*!< Reserved >*/
    __vo uint32_t DOE31_0;          /*!< Data Output Enable for DIO 0 to 31 >*/
    __vo uint32_t EMPTY6[3];        /*!< Reserved >*/
    __vo uint32_t EVFLAGS31_0;      /*!< Event Register for DIO 0 to 31 >*/

}GPIO_RegDef_t;

/* IOC REGISTERS STRUCT */
typedef struct
{
    __vo uint32_t IOCFG0;       /*!< Configuration of DIO0 >*/
    __vo uint32_t IOCFG1;       /*!< Configuration of DIO1 >*/
    __vo uint32_t IOCFG2;       /*!< Configuration of DIO2 >*/
    __vo uint32_t IOCFG3;       /*!< Configuration of DIO3 >*/
    __vo uint32_t IOCFG4;       /*!< Configuration of DIO4 >*/
    __vo uint32_t IOCFG5;       /*!< Configuration of DIO5 >*/
    __vo uint32_t IOCFG6;       /*!< Configuration of DIO6 >*/
    __vo uint32_t IOCFG7;       /*!< Configuration of DIO7 >*/
    __vo uint32_t IOCFG8;       /*!< Configuration of DIO8 >*/
    __vo uint32_t IOCFG9;       /*!< Configuration of DIO9 >*/
    __vo uint32_t IOCFG10;      /*!< Configuration of DIO10 >*/
    __vo uint32_t IOCFG11;      /*!< Configuration of DIO11 >*/
    __vo uint32_t IOCFG12;      /*!< Configuration of DIO12 >*/
    __vo uint32_t IOCFG13;      /*!< Configuration of DIO13 >*/
    __vo uint32_t IOCFG14;      /*!< Configuration of DIO14 >*/
    __vo uint32_t IOCFG15;      /*!< Configuration of DIO15 >*/
    __vo uint32_t IOCFG16;      /*!< Configuration of DIO16 >*/
    __vo uint32_t IOCFG17;      /*!< Configuration of DIO17 >*/
    __vo uint32_t IOCFG18;      /*!< Configuration of DIO18 >*/
    __vo uint32_t IOCFG19;      /*!< Configuration of DIO19 >*/
    __vo uint32_t IOCFG20;      /*!< Configuration of DIO20 >*/
    __vo uint32_t IOCFG21;      /*!< Configuration of DIO21 >*/
    __vo uint32_t IOCFG22;      /*!< Configuration of DIO22 >*/
    __vo uint32_t IOCFG23;      /*!< Configuration of DIO23 >*/
    __vo uint32_t IOCFG24;      /*!< Configuration of DIO24 >*/
    __vo uint32_t IOCFG25;      /*!< Configuration of DIO25 >*/
    __vo uint32_t IOCFG26;      /*!< Configuration of DIO26 >*/
    __vo uint32_t IOCFG27;      /*!< Configuration of DIO27 >*/
    __vo uint32_t IOCFG28;      /*!< Configuration of DIO28 >*/
    __vo uint32_t IOCFG29;      /*!< Configuration of DIO29 >*/
    __vo uint32_t IOCFG30;      /*!< Configuration of DIO30 >*/
    __vo uint32_t IOCFG31;      /*!< Configuration of DIO31 >*/

}IOC_RegDef_t;

/* PARAMS STRUCT */
typedef struct
{
    __vo uint8_t GPIO_MODE;        /*!< GPIO Mode Selection => Check InOutSelect for possible values >*/
    __vo uint32_t OUT_PINS;        /*!< OUT Mode Pins Selection >*/
    __vo uint32_t IN_PINS;         /*!< IN Mode Pins Selection >*/
    __vo uint32_t INT_PINS;        /*!< INT Mode Pins Selection >*/
    __vo uint8_t HYST_EN;          /*!< HYSTERESIS Selection => 0 For Disable, 1 For Enable >*/
    __vo uint8_t IOMODE;           /*!< IO Mode Selection => Check IOMODE_SEL for possible values >*/
    __vo uint8_t WU_CFG;           /*!< Wake-Up Configuration => Check WU_SEL for possible values >*/
    __vo uint8_t EDGE_DET;         /*!< Edge Detect Mode Selection => Check EDGE_DET_SEL for possible values >*/
    __vo uint8_t PULL_CTL;         /*!< Pull control Mode Selection => Check PULL_CTL_SEL for possible values >*/
    __vo uint8_t SLEW_RED;         /*!< Slew Rate Configuration => 0 For Normal slew rate, 1 For reduced slew rate >*/
    __vo uint8_t IOCURR;           /*!< IO Current Mode Selection => Check IOCURR_SEL for possible values >*/
    __vo uint8_t IOSTR;            /*!< IO Drive Strength Mode Selection => Check IOSTR_SEL for possible values >*/
    __vo uint8_t PORT_ID;          /*!< Selects usage for DIO => Only GPIO Mode supported by this driver >*/
    void (*callbackFxn1)(void);    /*!< CallBack Functions used for Interrupt Handling (Only two supported by this driver) >*/
    void (*callbackFxn2)(void);    /*!< CallBack Functions used for Interrupt Handling >*/

}GPIO_Params_t;

/* HANDLE STRUCT */
typedef struct
{
    IOC_RegDef_t*  IOC_RegDef;
    GPIO_RegDef_t* GPIO_RegDef;
    GPIO_Params_t Params;

}GPIO_Handle_t;


/* ======= Parameter Settings ======= */

/* Mode Settings */
typedef enum{
    Input,                  /*!< Input Mode >*/
    Input_IE,               /*!< Interrupt Mode >*/
    Output,                 /*!< Output Mode >*/
    Input_Output_IE,        /*!< Input Output and Interrupt All in once mode >*/
}InOutSelect;

/* Wake-Up Settings */
typedef enum{
    No_WakeUp,              /*!<  No wake-up */
    WakeUp_Low = 2,         /*!< Wakes up from shutdown if this pad is going low. >*/
    WakeUp_High,            /*!< Wakes up from shutdown if this pad is going high.  >*/
}WU_SEL;


/* IO CURRENT Settings */
typedef enum{
    Low_Current_Mode,       /*!< 2MA : Low-Current (LC) mode: Min 2 mA when IOSTR is set to AUTO >*/
    High_Current_Mode,      /*!< 4MA : High-Current (HC) mode: Min 4 mA when IOSTR is set to AUTO >*/
    Extended_Current_Mode,  /*!< 4_8MA : Extended-Current (EC) mode: Min 8 mA for double
                             * drive strength IOs (min 4 mA for normal IOs) when IOSTR is set to AUTO >*/
}IOCURR_SEL;

/* IO STRENGTH Settings */
typedef enum{
    Auto_Drive_Str,         /*!< Automatic drive strength, controlled by AON BATMON based on battery voltage. (min 2 mA @VDDS) >*/
    Min_Drive_Str,          /*!< Minimum drive strength, controlled by AON_IOC:IOSTRMIN (min 2 mA @3.3V with default values) >*/
    Med_Drive_Str,          /*!< Medium drive strength, controlled by AON_IOC:IOSTRMED (min 2 mA @2.5V with default values) >*/
    Max_Drive_Str,          /*!< Maximum drive strength, controlled by AON_IOC:IOSTRMAX (min 2 mA @1.8V with default values) >*/
}IOSTR_SEL;

/* PULL_CTL Settings */
typedef enum{
    PULLDWN = 1,            /*!< Pull Down >*/
    PULLUP,                 /*!< Push Up >*/
    NOPULL,                 /*!< No Pull >*/
}PULL_CTL_SEL;

/* EDGE_DET Settings */
typedef enum{
    NOEDGE,                 /*!< No Edge Detection >*/
    NEGEDGE,                /*!< Negative Edge Detection >*/
    POSEDGE,                /*!< Positive Edge Detection >*/
    POSNEGEDGE,             /*!< Negative and Positive Edge Detection >*/
}EDGE_DET_SEL;

/* IOMODE Settings */
typedef enum{
    NORMAL,                 /*!< Normal Input-Output >*/
    INV,                    /*!< Inverted Input-Output >*/
    OPENDR = 4,             /*!< Open Drain Normal Input-Output >*/
    OPENDR_INV = 5,         /*!< Open Drain Inverted Input-Output >*/
    OPENSRC = 6,            /*!< Open Source Normal Input-Output >*/
    OPENSRC_INV = 7,        /*!< Open Source Inverted Input-Output >*/
}IOMODE_SEL;

/* Function Prototypes */
void GPIO_Init(GPIO_Handle_t* pGPIOx);
void GPIO_Params_Reset(GPIO_Handle_t* pGPIOx);
void GPIO_DeInit(GPIO_Handle_t* pGPIOx);
void GPIO_Toggle(GPIO_Handle_t* pGPIOx,uint32_t PinNo);
void GPIO_Output(GPIO_Handle_t* pGPIOx,uint32_t PinNo);
uint32_t GPIO_Input(GPIO_Handle_t* pGPIOx,uint32_t PinNo);
static inline void IOCIntRegister(void (*pfnHandler)(void));
static void GPIOIntClear(void);
static void findSetBits(uint32_t num_INT, uint32_t num_IN, uint32_t num_OUT);
static inline void IOCUnIntRegister();


#endif /* CC1310_GPIO_H_ */

