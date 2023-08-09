/*
 * cc1310.h
 *
 *  Created on: 26 Tem 2023
 *      Author: erdem
 */

#ifndef INC_CC1310_H_
#define INC_CC1310_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))


/* Base addresses of AHB1 Peripherals */

#define GPIO_BASEADDR               0x40022000U
#define IOC_BASEADDR                0x40081000U


/* Peripheral Register Definition Structures */

typedef struct                        // GPIO STRUCT
{
    __vo uint32_t DOUT3_0;                      // GPIO port mode register
    __vo uint32_t DOUT7_4;
    __vo uint32_t DOUT11_8;
    __vo uint32_t DOUT15_12;
    __vo uint32_t DOUT19_16;
    __vo uint32_t DOUT23_20;
    __vo uint32_t DOUT27_24;
    __vo uint32_t DOUT31_28;
    __vo uint32_t EMPTY[24];
    __vo uint32_t DOUT31_0;
    __vo uint32_t EMPTY1[3];
    __vo uint32_t DOUTSET31_0;                      // GPIO port mode register
    __vo uint32_t EMPTY2[3];
    __vo uint32_t DOUTCLR31_0;
    __vo uint32_t EMPTY3[3];
    __vo uint32_t DOUTTGL31_0;
    __vo uint32_t EMPTY4[3];
    __vo uint32_t DIN31_0;
    __vo uint32_t EMPTY5[3];
    __vo uint32_t DOE31_0;
    __vo uint32_t EMPTY6[3];
    __vo uint32_t EVFLAGS31_0;
}GPIO_RegDef_t;

typedef struct                        // IOC STRUCT
{
    __vo uint32_t IOCFG0;                      // GPIO port mode register
    __vo uint32_t IOCFG1;
    __vo uint32_t IOCFG2;
    __vo uint32_t IOCFG3;
    __vo uint32_t IOCFG4;
    __vo uint32_t IOCFG5;
    __vo uint32_t IOCFG6;
    __vo uint32_t IOCFG7;
    __vo uint32_t IOCFG8;
    __vo uint32_t IOCFG9;
    __vo uint32_t IOCFG10;
    __vo uint32_t IOCFG11;
    __vo uint32_t IOCFG12;
    __vo uint32_t IOCFG13;
    __vo uint32_t IOCFG14;
    __vo uint32_t IOCFG15;
    __vo uint32_t IOCFG16;
    __vo uint32_t IOCFG17;
    __vo uint32_t IOCFG18;
    __vo uint32_t IOCFG19;
    __vo uint32_t IOCFG20;
    __vo uint32_t IOCFG21;
    __vo uint32_t IOCFG22;
    __vo uint32_t IOCFG23;
    __vo uint32_t IOCFG24;
    __vo uint32_t IOCFG25;
    __vo uint32_t IOCFG26;
    __vo uint32_t IOCFG27;
    __vo uint32_t IOCFG28;
    __vo uint32_t IOCFG29;
    __vo uint32_t IOCFG30;
    __vo uint32_t IOCFG31;

}IOC_RegDef_t;




/* Peripheral Definitions */

#define GPIO               ((GPIO_RegDef_t*)GPIO_BASEADDR)
#define IOC                ((IOC_RegDef_t*)IOC_BASEADDR)



/* Some Generic Macros */

#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET            SET






#endif /* INC_CC1310_H_ */
