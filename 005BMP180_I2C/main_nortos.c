/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 * Author: Nihat Emre Albayrak
 */

/*
 *  ======== main_nortos.c ========
 */
#include <stdint.h>
#include <stddef.h>

#include <NoRTOS.h>

/* Example/Board Header files */
#include "Board.h"

extern void *mainThread(void *arg0);

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions */
    Board_initGeneral();

    /* Start NoRTOS */
    NoRTOS_start();

    /* Call mainThread function */
    mainThread(NULL);

    while (1);
}
