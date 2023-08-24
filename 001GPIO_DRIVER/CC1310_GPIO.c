/*
 * CC1310_GPIO.c
 *
 *  Created on: 22 Aðu 2023
 *      Author: Nihat Emre Albayrak
 */

#include "CC1310_GPIO.h"

uint8_t numberOfDigits_IN = 0;          /* Configured Input pin count */
uint8_t indexes_IN[TOTAL_PIN_NUMBER];   /* Configured Input pin numbers */
uint8_t numberOfDigits_INT = 0;         /* Configured Interrupt pin count */
uint8_t indexes_INT[TOTAL_PIN_NUMBER];  /* Configured Interrupt pin numbers */
uint8_t numberOfDigits_OUT = 0;         /* Configured Output pin count */
uint8_t indexes_OUT[TOTAL_PIN_NUMBER];  /* Configured Interrupt pin numbers */
uint8_t Init_Count = 0;                 /* Init func. call count */


GPIO_Handle_t* Dummy_handle;            /* Dummy Handle used in Interrupt function */

/**
 * @brief Reset GPIO Parameters with Reset values.
 * To hold Interrupt function address, count is checked
 * @retval None.
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * */
void GPIO_Params_Reset(GPIO_Handle_t* pGPIOx)
{
    GPIO_Params_t tempParams = {
        .GPIO_MODE = 0,
        .OUT_PINS = 0,
        .IN_PINS = 0,
        .INT_PINS = 0,
        .HYST_EN = 0,
        .IOMODE = 0,
        .WU_CFG = 0,
        .EDGE_DET = 0,
        .PULL_CTL = 0x3,
        .SLEW_RED = 0,
        .IOCURR = 0,
        .IOSTR = 0,
        .PORT_ID = 0,
        tempParams.callbackFxn1 = NULL,
        tempParams.callbackFxn2 = NULL
    };
    if(Init_Count != 0){
        // Store the existing callbackFxn values
        void (*savedCallbackFxn1)(void) = pGPIOx->Params.callbackFxn1;
        void (*savedCallbackFxn2)(void) = pGPIOx->Params.callbackFxn2;

        tempParams.callbackFxn1 = savedCallbackFxn1;
        tempParams.callbackFxn2 = savedCallbackFxn2;
    }

    pGPIOx->Params = tempParams;
}

/**
 * @brief Initialize GPIO with configured Parameters.
 * Can be recalled to initialize different port with different configurations.
 * @retval None.
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * */
void GPIO_Init(GPIO_Handle_t* pGPIOx)
{
    /* Find set bits */
    findSetBits(pGPIOx->Params.INT_PINS, pGPIOx->Params.IN_PINS, pGPIOx->Params.OUT_PINS);

    if(pGPIOx->Params.GPIO_MODE == Input)
    {
        /* Disable output for input pins */
        pGPIOx->GPIO_RegDef->DOE31_0 &= ~(pGPIOx->Params.IN_PINS);

        uint32_t temp = 0;
        uint32_t temp2 = 0;
        temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
        temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
        temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
        temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
        temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
        temp |= (SET << IE_ADDR);
        /* Enable input for input pins and set other configurations */

        uint8_t pin_id;
        for(pin_id = 0 ; pin_id < numberOfDigits_IN ; pin_id++)
        {
            /* Find IOCFG Reg number to initialize */
            temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_IN[pin_id]);
            /* Configure only Non-Reserved Bits */
            temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
            *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_IN[pin_id]) = temp2 ;

        }

    }
    else if(pGPIOx->Params.GPIO_MODE == Output){

        uint32_t temp = 0;
        uint32_t temp2 = 0;

        temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
        temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
        temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
        temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
        temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
        /* Disable input for output pins */
        temp &= ~(SET << IE_ADDR);

        /* Enable output for output pins */
        pGPIOx->GPIO_RegDef->DOE31_0 |= pGPIOx->Params.OUT_PINS;

        uint8_t pin_id;
        for(pin_id = 0 ; pin_id < numberOfDigits_OUT ; pin_id++)
        {
            temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_OUT[pin_id]);
            temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
            *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_OUT[pin_id]) = temp2 ;
        }
    }
    else if(pGPIOx->Params.GPIO_MODE == Input_IE)
    {
        Dummy_handle = pGPIOx;

        uint32_t temp = 0;
        uint32_t temp2 = 0;

        /* Disable output for input pins */
        pGPIOx->GPIO_RegDef->DOE31_0 &= ~(pGPIOx->Params.INT_PINS);

        temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
        temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
        temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
        temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
        temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
        temp |= (pGPIOx->Params.EDGE_DET << EDGE_DET_ADDR);
        /* Enable Interrupts for selected input pins */
        temp |= (SET << EDGE_IRQ_EN_ADDR);
        /* Enable input for input pins and set other configurations */
        temp |= (SET << IE_ADDR);


        uint8_t pin_id;
        for(pin_id = 0 ; pin_id < numberOfDigits_INT ; pin_id++)
        {
            temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_INT[pin_id]);
            temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
            *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_INT[pin_id]) = temp2 ;

        }

        /* Initialize Interrupt function to table */
        IOCIntRegister(&GPIOIntClear);
        Init_Count++;   // Mark that initialization was performed before
    }

    /* Initialize pins once in for all */
    else if(pGPIOx->Params.GPIO_MODE == Input_Output_IE)
    {
        Dummy_handle = pGPIOx;

        uint32_t temp = 0;
        uint32_t temp2 = 0;
        uint8_t pin_id;

        if(pGPIOx->Params.IN_PINS != 0)
        {
            pGPIOx->GPIO_RegDef->DOE31_0 &= ~(pGPIOx->Params.IN_PINS);

            temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
            temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
            temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
            temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
            temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
            temp |= (SET << IE_ADDR);

            for(pin_id = 0 ; pin_id < numberOfDigits_IN ; pin_id++)
            {
                temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_IN[pin_id]);
                temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
                *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_IN[pin_id]) = temp2 ;

            }
        }

        if(pGPIOx->Params.INT_PINS != 0)
        {
            temp = 0;
            pGPIOx->GPIO_RegDef->DOE31_0 &= ~(pGPIOx->Params.INT_PINS);

            temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
            temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
            temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
            temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
            temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
            temp |= (pGPIOx->Params.EDGE_DET << EDGE_DET_ADDR);
            temp |= (SET << EDGE_IRQ_EN_ADDR);
            temp |= (SET << IE_ADDR);

            for(pin_id = 0 ; pin_id < numberOfDigits_INT ; pin_id++)
            {
                temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_INT[pin_id]);
                temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
                *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_INT[pin_id]) = temp2 ;

            }

            IOCIntRegister(&GPIOIntClear);
            Init_Count++;
        }

        if(pGPIOx->Params.OUT_PINS != 0)
        {
            temp = 0;
            temp |= (pGPIOx->Params.HYST_EN << HYST_EN_ADDR);
            temp |= (pGPIOx->Params.IOMODE << IOMODE_ADDR);
            temp |= (pGPIOx->Params.PORT_ID << PORT_ID_ADDR);
            temp |= (pGPIOx->Params.PULL_CTL << PULL_CTL_ADDR);
            temp |= (pGPIOx->Params.SLEW_RED << SLEW_RED_ADDR);
            temp &= ~(SET << IE_ADDR);

            pGPIOx->GPIO_RegDef->DOE31_0 |= pGPIOx->Params.OUT_PINS;

            for(pin_id = 0 ; pin_id < numberOfDigits_OUT ; pin_id++)
            {
                temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_OUT[pin_id]);
                temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
                *(&pGPIOx->IOC_RegDef->IOCFG0 + indexes_OUT[pin_id]) = temp2 ;

            }
        }
    }

}

/**
 * @brief DeInitialize GPIO.
 * Resets all the configurations including Interrupts,
 * Disables any output even if previous state is on.
 * @retval None.
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * */
void GPIO_DeInit(GPIO_Handle_t* pGPIOx)
{
    uint8_t pin_id;
    uint32_t temp2;
    uint32_t temp = IOC_RESET_VALUE;
    for(pin_id = 0 ; pin_id < TOTAL_PIN_NUMBER ; pin_id++)
    {
        temp2 = *(&pGPIOx->IOC_RegDef->IOCFG0 + pin_id);
        temp2 = (temp2 & ~IOC_REG_BITMASK) | (temp & IOC_REG_BITMASK);
        *(&pGPIOx->IOC_RegDef->IOCFG0 + pin_id) = temp2 ;
    }
    /* Clear All */
    pGPIOx->GPIO_RegDef->DOUTCLR31_0 = ALL_BITMASK;
    /* Disable Outputs */
    pGPIOx->GPIO_RegDef->DOE31_0 = DISABLE;
    /* Delete Functions from table*/
    IOCUnIntRegister();
    Init_Count = 0;

}

/**
 * @brief Toggle GPIO Pin or Port.
 * @retval None.
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * @param uint32_t PinNo Pin number(s) which will be toggled.
 * */
void GPIO_Toggle(GPIO_Handle_t* pGPIOx,uint32_t PinNo)
{
    pGPIOx->GPIO_RegDef->DOUTTGL31_0 = PinNo;
}

/**
 * @brief Set corresponding Pin(s).
 * @retval None.
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * @param uint32_t PinNo Pin number(s) which will be set on.
 * */
void GPIO_Output(GPIO_Handle_t* pGPIOx,uint32_t PinNo)
{
    pGPIOx->GPIO_RegDef->DOUTSET31_0 |= PinNo;
}

/**
 * @brief Take input from corresponding Pin(s).
 * @retval Input value. Active low; ( Returns 0 if Input present )
 * @param GPIO_Handle_t* pGPIOx GPIO Handle pointer.
 * @param uint32_t PinNo Pin number(s) which will be sampled.
 * */
uint32_t GPIO_Input(GPIO_Handle_t* pGPIOx,uint32_t PinNo)
{
    return (pGPIOx->GPIO_RegDef->DIN31_0 & PinNo);
}

/**
 * @brief Registers a function as an interrupt handler in the dynamic vector table.And enables it.
 * Configured only to edit GPIO Interrupt.
 * @retval None.
 * @param void (*pfnHandler)(void) pfnHandler is a pointer to the function to register as interrupt handler.
 * */
static inline void IOCIntRegister(void (*pfnHandler)(void))
{
    // Register the interrupt handler.
    IntRegister(INT_AON_GPIO_EDGE, pfnHandler);

    // Enable the IO edge interrupt.
    IntEnable(INT_AON_GPIO_EDGE);
}

/**
 * @brief Unregisters an interrupt handler in the dynamic vector table.
 * Configured only to edit GPIO Interrupt.
 * @retval None.
 * @param None.
 * */
static inline void IOCUnIntRegister()
{
    // Register the interrupt handler.
    IntUnregister(INT_AON_GPIO_EDGE);

    // Enable the IO edge interrupt.
    IntDisable(INT_AON_GPIO_EDGE);
}

/**
 * @brief Callback function which is registered to the Vector Table.
 * Checks which Interrupt Function should be triggered. Initiates the function set in the main code.
 * @retval None.
 * @param None.
 * */
void GPIOIntClear(void)
{

    if(((GPIO->EVFLAGS31_0 >> indexes_INT[0]) & BIT1) == FLAG_SET)
    {
        /* Interrupt related to configured BIT */
        GPIO->EVFLAGS31_0 |= (SET << indexes_INT[0]);
        void (*callbackFxn)(void) = Dummy_handle->Params.callbackFxn1;
        callbackFxn();
    }
    else if(((GPIO->EVFLAGS31_0 >> indexes_INT[1]) & BIT1) == FLAG_SET)
    {
        /* Interrupt related to configured BIT */
        GPIO->EVFLAGS31_0 |= (SET << indexes_INT[1]);
        void (*callbackFxn)(void) = Dummy_handle->Params.callbackFxn2;
        callbackFxn();
    }

}

/**
 * @brief Finds Pin numbers and Pin count from a merged pin input(MASK)
 * For Performance considerations, variables are stored Globally.
 * @retval None.
 * @param uint32_t num_INT, merged interrupt pins input(MASK)
 * @param uint32_t num_IN, merged input pins input(MASK)
 * @param uint32_t num_OUT, merged output pins input(MASK)
 * */
static void findSetBits(uint32_t num_INT, uint32_t num_IN, uint32_t num_OUT)
{
    if(Init_Count == 0){
        numberOfDigits_INT = 0; // Initialize count to 0
    }
    numberOfDigits_IN = 0; // Initialize count to 0
    numberOfDigits_OUT = 0; // Initialize count to 0

    uint8_t index = 0; // Index to track the array position
    uint8_t i;

    /* Check all pin numbers possible */
    if(Dummy_handle->Params.INT_PINS != 0)
    {
        for (i = 0; i < TOTAL_PIN_NUMBER; i++)
        {
            if ((num_INT & (SET << i)) != 0)
            {
                indexes_INT[index++] = i; // Store the index
                numberOfDigits_INT++; // Increment the count
            }
        }
    }

    if(Dummy_handle->Params.IN_PINS != 0)
    {
        index = 0;
        for (i = 0; i < TOTAL_PIN_NUMBER; i++)
        {
            if ((num_IN & (SET << i)) != 0)
            {
                indexes_IN[index++] = i; // Store the index
                numberOfDigits_IN++; // Increment the count
            }
        }
    }


    if(Dummy_handle->Params.OUT_PINS != 0)
    {
        index = 0;
        for (i = 0; i < TOTAL_PIN_NUMBER; i++)
        {
            if ((num_OUT & (SET << i)) != 0)
            {
                indexes_OUT[index++] = i; // Store the index
                numberOfDigits_OUT++; // Increment the count
            }
        }
    }

}
