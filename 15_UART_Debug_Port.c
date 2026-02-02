/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3V
 * Program: 15_UART_Debug_Port.c
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.0
 *
 * Program Description: Initializes a UART debug port and prints a startup banner plus a
 *                      1 Hz heartbeat message using printf().
 *
 * Hardware Description: An LED is connected via a 1k resistor connected to PD0.
 *                       Uses the default UART pins configured by WCH's debug library
 *                       as we use a WCH-Link for handling UART communication. 
 *
 * Created August 10th, 2025, 7:30 PM
 * Updated August 10th, 2025, 7:45 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

static void initMain(void);
static void initUART(void);

/*******************************************************************************
 * Function: void initMain()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: initMain()
 ******************************************************************************/
static void initMain(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}

/*******************************************************************************
 * Function: static void initUART(void)
 *
 * Returns: Nothing
 *
 * Description: Uses SDI if enabled, otherwise initializes USART at 115200.
 * 
 * Usage: initUART(void)
 ******************************************************************************/
static void initUART(void)
{
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
}

/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void)
{
    initMain();
    initUART();

    printf("\r\n===== UART Debug Port Ready =====\r\n");
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    uint32_t hb = 0;
    while (1)
    {
        Delay_Ms(1000);
        printf("HB:%lu\r\n", (unsigned long)hb++);
    }
}
