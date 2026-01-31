/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3V
 * Program: 16_UART_Echo_Add.c
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.0
 *
 * Program Description: This program reads a line of input from UART (terminated by '\n' or '\r') 
 *                      and if the input is a number, it adds 1 and sends the result, 
 *                      if the input is not a number it replies with "yes?" 
 *
 * Hardware Description: An LED is connected via a 1k resistor connected to PD0.
 *                       Uses the default UART pins configured by WCH's debug library
 *                       as we use a WCH-Link for handling UART communication. 
 *
 * Created August  10th, 2025, 08:09 PM
 * Updated January 03rd, 2025, 12:40 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define RX_BUF_SIZE 32

static int is_number(const char *s);
static void USART1_Init(void);


/*******************************************************************************
 * Function: void initMain()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: initMain()
 ******************************************************************************/
static void initMain(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}


/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void) {
    initMain();
    USART1_Init();

    char buf[RX_BUF_SIZE];
    uint8_t idx = 0;

    while (1) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
            char c = USART_ReceiveData(USART1) & 0xFF;

            if (c == '\r' || c == '\n') {
                buf[idx] = '\0';
                idx = 0;

                if (is_number(buf)) {
                    long val = strtol(buf, NULL, 10);
                    char out[16];
                    snprintf(out, sizeof(out), "%ld\r\n", val + 1);
                    for (char *p = out; *p; p++) {
                        USART_SendData(USART1, (uint8_t)*p);
                        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
                    }
                } else if (buf[0] != '\0') {
                    const char reply[] = "yes?\r\n";
                    for (const char *p = reply; *p; p++) {
                        USART_SendData(USART1, (uint8_t)*p);
                        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
                    }
                }
            } else {
                if (idx < RX_BUF_SIZE - 1) buf[idx++] = c;
            }
        }
    }
}


/*******************************************************************************
 * Function: static void USART1_Init(void)
 *
 * Returns: Nothing
 *
 * Description: Initalizes the UART module 
 * 
 * Usage: USART1_Init()
 ******************************************************************************/
static void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    // TX = PD5 (AF push-pull), RX = PD6 (floating input)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/*******************************************************************************
 * Function: static int is_number(const char *s)
 *
 * Returns: If is number or not
 *
 * Description: Checks if character is a number
 * 
 * Usage:  if (is_number(buf))
 ******************************************************************************/
static int is_number(const char *s) {
    if (*s == '\0') return 0;
    while (*s) {
        if (!isdigit((unsigned char)*s)) return 0;
        s++;
    }
    return 1;
}