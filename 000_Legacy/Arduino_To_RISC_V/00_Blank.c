/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 00_Blank
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program is a blank program for 
 *                      the CH32V003F4P6 microcontroller 
 * 
 * Hardware Description: A LinkE Programmer is connected to the device 
 *
 * Created December 29th, 2025, 10:20 PM
 * Updated December 29th, 2025, 10:20 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"

void initMain()
{
    // put your setup code here
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
}

int main(void)
{
    initMain();

    while(1)
    {
       // put your main looping code here 
    }
}

