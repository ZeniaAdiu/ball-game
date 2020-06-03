//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - I2C
// Application Overview - The objective of this application is act as an I2C
//                        diagnostic tool. The demo application is a generic
//                        implementation that allows the user to communicate
//                        with any I2C device over the lines.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup i2c_demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"

#include "pin_mux_config.h"

// Common interface includes
#include "uart_if.h"
#include "i2c_if.h"

// OLED screen includes
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "Adafruit_GFX.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.4.0"
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************


int convert(signed char data){
    float delta = ((float)data);
    return (int)delta;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function handling the I2C example
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void main()
{
    //
    // Initialize board configurations
    //
    BoardInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Enable Transmit/Receiving FIFO
    //
    MAP_SPIFIFOEnable(GSPI_BASE, SPI_TX_FIFO || SPI_RX_FIFO);

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initializing the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Send the string to slave.
    //
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);


    MAP_SPICSEnable(GSPI_BASE);
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    Adafruit_Init();
    fillScreen(BLACK);

    int x = 64;//center
    int y = 64;

    int xmove = 0;//axis change
    int ymove = 0;

    unsigned char tempX[64];//buffer
    unsigned char tempY[64];

    unsigned char xaxis = 0x3;//address
    unsigned char yaxis = 0x5;

    while(1)
    {
        I2C_IF_Write(0x18, &xaxis, 1, 0);
        I2C_IF_Read(0x18, &tempX[0], 1);
        I2C_IF_Write(0x18, &yaxis, 1, 0);
        I2C_IF_Read(0x18, &tempY[0], 1);

        xmove = convert((signed char) tempY[0]);
        ymove = convert((signed char) tempX[0]);

        if (x < 123 && x > 5) {//can move
            x = x - xmove;
        }

        if (x > 122) {//out of range, at the boundary
            x = 121;
            drawCircle(x,y,4, BLUE);
            fillCircle(x,y,4, BLUE);

            int delay;
            for(delay=0; delay<100; delay=delay+1);
            drawCircle(x,y,4, BLACK);
            fillCircle(x,y,4, BLACK);
        }

        if (x < 5) {
            x = 7;
            drawCircle(x,y,4, BLUE);
            fillCircle(x,y,4, BLUE);
            //delay
            int delay;
            for(delay=0; delay<100; delay=delay+1);
            drawCircle(x,y,4, BLACK);
            fillCircle(x,y,4, BLACK);
        }

        if (y < 123 && y > 5) {//can move
            y = y - ymove;
        }

        if (y > 122) {//nboundary
            y = 121;
            drawCircle(x,y,4, BLUE);
            fillCircle(x,y,4, BLUE);

            int delay;
            for(delay=0; delay<100; delay=delay+1);
            drawCircle(x,y,4, BLACK);
            fillCircle(x,y,4, BLACK);
        }

        if (y < 5) {
            y = 7;
            drawCircle(x,y,4, BLUE);
            fillCircle(x,y,4, BLUE);

            int delay;
            for(delay=0; delay<100; delay=delay+1);
            drawCircle(x,y,4, BLACK);
            fillCircle(x,y,4, BLACK);
        }

        drawCircle(x,y,4, BLUE);
        fillCircle(x,y,4, BLUE);
        drawCircle(x,y,4, BLACK);
        fillCircle(x,y,4, BLACK);
    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************
