/*
    FreeRTOS V7.4.2 - Copyright (C) 2013 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.

    >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
    details. You should have received a copy of the GNU General Public License
    and the FreeRTOS license exception along with FreeRTOS; if not it can be
    viewed here: http://www.freertos.org/a00114.html and also obtained by
    writing to Real Time Engineers Ltd., contact details for whom are available
    on the FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, books, training, latest versions, 
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, and our new
    fully thread aware and reentrant UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High 
    Integrity Systems, who sell the code with commercial support, 
    indemnification and middleware, under the OpenRTOS brand.
    
    http://www.SafeRTOS.com - High Integrity Systems also provide a safety 
    engineered and independently SIL3 certified version for use in safety and 
    mission critical applications that require provable dependability.
*/

/*
	Changes from V2.4.0

		+ Made serial ISR handling more complete and robust.

	Changes from V2.4.1

		+ Split serial.c into serial.c and serialISR.c.  serial.c can be 
		  compiled using ARM or THUMB modes.  serialISR.c must always be
		  compiled in ARM mode.
		+ Another small change to cSerialPutChar().

	Changed from V2.5.1

		+ In cSerialPutChar() an extra check is made to ensure the post to
		  the queue was successful if then attempting to retrieve the posted
		  character.

*/

/* 
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0. 

	This file contains all the serial port components that can be compiled to
	either ARM or THUMB mode.  Components that must be compiled to ARM mode are
	contained in serialISR.c.
*/

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application includes. */
#include "serial.h"

/*-----------------------------------------------------------*/

/* Constants to setup and access the UART. */
#define serDLAB 			( ( unsigned char ) 0x80 )
#define serENABLE_INTERRUPTS		( ( unsigned char ) 0x03 )
#define serNO_PARITY			( ( unsigned char ) 0x00 )
#define ser1_STOP_BIT			( ( unsigned char ) 0x00 )
#define ser8_BIT_CHARS			( ( unsigned char ) 0x03 )
#define serFIFO_ON			( ( unsigned char ) 0x01 )
#define serCLEAR_FIFO			( ( unsigned char ) 0x06 )
#define serWANTED_CLOCK_SCALING 	( ( unsigned long ) 16 )

#define serLSB_DATA_READY		( ( unsigned long ) ( 1 << 0 ) )
#define serLSB_THR_EMPTY		( ( unsigned long ) ( 1 << 5 ) )

#define serHANDLE			( ( xComPortHandle ) 1 )
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud )
{
unsigned long ulDivisor;
xComPortHandle xReturn = serHANDLE;

	if( ulWantedBaud != ( unsigned long ) 0 )
	{
		portENTER_CRITICAL();
		{
			/* Setup the baud rate:  Calculate the divisor value. */
			if ( ulWantedBaud == 115200 )
			{
				ulDivisor = configUART_CLOCK_HZ / 1843200;
			}
			else if ( ulWantedBaud == 57600 )
			{
				ulDivisor = configUART_CLOCK_HZ / 921600;
			}
			else if ( ulWantedBaud == 38400 )
			{
				ulDivisor = configUART_CLOCK_HZ / 614400;
			}
			else if ( ulWantedBaud == 19200 )
			{
				ulDivisor = configUART_CLOCK_HZ / 307200;
			}
			else if ( ulWantedBaud == 14400 )
			{
				ulDivisor = configUART_CLOCK_HZ / 230400;
			}

			/* Set the DLAB bit so we can access the divisor. */
			UART0_LCR |= serDLAB;

			/* Setup the divisor. */
			UART0_DLL = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );
			ulDivisor >>= 8;
			UART0_DLM = ( unsigned char ) ( ulDivisor & ( unsigned long ) 0xff );

			UART0_LCR &= ~serDLAB;

			/* Turn on the FIFO's and clear the buffers. */
			UART0_FCR = ( serFIFO_ON | serCLEAR_FIFO );

			/* Setup transmission format. */
			UART0_LCR = serNO_PARITY | ser1_STOP_BIT | ser8_BIT_CHARS;

			/* Disable UART0 interrupts. */
			UART0_IER &= ~serENABLE_INTERRUPTS;
		}
		portEXIT_CRITICAL();
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( signed char *pcRxedChar )
{
	while (!(UART0_LSR & serLSB_DATA_READY)) ;

	if ((*pcRxedChar = UART0_RBR) == '\r')
		*pcRxedChar = '\n';

	return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialPutString( const char * pcString )
{
signed char *pxNext;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* Send each character in the string, one at a time. */
	pxNext = ( signed char * ) pcString;
	while( *pxNext )
	{
		if (*pxNext == '\n')
			xSerialPutChar( '\r' );

		xSerialPutChar( *pxNext );
		pxNext++;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( signed char cOutChar )
{
	portENTER_CRITICAL();
	{
		/* Is there space to write to the UART? */
		while (!(UART0_LSR & serLSB_THR_EMPTY)) ;

		UART0_THR = cOutChar;
	}
	portEXIT_CRITICAL();

	return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
	( void ) xPort;
}
/*-----------------------------------------------------------*/





	
