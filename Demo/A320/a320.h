#ifndef a320_h
#define a320_h
/*******************************************************************************
a320.h - Modified from lpc210x.h by BingYao for A320 board Register defs

THE SOFTWARE IS DELIVERED "AS IS" WITHOUT WARRANTY OR CONDITION OF ANY KIND, 
EITHER EXPRESS, IMPLIED OR STATUTORY. THIS INCLUDES WITHOUT LIMITATION ANY 
WARRANTY OR CONDITION WITH RESPECT TO MERCHANTABILITY OR FITNESS FOR ANY 
PARTICULAR PURPOSE, OR AGAINST THE INFRINGEMENTS OF INTELLECTUAL PROPERTY RIGHTS 
OF OTHERS.
           
This file may be freely used for commercial and non-commercial applications, 
including being redistributed with any tools.

If you find a problem with the file, please report it so that it can be fixed.

Created by Sten Larsson (sten_larsson at yahoo com)

Edited by Richard Barry.
*******************************************************************************/

#define REG8  (volatile unsigned char*)
#define REG16 (volatile unsigned short*)
#define REG32 (volatile unsigned int*)


/*##############################################################################
## MISC
##############################################################################*/

/*##############################################################################
## VECTORED INTERRUPT CONTROLLER
##############################################################################*/
#define VICIRQSource	(*(REG32 (0x98800000)))
#define VICIRQMask 	(*(REG32 (0x98800004)))
#define VICIRQClear	(*(REG32 (0x98800008)))
#define VICIRQMode	(*(REG32 (0x9880000C))) /* 0: level, 1: edge */
#define VICIRQLevel	(*(REG32 (0x98800010))) /* 0 - Active High or rising edge, 1 - Active Low or falling edge */
#define VICIRQStatus	0x98800014

extern int VICVectISR[32];
#define VICVectAddr0	VICVectISR[0]
#define VICVectAddr1	VICVectISR[1]
#define VICVectAddr2	VICVectISR[2]
#define VICVectAddr3	VICVectISR[3]
#define VICVectAddr4	VICVectISR[4]
#define VICVectAddr5	VICVectISR[5]
#define VICVectAddr6	VICVectISR[6]
#define VICVectAddr7	VICVectISR[7]
#define VICVectAddr8	VICVectISR[8]
#define VICVectAddr9	VICVectISR[8]
#define VICVectAddr10	VICVectISR[10]
#define VICVectAddr11	VICVectISR[11]
#define VICVectAddr12	VICVectISR[12]
#define VICVectAddr13	VICVectISR[13]
#define VICVectAddr14	VICVectISR[14]
#define VICVectAddr15	VICVectISR[15]
#define VICVectAddr16	VICVectISR[16]
#define VICVectAddr17	VICVectISR[17]
#define VICVectAddr18	VICVectISR[18]
#define VICVectAddr19	VICVectISR[19]
#define VICVectAddr20	VICVectISR[20]
#define VICVectAddr21	VICVectISR[21]
#define VICVectAddr22	VICVectISR[22]
#define VICVectAddr23	VICVectISR[23]
#define VICVectAddr24	VICVectISR[24]
#define VICVectAddr25	VICVectISR[25]

/*##############################################################################
## PCB - Pin Connect Block
##############################################################################*/

#define PCB_PINSEL0     (*(REG32 (0xE002C000)))
#define PCB_PINSEL1     (*(REG32 (0xE002C004)))

/*##############################################################################
## SMC - SMC address for LED output
##############################################################################*/
#define SMC_LED_ADDR	(*(REG32 (0x902FFFFC)))

/*##############################################################################
## UART0 / UART1
##############################################################################*/

/* ---- UART 0 --------------------------------------------- */
#define UART0_RBR       (*(REG32 (0x98200000)))
#define UART0_THR       (*(REG32 (0x98200000)))
#define UART0_IER       (*(REG32 (0x98200004)))
#define UART0_IIR       (*(REG32 (0x98200008)))
#define UART0_FCR       (*(REG32 (0x98200008)))
#define UART0_LCR       (*(REG32 (0x9820000C)))
#define UART0_MCR       (*(REG32 (0x98200010)))
#define UART0_LSR       (*(REG32 (0x98200014)))
#define UART0_SPR       (*(REG32 (0x9820001C)))
#define UART0_DLL       (*(REG32 (0x98200000)))
#define UART0_DLM       (*(REG32 (0x98200004)))

/* ---- UART 1 --------------------------------------------- */
#define UART1_RBR       (*(REG32 (0x98300000)))
#define UART1_THR       (*(REG32 (0x98300000)))
#define UART1_IER       (*(REG32 (0x98300004)))
#define UART1_IIR       (*(REG32 (0x98300008)))
#define UART1_FCR       (*(REG32 (0x98300008)))
#define UART1_LCR       (*(REG32 (0x9830000C)))
#define UART1_MCR       (*(REG32 (0x98300010)))
#define UART1_MSR       (*(REG32 (0x98300018)))
#define UART1_LSR       (*(REG32 (0x98300014)))
#define UART1_SPR       (*(REG32 (0x9830001C)))
#define UART1_DLL       (*(REG32 (0x98300000)))
#define UART1_DLM       (*(REG32 (0x98300004)))


/*##############################################################################
## I2C
##############################################################################*/

#define I2C_I2CONSET    (*(REG32 (0xE001C000)))
#define I2C_I2STAT      (*(REG32 (0xE001C004)))
#define I2C_I2DAT       (*(REG32 (0xE001C008)))
#define I2C_I2ADR       (*(REG32 (0xE001C00C)))
#define I2C_I2SCLH      (*(REG32 (0xE001C010)))
#define I2C_I2SCLL      (*(REG32 (0xE001C014)))
#define I2C_I2CONCLR    (*(REG32 (0xE001C018)))


/*##############################################################################
## SPI - Serial Peripheral Interface
##############################################################################*/

#define SPI_SPCR        (*(REG32 (0xE0020000)))
#define SPI_SPSR        (*(REG32 (0xE0020004)))
#define SPI_SPDR        (*(REG32 (0xE0020008)))
#define SPI_SPCCR       (*(REG32 (0xE002000C)))
#define SPI_SPTCR       (*(REG32 (0xE0020010)))
#define SPI_SPTSR       (*(REG32 (0xE0020014)))
#define SPI_SPTOR       (*(REG32 (0xE0020018)))
#define SPI_SPINT       (*(REG32 (0xE002001C)))


/*##############################################################################
## Timer 0 and Timer 1
##############################################################################*/

/* ---- Timer 0 -------------------------------------------- */
#define T0_COUNTER	(*(REG32 (0x98400000)))
#define T0_LOAD 	(*(REG32 (0x98400004)))
#define T0_MATCH1 	(*(REG32 (0x98400008)))
#define T0_MATCH2 	(*(REG32 (0x9840000C)))

/* ---- Timer 1 -------------------------------------------- */
#define T1_COUNTER	(*(REG32 (0x98400010)))
#define T1_LOAD 	(*(REG32 (0x98400014)))
#define T1_MATCH1 	(*(REG32 (0x98400018)))
#define T1_MATCH2 	(*(REG32 (0x9840001C)))

/* ---- Timer 2 -------------------------------------------- */
#define T2_COUNTER	(*(REG32 (0x98400020)))
#define T2_LOAD 	(*(REG32 (0x98400024)))
#define T2_MATCH1 	(*(REG32 (0x98400028)))
#define T2_MATCH2 	(*(REG32 (0x9840002C)))

/* ---- Timer Control ------------------------------------- */
#define TM_CONTROL	(*(REG32 (0x98400030)))
#define TM_INTRSTS 	(*(REG32 (0x98400034)))
#define TM_INTRMASK 	(*(REG32 (0x98400038)))

/*##############################################################################
## PWM
##############################################################################*/

#define PWM_IR          (*(REG32 (0xE0014000)))
#define PWM_TCR         (*(REG32 (0xE0014004)))
#define PWM_TC          (*(REG32 (0xE0014008)))
#define PWM_PR          (*(REG32 (0xE001400C)))
#define PWM_PC          (*(REG32 (0xE0014010)))
#define PWM_MCR         (*(REG32 (0xE0014014)))
#define PWM_MR0         (*(REG32 (0xE0014018)))
#define PWM_MR1         (*(REG32 (0xE001401C)))
#define PWM_MR2         (*(REG32 (0xE0014020)))
#define PWM_MR3         (*(REG32 (0xE0014024)))
#define PWM_MR4         (*(REG32 (0xE0014040)))
#define PWM_MR5         (*(REG32 (0xE0014044)))
#define PWM_MR6         (*(REG32 (0xE0014048)))
#define PWM_EMR         (*(REG32 (0xE001403C)))
#define PWM_PCR         (*(REG32 (0xE001404C)))
#define PWM_LER         (*(REG32 (0xE0014050)))
#define PWM_CCR         (*(REG32 (0xE0014028)))
#define PWM_CR0         (*(REG32 (0xE001402C)))
#define PWM_CR1         (*(REG32 (0xE0014030)))
#define PWM_CR2         (*(REG32 (0xE0014034)))
#define PWM_CR3         (*(REG32 (0xE0014038)))

/*##############################################################################
## RTC
##############################################################################*/

/* ---- RTC: Miscellaneous Register Group ------------------ */
#define RTC_ILR         (*(REG32 (0xE0024000)))
#define RTC_CTC         (*(REG32 (0xE0024004)))
#define RTC_CCR         (*(REG32 (0xE0024008)))  
#define RTC_CIIR        (*(REG32 (0xE002400C)))
#define RTC_AMR         (*(REG32 (0xE0024010)))
#define RTC_CTIME0      (*(REG32 (0xE0024014)))
#define RTC_CTIME1      (*(REG32 (0xE0024018)))
#define RTC_CTIME2      (*(REG32 (0xE002401C)))

/* ---- RTC: Timer Control Group --------------------------- */
#define RTC_SEC         (*(REG32 (0xE0024020)))
#define RTC_MIN         (*(REG32 (0xE0024024)))
#define RTC_HOUR        (*(REG32 (0xE0024028)))
#define RTC_DOM         (*(REG32 (0xE002402C)))
#define RTC_DOW         (*(REG32 (0xE0024030)))
#define RTC_DOY         (*(REG32 (0xE0024034)))
#define RTC_MONTH       (*(REG32 (0xE0024038)))
#define RTC_YEAR        (*(REG32 (0xE002403C)))

/* ---- RTC: Alarm Control Group --------------------------- */
#define RTC_ALSEC       (*(REG32 (0xE0024060)))
#define RTC_ALMIN       (*(REG32 (0xE0024064)))
#define RTC_ALHOUR      (*(REG32 (0xE0024068)))
#define RTC_ALDOM       (*(REG32 (0xE002406C)))
#define RTC_ALDOW       (*(REG32 (0xE0024070)))
#define RTC_ALDOY       (*(REG32 (0xE0024074)))
#define RTC_ALMON       (*(REG32 (0xE0024078)))
#define RTC_ALYEAR      (*(REG32 (0xE002407C)))

/* ---- RTC: Reference Clock Divider Group ----------------- */
#define RTC_PREINT      (*(REG32 (0xE0024080)))
#define RTC_PREFRAC     (*(REG32 (0xE0024084)))


/*##############################################################################
## WD - Watchdog
##############################################################################*/

#define WD_WDMOD        (*(REG32 (0xE0000000)))
#define WD_WDTC         (*(REG32 (0xE0000004)))
#define WD_WDFEED       (*(REG32 (0xE0000008)))
#define WD_WDTV         (*(REG32 (0xE000000C)))


/*##############################################################################
## Power Management Unit (include clock source and GPIO control)
##############################################################################*/
#define PMU_OSCC        (*(REG32 (0x98100008)))

#endif /* a320_h */

