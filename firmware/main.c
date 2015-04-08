/*    Copyright (C) 2014 GP Orcullo
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <plib.h>
#include "hardware.h"
#include "stepgen.h"

#pragma config POSCMOD = OFF		/* Primary Oscillator disabled */
#pragma config FNOSC = FRCPLL		/* Fast RC Osc w/Div-by-N */
#pragma config FPLLODIV = DIV_2		/* PLL configured for 40MHz clock */
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPBDIV = DIV_1		/* Peripheral Clock Divisor */
#pragma config IESO = ON		/* Internal/External Switch Over disabled */
#pragma config FSOSCEN = OFF		/* Secondary Oscillator disabled */
#pragma config CP = OFF			/* Code Protect Disabled */
#pragma config FWDTEN = ON		/* Watchdog Timer Enable */
#pragma config WDTPS = PS4096		/* Watchdog Timer Postscaler */
#pragma config PMDL1WAY = OFF		/* Allow multiple PM configurations */
#pragma config IOL1WAY = OFF		/* Allow multiple PPS configurations */

#define BASEFREQ			80000
#define CORE_TICK_RATE	        	(SYS_FREQ/2/BASEFREQ)
#define SPIBUFSIZE			20
#define BUFSIZE				(SPIBUFSIZE/4)
#define SPI_TIMEOUT			1000L

static volatile uint32_t rxBuf[BUFSIZE], txBuf[BUFSIZE];
static volatile int spi_data_ready;

static void map_peripherals()
{
	/* unlock PPS sequence */
	SYSKEY = 0x0;			/* make sure it is locked */
	SYSKEY = 0xAA996655;		/* Key 1 */
	SYSKEY = 0x556699AA;		/* Key 2 */
	CFGCONbits.IOLOCK=0;		/* now it is unlocked */

	/* map SPI and PWM pins */
	PPSInput(4, SS2, RPB14);	/* CS */
	PPSInput(3, SDI2, RPB13);	/* MOSI */
	PPSOutput(2, RPB11, SDO2);	/* MISO */
	PPSOutput(1, RPA0, OC1);	/* PWM 0 */
	PPSOutput(2, RPA1, OC2);	/* PWM 1 */
	PPSOutput(4, RPB0, OC3);	/* PWM 2 */

	/* lock PPS sequence */
	CFGCONbits.IOLOCK=1;		/* now it is locked */
	SYSKEY = 0x0;			/* lock register access */
}

static void init_io_ports()
{
	/* disable all analog pins */
	ANSELA = 0x0;
	ANSELB = 0x0;

	/* configure inputs */
	TRISBSET = BIT_6 | BIT_7 | BIT_8 | BIT_9 |
		   BIT_13 | BIT_14 | BIT_15;

	/* configure_outputs */
	TRISACLR = BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 ;
	TRISBCLR = BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 |
		   BIT_5 | BIT_10 | BIT_11 | BIT_12;

}

static void init_spi()
{
	int i;

	SPI2CON = 0;		/* stop SPI 2, set Slave mode, 8 bits, std buffer */
	i = SPI2BUF;		/* clear rcv buffer */
	SPI2CON = 0<<8 | 0<<6;	/* Clock Edge, CKE=0, CKP=0 (SPI Mode 0,1) */
	SPI2CONSET = 1<<15;	/* start SPI 2 */
}

static void init_dma()
{
	/* open and configure the DMA channels
	     DMA 0 is for SPI -> buffer
	     DMA 1 is for buffer -> SPI */
	DmaChnOpen(DMA_CHANNEL0, DMA_CHN_PRI3, DMA_OPEN_AUTO);
	DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI3, DMA_OPEN_AUTO);

	/* DMA channels trigger on SPI RX/TX */
	DmaChnSetEventControl(DMA_CHANNEL0, DMA_EV_START_IRQ(_SPI2_RX_IRQ));
	DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ(_SPI2_TX_IRQ));

	/* transfer 8bits at a time */
	DmaChnSetTxfer(DMA_CHANNEL0, (void *)&SPI2BUF, (void *)rxBuf, 1, SPIBUFSIZE, 1);
	DmaChnSetTxfer(DMA_CHANNEL1, (void *)txBuf, (void *)&SPI2BUF, SPIBUFSIZE, 1, 1);

	/* start DMA 0 */
	DmaChnEnable(0);
	DmaChnEnable(1);
}

/* OC1 - OC3 is using Timer2 as clock source */
static inline void configure_pwm()
{
	OC1CON = 0x0000;	/* disable OCn */
	OC1R = 0;		/* set output high */
	OC1RS = 0;
	OC1CON = 0x0006;	/* PWM mode, fault pin disabled */

	OC2CON = 0x0000;
	OC2R = 0;
	OC2RS = 0;
	OC2CON = 0x0006;

	OC3CON = 0x0000;
	OC3R = 0;
	OC3RS = 0;
	OC3CON = 0x0006;

	T2CONSET = 0x0008;	/* Timer2 32 bit mode */
	PR2 = 0x9C3F;		/* set period, 1kHz */
	T2CONSET = 0x8000;	/* start timer */
	
	OC1CONSET = 0x8020;	/* enable OCn in 32 bit mode */
	OC2CONSET = 0x8020;
	OC3CONSET = 0x8020;
}

static inline void update_pwm_period(uint32_t val)
{
	PR2 = val;
}

static inline void update_pwm_duty(uint32_t *val)
{
	OC1RS = val[0];
	OC2RS = val[1];
	OC3RS = val[2];
}

static inline uint32_t read_inputs()
{
	uint32_t x, y;

	y = BIT_5 & ~LATB;	/* push LED state */
	LATBSET = BIT_5;	/* set LED pin high */
	TRISBSET = BIT_5;	/* set LED pin as input */
	LATBCLR = y;		/* pop LED state */

	x  = (HOME_X_IN ? 1 : 0) << 0;
	x |= (HOME_Y_IN ? 1 : 0) << 1;
	x |= (HOME_Z_IN ? 1 : 0) << 2;
	x |= (HOME_A_IN ? 1 : 0) << 3;
	x |= (STOP_IN   ? 1 : 0) << 4;

	TRISBCLR = BIT_5;	/* set LED pin as output */

	return x;
}

static inline void update_outputs(uint32_t x)
{
	if (x & (1 << 0))
		ENABLE_LO;	/* active low signal */
	else
		ENABLE_HI;
}

void reset_board()
{
	stepgen_reset();
	update_outputs(0);
}

int main(void)
{
	int i, spi_timeout;
	unsigned long counter;

	/* Disable JTAG port so we get our I/O pins back */
	DDPCONbits.JTAGEN = 0;
	/* Enable optimal performance */
	SYSTEMConfigPerformance(GetSystemClock());
	/* Use 1:1 CPU Core:Peripheral clocks */
	OSCSetPBDIV(OSC_PB_DIV_1);

	/* configure the core timer roll-over rate */
	OpenCoreTimer(CORE_TICK_RATE);

	/* set up the core timer interrupt */
	mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_6 | CT_INT_SUB_PRIOR_0));

	/* enable multi vector interrupts */
	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	INTEnableInterrupts();

	map_peripherals();
	init_io_ports();
	configure_pwm();
	init_spi();
	init_dma();

	/* wait until tx buffer is filled up */
	while (!SPI2STATbits.SPITBF);

	reset_board();
	spi_data_ready = 0;
	spi_timeout = 0;
	counter = 0;

	/* enable watchdog */
	WDTCONSET = 0x8000;

	/* main loop */
	while (1) {
		if (spi_data_ready) {
			spi_data_ready = 0;

			/* the first element received is a command string */
			switch (rxBuf[0]) {
			case 0x5453523E:	/* >RST */
				reset_board();
				break;
			case 0x314D433E:	/* >CM1 */
				stepgen_update_input((const void *)&rxBuf[1]);
				stepgen_get_position((void *)&txBuf[1]);
				break;
			case 0x324D433E:	/* >CM2 */
				update_outputs(rxBuf[1]);
				update_pwm_duty((uint32_t *)&rxBuf[2]);
				txBuf[1] = read_inputs();
				break;
			case 0x4746433E:	/* >CFG */
				stepgen_update_stepwidth(rxBuf[1]);
				update_pwm_period(rxBuf[2]);
				stepgen_reset();
				break;
			case 0x5453543E:	/* >TST */
				for (i=0; i<BUFSIZE; i++)
					txBuf[i] = rxBuf[i] ^ ~0;
				break;
			}
		}

		/* if rx buffer is half-full, update the integrity check.
		   There isn't enough time if we wait for complete transfer */
		if (DCH0INTbits.CHDHIF) {
			DCH0INTCLR = 1<<4;		/* clear flag */
			txBuf[0] = rxBuf[0] ^ ~0;
		}

		/* if rx buffer is full, data from spi bus is ready */
		if (DCH0INTbits.CHBCIF) {
			DCH0INTCLR = 1<<3;		/* clear flag */
			spi_data_ready = 1;
			spi_timeout = SPI_TIMEOUT;
		}

		/* reset the board if there is no SPI activity */
		if (spi_timeout)
			spi_timeout--;

		if (spi_timeout == 1) {				
			DCH0ECONSET=BIT_6;	/* abort DMA transfers */
			DCH1ECONSET=BIT_6;
		
			init_spi();
			init_dma();
			reset_board();

			/* wait until tx buffer is filled up */
			while (!SPI2STATbits.SPITBF);
		}

		/* blink onboard led */
		if (!(counter++ % (spi_timeout ? 0x10000 : 0x40000))) {
			LED_TOGGLE;
		}

		/* keep alive */
		WDTCONSET = 0x01;
	}
	return 0;
}

void __ISR(_CORE_TIMER_VECTOR, ipl6) CoreTimerHandler(void)
{
	/* update the period */
	UpdateCoreTimer(CORE_TICK_RATE);

	/* do repetitive tasks here */
	stepgen();

	/* clear the interrupt flag */
	mCTClearIntFlag();
}


