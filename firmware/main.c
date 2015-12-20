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
#include <stdbool.h>
#include <plib.h>
#include <xc.h>
#include "hardware_v3.h"
#include "stepgen.h"

#pragma config POSCMOD = HS         /* Primary Oscillator HS */
#pragma config FNOSC = PRIPLL          /* Primary */
#pragma config FPLLODIV = DIV_1		/* PLL configured for 80MHz clock */
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPBDIV = DIV_1		/* Peripheral Clock Divisor */
#pragma config IESO = OFF            /* Internal/External Switch Over disabled */
#pragma config FSOSCEN = OFF		/* Secondary Oscillator disabled */
#pragma config CP = OFF             /* Code Protect Disabled */
#pragma config FWDTEN = OFF          /* Watchdog Timer Enable */
#pragma config WDTPS = PS4096		/* Watchdog Timer Postscaler */

#define BASEFREQ			200000
#define CORE_TICK_RATE	    (SYS_FREQ/2/BASEFREQ)
#define SPIBUFSIZE			20
#define BUFSIZE				(SPIBUFSIZE/4)
#define SPI_TIMEOUT			10000L

static volatile uint32_t rxBuf[BUFSIZE];
static volatile uint32_t txBuf[BUFSIZE];

static bool spi_timeout;

#define CMD_RESET 0x5453523E
#define CMD_CM1   0x314D433E
#define CMD_CM2   0x324D433E
#define CMD_CFG   0x4746433E
#define CMD_TEST  0x5453543E

static void map_peripherals()
{
}

static void init_io_ports()
{
	/* disable all analog pins */
    
    /* configure SPI pins */
    TRISGbits.TRISG9 = 1; // input on Chip Select
    TRISGbits.TRISG6 = 1; // input on SPI clock
    TRISGbits.TRISG7 = 1; // input on MOSI
    TRISGbits.TRISG8  = 0; // output on MISO
    
	/* configure inputs */
    SET_STOP_INPUT;
    SET_TOUCHOFF_INPUT;
    SET_X_MAX_INPUT;
    SET_X_MIN_INPUT;
    SET_Y_MAX_INPUT;
    SET_Y_MIN_INPUT;
    SET_Z_MAX_INPUT;
    SET_Z_MIN_INPUT;

	/* configure_outputs */
    SET_DRV_OUTPUT;
    SET_ENABLE_OUTPUT;
    SET_SPINDLE_EN_OUTPUT;
    SET_COOLANT_OUTPUT;
    SET_MIST_EN_OUTPUT;
    SET_FLOOD_EN_OUTPUT;
    SET_E1_ENABLE_OUTPUT;
    SET_STEP_X_OUTPUT;
    SET_DIR_X_OUTPUT;
    SET_STEP_Y_OUTPUT;
    SET_DIR_Y_OUTPUT;
    SET_STEP_Z_OUTPUT;
    SET_DIR_Z_OUTPUT;
    SET_STEP_A_OUTPUT;
    SET_DIR_A_OUTPUT;
    SET_LED1_OUTPUT;
    SET_LED2_OUTPUT;
    SET_LED3_OUTPUT;
    SET_LED4_OUTPUT;
    
    /* Turn on the Drivers (level shifters) */
    DRIVER_LO;
}

static void init_spi()
{
	int i;

	SPI2CON = 0;		/* stop SPI 2, set Slave mode, 8 bits, std buffer */
	i = SPI2BUF;		/* clear rcv buffer */

    SPI2STATCLR = BIT_6;    /* clear the overflow */
	SPI2CONbits.CKE  = 0;
    SPI2CONbits.CKP  = 0;
    SPI2CONbits.SSEN = 0;
	SPI2CONSET = BIT_15;	/* start SPI 2 */
}

static void init_dma()
{
	/* open and configure the DMA channels
	     DMA 0 is for SPI -> buffer
	     DMA 1 is for buffer -> SPI */
	DmaChnOpen(DMA_CHANNEL0, DMA_CHN_PRI3, DMA_OPEN_AUTO);
	DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI2, DMA_OPEN_AUTO);

	/* DMA channels trigger on SPI RX/TX */
	DmaChnSetEventControl(DMA_CHANNEL0, DMA_EV_SRC_HALF |DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_SPI2_RX_IRQ));
	DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ(_SPI2_TX_IRQ));

	/* transfer 8bits at a time */
	DmaChnSetTxfer(DMA_CHANNEL0, (void *)&SPI2BUF, (void*)rxBuf,      1, SPIBUFSIZE, 1);
	DmaChnSetTxfer(DMA_CHANNEL1, (void *)txBuf,    (void*)&SPI2BUF, SPIBUFSIZE, 1, 1);
    
  	DmaChnSetEvEnableFlags(DMA_CHANNEL0, DMA_EV_BLOCK_DONE);	// enable the transfer done interrupt, when all buffer transferred
	DmaChnSetIntPriority(DMA_CHANNEL0, INT_PRIORITY_LEVEL_5, INT_SUB_PRIORITY_LEVEL_3);		// set INT controller priorities
    DmaChnIntEnable(DMA_CHANNEL0);
    
   	DmaChnStartTxfer(DMA_CHANNEL1, DMA_WAIT_NOT, 0);	// force the DMA transfer: the SPI TBE flag it's already been active
	DmaChnEnable(DMA_CHANNEL0);
}

/* OC1 - OC3 is using Timer2 as clock source */
static inline void configure_pwm()
{
	OC2CON = 0x0000;
	OC2R = 0;
	OC2RS = 0;
	OC2CON = 0x0006;

	OC3CON = 0x0000;
	OC3R = 0;
	OC3RS = 0;
	OC3CON = 0x0006;

	OC4CON = 0x0000;
	OC4R = 0;
	OC4RS = 0;
	OC4CON = 0x0006;

	OC5CON = 0x0000;
	OC5R = 0;
	OC5RS = 0;
	OC5CON = 0x0006;

	T2CONSET = 0x0008;	/* Timer2 32 bit mode */
	PR2 = 0x9C3F;		/* set period, 1kHz */
	T2CONSET = 0x8000;	/* start timer */
	
	OC2CONSET = 0x8020;
	OC3CONSET = 0x8020;
	OC4CONSET = 0x8020;
	OC5CONSET = 0x8020;
}

static inline void update_pwm_period(uint32_t val)
{
	PR2 = val;
}

static inline void update_pwm1_duty(uint32_t val)
{
	OC2RS = val;
}
static inline void update_pwm2_duty(uint32_t val)
{
	OC3RS = val;
}
static inline void update_pwm3_duty(uint32_t val)
{
	OC4RS = val;
}
static inline void update_pwm4_duty(uint32_t val)
{
	OC5RS = val;
}

static inline uint32_t read_inputs()
{
	uint32_t x = 0;

	x |= HOME_X_IN   ? BIT_0 : 0;
	x |= HOME_Y_IN   ? BIT_1 : 0;
	x |= HOME_Z_IN   ? BIT_2 : 0;
	x |= HOME_A_IN   ? BIT_3 : 0;
	x |= STOP_IN     ? BIT_4 : 0;
    x |= TOUCHOFF_IN ? BIT_5 : 0;
    x |= Z_MAX_IN    ? BIT_6 : 0;
    x |= Y_MAX_IN    ? BIT_7 : 0;

	return x;
}

static inline void update_outputs(uint32_t x)
{
	if (x & (1 << 0))
		DRIVER_ENABLE_HI;	/* active low signal */
	else
		DRIVER_ENABLE_LO;
}

void reset_board()
{
	stepgen_reset();
	update_outputs(0);
    DRIVER_ENABLE_LO; // disable the stepper driver until driven by Machine kit
}

int main(void)
{
	int i;
	unsigned long counter;

	/* enable JTAG port so we get can program from the PI */
	DDPCONbits.JTAGEN = 1;
	/* Enable optimal performance */
	SYSTEMConfigPerformance(GetSystemClock());
	/* Use 1:1 CPU Core:Peripheral clocks */
	OSCSetPBDIV(OSC_PB_DIV_1);

	/* configure the core timer roll-over rate */
	OpenCoreTimer(CORE_TICK_RATE);

	/* set up the core timer interrupt */
	mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_6 | CT_INT_SUB_PRIOR_0));

	map_peripherals();
	init_io_ports();
	configure_pwm(); // not using PWM on this board
    
	init_spi();
	init_dma();
	reset_board();
	spi_timeout = SPI_TIMEOUT;
    
	counter = 0;

	/* enable watchdog */
	WDTCONSET = 0x8000;

	/* enable multi vector interrupts */
	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
	INTEnableInterrupts();
    
	/* main loop */
	while (1)
    {
        /* reset the board if there is no SPI activity */
#if 0
        if(!(--spi_timeout))
        {
            DmaChnAbortTxfer(DMA_CHANNEL0);
            DmaChnAbortTxfer(DMA_CHANNEL1);
		
            init_spi();
            init_dma();
            reset_board();
		}
#endif
		/* blink on board led */
		if (!(counter++ % (spi_timeout ? 0x10000 : 0x40000)))
        {
			LED1_TOGGLE;
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
    HEARTBEAT_ENABLE_HI;
	/* clear the interrupt flag */
	mCTClearIntFlag();

	/* do repetitive tasks here */
    stepgen();
    
    HEARTBEAT_ENABLE_LO;
}

void __ISR(_DMA0_VECTOR,ipl5) DMA0Handler(void)
{
	int	evFlags;				// event flags when getting the interrupt
    int i;
 
	mDmaChnClrIntFlag(0);		// acknowledge the INT controller, we're servicing int
	evFlags=DmaChnGetEvFlags(0);	// get the event flags

    if(evFlags & DMA_EV_SRC_HALF)
    {
        DmaChnClrEvFlags(0,DMA_EV_SRC_HALF);
        txBuf[0] = ~rxBuf[0];
    }
    if(evFlags&DMA_EV_BLOCK_DONE)
    { // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
     	DmaChnClrEvFlags(0, DMA_EV_BLOCK_DONE);
        txBuf[0] = ~rxBuf[0];
        spi_timeout = SPI_TIMEOUT;
        /* the first element received is a command string */
        switch (rxBuf[0])
        {
            case CMD_RESET:
                reset_board();
                break;
            case CMD_CM1:
                stepgen_update_x_velocity( rxBuf[1] );
                stepgen_update_y_velocity( rxBuf[2] );
                stepgen_update_z_velocity( rxBuf[3] );
                stepgen_update_a_velocity( rxBuf[4] );
                txBuf[1] = stepgen_get_x_position(  );
                txBuf[2] = stepgen_get_y_position(  );
                txBuf[3] = stepgen_get_z_position(  );
                txBuf[4] = stepgen_get_a_position(  );
                break;
            case CMD_CM2:
                update_outputs(   rxBuf[1] );
                update_pwm1_duty( rxBuf[2] );
                update_pwm2_duty( rxBuf[3] );
                update_pwm3_duty( rxBuf[4] );
                txBuf[1] = read_inputs();
                break;
            case CMD_CFG:
                stepgen_update_stepwidth(rxBuf[1] );
                update_pwm_period(       rxBuf[2] );
                stepgen_reset();
                break;
            case CMD_TEST:
                for (i=1; i<sizeof(rxBuf); i++)
                    txBuf[i] = ~rxBuf[i];
                break;
            default:
                break;
        }
    }
}
