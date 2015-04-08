/*    Copyright (C) 2014 GP Orcullo
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
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

#ifndef PICNC_H
#define PICNC_H

enum pin_input_names {
	HOME_X,
	HOME_Y,
	HOME_Z,
	HOME_A,
	STOP
};

enum pin_output_names {
	SPINDLE,
	MIST,
	FLOOD,
	OUTPUT
};

#define BCM2835_SPICLKDIV	32		/* ~8 Mhz */
#define NUMAXES			4		/* X Y Z A*/
#define PWMCHANS		3
#define NUMOUTPUTS		4
#define NUMINPUTS		5

#define REQ_TIMEOUT		10000ul

#define SPIBUFSIZE		20		/* SPI buffer size */
#define BUFSIZE			(SPIBUFSIZE/4)

#define STEPBIT			23		/* bit location in DDS accum */
#define STEP_MASK		(1<<STEPBIT)

#define BASEFREQ		80000ul		/* Base freq of the PIC stepgen in Hz */
#define SYS_FREQ		(40000000ul)    /* 40 MHz */

#define PERIODFP 		((double)1.0 / (double)(BASEFREQ))
#define VELSCALE		((double)STEP_MASK * PERIODFP)
#define ACCELSCALE		(VELSCALE * PERIODFP)

#define get_position(a)		(rxBuf[1 + (a)])
#define update_velocity(a, b)	(txBuf[1 + (a)] = (b))

#define PAGE_SIZE		(4*1024)
#define BLOCK_SIZE		(4*1024)

/* Odroid C1 defines */

#define ODROID_FIFO_SIZE	16

#define ODROID_CBUS_PHY_BASE	0xC1100000
#define ODROID_GPIO_BASE	(ODROID_CBUS_PHY_BASE + 0x8000)
#define ODROID_GCLK_MPEG0	(ODROID_CBUS_PHY_BASE + 0x4000)

#define ODROID_PPMUX_3		*(gpio + 0x2F)
#define ODROID_PPMUX_4		*(gpio + 0x30)
#define ODROID_PPMUX_5		*(gpio + 0x31)
#define ODROID_PPMUX_6		*(gpio + 0x32)
#define ODROID_PPMUX_7		*(gpio + 0x33)
#define ODROID_PPMUX_8		*(gpio + 0x34)
#define ODROID_PPMUX_9		*(gpio + 0x35)

#define ODROID_GPIOX_OEN	*(gpio + 0x0C)
#define ODROID_GPIOX_PUEN	*(gpio + 0x4C)
#define ODROID_GPIOX_PUPD	*(gpio + 0x3E)

#define ODROID_SPI_RX		*(gpio + 0x360)
#define ODROID_SPI_TX		*(gpio + 0x361)
#define ODROID_SPI_CON		*(gpio + 0x362)
#define ODROID_SPI_STAT		*(gpio + 0x365)

#define ODROID_SPI_CLKGATE	*(spi + 0x50)

/* Broadcom defines */

#define BCM2835_PERI_BASE	0x20000000
#define BCM2835_GPIO_BASE	(BCM2835_PERI_BASE + 0x200000) /* GPIO controller */
#define BCM2835_SPI_BASE	(BCM2835_PERI_BASE + 0x204000) /* SPI controller */

#define BCM2835_GPFSEL0		*(gpio)
#define BCM2835_GPFSEL1		*(gpio + 1)
#define BCM2835_GPFSEL2		*(gpio + 2)
#define BCM2835_GPFSEL3		*(gpio + 3)
#define BCM2835_GPFSEL4		*(gpio + 4)
#define BCM2835_GPFSEL5		*(gpio + 5)
#define BCM2835_GPSET0		*(gpio + 7)
#define BCM2835_GPSET1		*(gpio + 8)
#define BCM2835_GPCLR0		*(gpio + 10)
#define BCM2835_GPCLR1		*(gpio + 11)
#define BCM2835_GPLEV0		*(gpio + 13)
#define BCM2835_GPLEV1		*(gpio + 14)

#define BCM2835_SPICS 		*(spi + 0)
#define BCM2835_SPIFIFO     	*(spi + 1)
#define BCM2835_SPICLK 		*(spi + 2)

#define SPI_CS_LEN_LONG		0x02000000
#define SPI_CS_DMA_LEN		0x01000000
#define SPI_CS_CSPOL2		0x00800000
#define SPI_CS_CSPOL1		0x00400000
#define SPI_CS_CSPOL0		0x00200000
#define SPI_CS_RXF		0x00100000
#define SPI_CS_RXR		0x00080000
#define SPI_CS_TXD		0x00040000
#define SPI_CS_RXD		0x00020000
#define SPI_CS_DONE		0x00010000
#define SPI_CS_LEN		0x00002000
#define SPI_CS_REN		0x00001000
#define SPI_CS_ADCS		0x00000800
#define SPI_CS_INTR		0x00000400
#define SPI_CS_INTD		0x00000200
#define SPI_CS_DMAEN		0x00000100
#define SPI_CS_TA		0x00000080
#define SPI_CS_CSPOL		0x00000040
#define SPI_CS_CLEAR_RX		0x00000020
#define SPI_CS_CLEAR_TX		0x00000010
#define SPI_CS_CPOL		0x00000008
#define SPI_CS_CPHA		0x00000004
#define SPI_CS_CS_10		0x00000002
#define SPI_CS_CS_01		0x00000001

#endif
