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

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#define SYS_FREQ		(40000000ul)    /* 40 MHz */
#define GetSystemClock()	(SYS_FREQ)
#define	GetPeripheralClock()	(GetSystemClock())
#define	GetInstructionClock()	(GetSystemClock())

#define SPICHAN			2

/*    PORT USAGE
 *
 *	Pin	Port	Dir	Signal
 *
 *	2	RA0	OUT	SPINDLE_ENABLE
 *	3	RA1	OUT	MIST_ENABLE
 *	9	RA2	OUT	STEP_Z
 *	10	RA3	OUT	DIR_Z
 *	12	RA4	OUT	STEP_X
 *	4	RB0	OUT	FLOOD_ENABLE
 *	5	RB1	OUT	DIR_A
 *	6	RB2	OUT	ENABLE
 *	7	RB3	OUT	STEP_A
 *	11	RB4	OUT	DIR_X
 *	14	RB5	OUT	Status LED
 *	21	RB10	OUT	DIR_Y
 *	22	RB11	OUT	MISO
 *	23	RB12	OUT	STEP_Y
 *
 *	15	RB6	IN	HOME_A
 *	16	RB7	IN	HOME_Z
 *	17	RB8	IN	HOME_Y
 *	18	RB9	IN	HOME_X
 *	24	RB13	IN	MOSI
 *	25	RB14	IN	CS
 *	26	RB15	IN	SCLK
 *
 */

#define LED_TOGGLE		(LATBINV = BIT_5)

#define STOP_IN			(PORTBbits.RB5)
#define HOME_A_IN		(PORTBbits.RB6)
#define HOME_X_IN		(PORTBbits.RB9)
#define HOME_Y_IN		(PORTBbits.RB8)
#define HOME_Z_IN		(PORTBbits.RB7)

#define ENABLE_LO		(LATBCLR = BIT_2)
#define ENABLE_HI		(LATBSET = BIT_2)
#define SPINDLE_EN_LO		(LATACLR = BIT_0)
#define SPINDLE_EN_HI		(LATASET = BIT_0)
#define MIST_EN_LO		(LATACLR = BIT_1)
#define MIST_EN_HI		(LATASET = BIT_1)
#define FLOOD_EN_LO		(LATBCLR = BIT_0)
#define FLOOD_EN_HI		(LATBSET = BIT_0)

#define STEP_X_LO		(LATACLR = BIT_4)
#define STEP_X_HI		(LATASET = BIT_4)
#define DIR_X_LO		(LATBCLR = BIT_4)
#define DIR_X_HI		(LATBSET = BIT_4)

#define STEP_Y_LO		(LATBCLR = BIT_12)
#define STEP_Y_HI		(LATBSET = BIT_12)
#define DIR_Y_LO		(LATBCLR = BIT_10)
#define DIR_Y_HI		(LATBSET = BIT_10)

#define STEP_Z_LO		(LATACLR = BIT_2)
#define STEP_Z_HI		(LATASET = BIT_2)
#define DIR_Z_LO		(LATACLR = BIT_3)
#define DIR_Z_HI		(LATASET = BIT_3)

#define STEP_A_LO		(LATBCLR = BIT_3)
#define STEP_A_HI		(LATBSET = BIT_3)
#define DIR_A_LO		(LATBCLR = BIT_1)
#define DIR_A_HI		(LATBSET = BIT_1)

#endif /* __HARDWARE_H__ */