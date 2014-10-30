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

#include <stdint.h>
#include <p32xxxx.h>
#include <plib.h>

#include <string.h>

#include "hardware.h"
#include "stepgen.h"

/*
  Timing diagram:

  STEPWIDTH   |<---->|
	       ______           ______
  STEP	     _/      \_________/      \__
  	     ________                  __
  DIR	             \________________/

  Direction signal changes on the falling edge of the step pulse.

*/

static void step_hi(int);
static void step_lo(int);
static void dir_hi(int);
static void dir_lo(int);

static volatile int32_t position[MAXGEN] = { 0 };

static volatile stepgen_input_struct stepgen_input = { {0} };

static int32_t oldpos[MAXGEN] = { 0 },
        oldvel[MAXGEN] = { 0 };

static int do_step_hi[MAXGEN] = { 1 },
	dirchange[MAXGEN] = { 0 },
	stepwdth[MAXGEN] = { 0 },
	step_width = STEPWIDTH;

void stepgen_get_position(void *buf)
{
	disable_int();
	memcpy(buf, (const void *)position, sizeof(position));
	enable_int();
}

void stepgen_update_input(const void *buf)
{
	disable_int();
	memcpy((void *)&stepgen_input, buf, sizeof(stepgen_input));
	enable_int();
}

void stepgen_update_stepwidth(int width)
{
	step_width = width;
}

void stepgen_reset(void)
{
	int i;

	disable_int();

	for (i = 0; i < MAXGEN; i++) {
		position[i] = 0;
		oldpos[i] = 0;
		oldvel[i] = 0;

		stepgen_input.velocity[i] = 0;
		do_step_hi[i] = 1;
	}

	enable_int();

	for (i = 0; i < MAXGEN; i++) {
		step_lo(i);
		dir_lo(i);
	}
}


void stepgen(void)
{
	uint32_t stepready;
	int i;

	for (i = 0; i < MAXGEN; i++) {

		/* check if a step pulse can be generated */
		stepready = (position[i] ^ oldpos[i]) & HALFSTEP_MASK;

		/* generate a step pulse */
		if (stepready) {
			oldpos[i] = position[i];
			stepwdth[i] =  step_width + 1;
			do_step_hi[i] = 0;
		}

		if (stepwdth[i]) {
			if (--stepwdth[i]) {
				step_hi(i);
			} else {
				do_step_hi[i] = 1;
				step_lo(i);
			}
		}

		/* check for direction change */
		if (!dirchange[i]) {
			if ((stepgen_input.velocity[i] ^ oldvel[i]) & DIR_MASK) {
				dirchange[i] = 1;
				oldvel[i] = stepgen_input.velocity[i];
			}
		}

		/* generate direction pulse after step hi-lo transition */
		if (do_step_hi[i] && dirchange[i]) {
			dirchange[i] = 0;
			if (oldvel[i] >= 0)
				dir_lo(i);
			if (oldvel[i] < 0)
				dir_hi(i);
		}

		/* update position counter */
		position[i] += stepgen_input.velocity[i];
	}
}

__inline__ void step_hi(int i)
{
	if (i == 0)
		STEP_X_HI;
	if (i == 1)
		STEP_Y_HI;
	if (i == 2)
		STEP_Z_HI;
	if (i == 3)
		STEP_A_HI;
}

__inline__ void step_lo(int i)
{
	if (i == 0)
		STEP_X_LO;
	if (i == 1)
		STEP_Y_LO;
	if (i == 2)
		STEP_Z_LO;
	if (i == 3)
		STEP_A_LO;
}

__inline__ void dir_hi(int i)
{
	if (i == 0)
		DIR_X_HI;
	if (i == 1)
		DIR_Y_HI;
	if (i == 2)
		DIR_Z_HI;
	if (i == 3)
		DIR_A_HI;
}

__inline__ void dir_lo(int i)
{
	if (i == 0)
		DIR_X_LO;
	if (i == 1)
		DIR_Y_LO;
	if (i == 2)
		DIR_Z_LO;
	if (i == 3)
		DIR_A_LO;
}
