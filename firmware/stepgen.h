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

#ifndef __STEPGEN_H__
#define __STEPGEN_H__

#define STEPBIT		22
#define STEP_MASK	(1L<<STEPBIT)
#define DIR_MASK	(1L<<31)

#define STEPWIDTH	1
#define MAXGEN		4

#define disable_int()								\
	do {									\
		asm volatile("di");						\
		asm volatile("ehb");						\
	} while (0)

#define enable_int()								\
	do {									\
		asm volatile("ei");						\
	} while (0)

typedef struct {
	int32_t velocity[MAXGEN];
} stepgen_input_struct;

void doXStep(void);
void doYStep(void);
void doZStep(void);
void doAStep(void);
    
void stepgen_reset(void);

int32_t stepgen_get_x_position(void);
int32_t stepgen_get_y_position(void);
int32_t stepgen_get_z_position(void);
int32_t stepgen_get_a_position(void);

void stepgen_update_x_velocity(int32_t velocity);
void stepgen_update_y_velocity(int32_t velocity);
void stepgen_update_z_velocity(int32_t velocity);
void stepgen_update_a_velocity(int32_t velocity);

void stepgen_update_stepwidth(int width);

#endif				/* __STEPGEN_H__ */
