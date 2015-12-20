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

#include "hardware_v3.h"

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

typedef struct {
    int32_t velocity; // this is from the host
    int32_t positionDesired;
    int32_t positionActual;
    int32_t distance;
} stepChannel_t;

#define STEPSIZE (1<<16)

#define FORWARD 0
#define REVERSE 1
/***************************************/

stepChannel_t x_channel;
stepChannel_t y_channel;
stepChannel_t z_channel;
stepChannel_t a_channel;

int32_t stepgen_get_x_position(void)
{
    return x_channel.positionDesired;
}
int32_t stepgen_get_y_position(void)
{
    return y_channel.positionDesired;
}
int32_t stepgen_get_z_position(void)
{
    return z_channel.positionDesired;
}
int32_t stepgen_get_a_position(void)
{
    return a_channel.positionDesired;
}

void stepgen_update_x_velocity(int32_t velocity)
{
    x_channel.velocity = velocity;
}
void stepgen_update_y_velocity(int32_t velocity)
{
    y_channel.velocity = velocity;
}
void stepgen_update_z_velocity(int32_t velocity)
{
    z_channel.velocity = velocity;
}
void stepgen_update_a_velocity(int32_t velocity)
{
    a_channel.velocity = velocity;
}

void stepgen_update_stepwidth(int width)
{
	disable_int();
	enable_int();
}

void stepgen_reset(void)
{
	int i;
    stepChannel_t *chn;

	disable_int();
	x_channel.positionActual = 0;
	x_channel.positionDesired = 0;
	x_channel.velocity = 0;
    x_channel.distance = STEPSIZE;
    
	y_channel.positionActual = 0;
	y_channel.positionDesired = 0;
	y_channel.velocity = 0;
    y_channel.distance = STEPSIZE;
    
	z_channel.positionActual = 0;
	z_channel.positionDesired = 0;
	z_channel.velocity = 0;
    z_channel.distance = STEPSIZE;
    
	a_channel.positionActual = 0;
	a_channel.positionDesired = 0;
	a_channel.velocity = 0;
    a_channel.distance = STEPSIZE;
    
	enable_int();
}

#define stepMacro(data,dir_forward,dir_reverse,high,low) do{\
        if(data.velocity < 0)\
        {\
            dir_reverse;\
            if (STEPSIZE < (data.positionActual - data.positionDesired))\
            {\
                high;\
                data.positionActual -= STEPSIZE;\
            }\
        }\
        else\
        {\
            dir_forward;\
            if (STEPSIZE < (data.positionDesired - data.positionActual))\
            {\
                high;\
                data.positionActual += STEPSIZE;\
            }\
        }\
        data.positionDesired += data.velocity;\
    } while(0)


inline void stepgen(void)
{
    stepMacro(x_channel,DIR_X_HI,DIR_X_LO,STEP_X_HI,STEP_X_LO);
    stepMacro(y_channel,DIR_Y_HI,DIR_Y_LO,STEP_Y_HI,STEP_Y_LO);
    stepMacro(z_channel,DIR_Z_HI,DIR_Z_LO,STEP_Z_HI,STEP_Z_LO);
    stepMacro(a_channel,DIR_A_HI,DIR_A_LO,STEP_A_HI,STEP_A_LO);

    STEP_X_LO; // stop the previous step...
    STEP_Y_LO; // stop the previous step...
    STEP_Z_LO; // stop the previous step...
    STEP_A_LO; // stop the previous step...
}