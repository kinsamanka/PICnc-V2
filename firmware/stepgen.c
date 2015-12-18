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
} stepChannel_t;

#define STEPSIZE 50000UL

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
    
	y_channel.positionActual = 0;
	y_channel.positionDesired = 0;
	y_channel.velocity = 0;
    
	z_channel.positionActual = 0;
	z_channel.positionDesired = 0;
	z_channel.velocity = 0;
    
	a_channel.positionActual = 0;
	a_channel.positionDesired = 0;
	a_channel.velocity = 0;
	enable_int();
}

void doXStep(void)
{
    int distance;
    
    if(x_channel.velocity & 1<<31)
        DIR_X_HI;
    else
        DIR_X_LO;

    distance = abs(x_channel.positionDesired - x_channel.positionActual);

    if (distance > STEPSIZE)
    {
        STEP_X_HI;
        x_channel.positionActual = x_channel.positionDesired;
    }    
    /* update positionDesired counter */
    x_channel.positionDesired += x_channel.velocity;
}

void doYStep(void)
{
    int distance;
    
    if(y_channel.velocity & 1<<31)
        DIR_Y_HI;
    else
        DIR_Y_LO;
    distance = abs(y_channel.positionDesired - y_channel.positionActual);
    if (distance > STEPSIZE)
    {
        STEP_Y_HI;
        y_channel.positionActual = y_channel.positionDesired;
    }    
    /* update positionDesired counter */
    y_channel.positionDesired += y_channel.velocity;
}

void doZStep(void)
{
    int distance;
    
    if(z_channel.velocity & 1<<31)
        DIR_Z_HI;
    else
        DIR_Z_LO;
    distance = abs(z_channel.positionDesired - z_channel.positionActual);
    if (distance > STEPSIZE)
    {
        STEP_Z_HI;
        z_channel.positionActual = z_channel.positionDesired;
    }    
    /* update positionDesired counter */
    z_channel.positionDesired += z_channel.velocity;
}

void doAStep(void)
{
    int distance;
    
    if(a_channel.velocity & 1<<31)
        DIR_A_HI;
    else
        DIR_A_LO;
    distance = abs(a_channel.positionDesired - a_channel.positionActual);
    if (distance > STEPSIZE)
    {
        STEP_A_HI;
        a_channel.positionActual = a_channel.positionDesired;
    }    
    /* update positionDesired counter */
    a_channel.positionDesired += a_channel.velocity;
}

