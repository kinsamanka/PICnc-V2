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
#define MX 1
#define MY 2
#define MZ 4
#define MA 8

typedef struct {
    int dir;
    int step;
} step_fifo_t;

typedef struct {
    int32_t velocity; // this is from the host
    int32_t positionDesired;
    int32_t positionActual;
    int32_t stepWidth;
    int32_t dir;
    int32_t step;
} stepChannel_t;

#define STEP_BUFFER_SIZE 32
step_fifo_t step_buffer[STEP_BUFFER_SIZE];
int buffer_insert=0;
int buffer_remove=0;
int buffer_count=0;

#define STEPMASK (1<<22)

#define FORWARD 0
#define REVERSE 1
/***************************************/

stepChannel_t x_channel;
stepChannel_t y_channel;
stepChannel_t z_channel;
stepChannel_t a_channel;

uint32_t StepWidth = 1;

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
    if(width > 0)
        StepWidth = width;
    else
        StepWidth = 1;
	enable_int();
}

void stepgen_reset(void)
{
	int i;
    stepChannel_t *chn;

	disable_int();
    buffer_insert = 0;
    buffer_remove = 0;
    buffer_count  = 0;
	x_channel.positionActual = 0;
	x_channel.positionDesired = 0;
	x_channel.velocity = 0;
    x_channel.step = 0;
    x_channel.dir = 0;
    
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

void stepgen_create(void)
{
    static step_fifo_t *fifo = step_buffer;
    int c;

    do
    {
        disable_int();
        c = buffer_count;
        enable_int();
        if(c < STEP_BUFFER_SIZE)
        {
            disable_int();
            buffer_count ++;
            fifo = (fifo == &step_buffer[STEP_BUFFER_SIZE-1])?fifo ++: step_buffer;
            enable_int();

            fifo->step = 0;
            fifo->dir = 0;

            if(x_channel.step) // previous step is One
            {
                if(--x_channel.stepWidth==0) // transitioned to zero
                {
                    x_channel.step = 0;
                    if(x_channel.velocity<0)
                        fifo->dir |= MX;
                }
            }
            else // previous step is Zero
            {
                if( (x_channel.positionDesired ^ x_channel.positionActual ) & STEPMASK )
                {
                    x_channel.step = 1;
                    fifo->step |= MX;
                    x_channel.positionActual = x_channel.positionDesired;
                    x_channel.stepWidth = StepWidth;
                }
            }
            x_channel.positionDesired += x_channel.velocity;

            if(y_channel.step) // previous step is One
            {
                if(--y_channel.stepWidth==0) // transitioned to zero
                {
                    y_channel.step = 0;
                    if(y_channel.velocity<0)
                        fifo->dir |= MY;
                }
            }
            else // previous step is Zero
            {
                if( (y_channel.positionDesired ^ y_channel.positionActual ) & STEPMASK )
                {
                    y_channel.step = 1;
                    fifo->step |= MY;
                    y_channel.positionActual = y_channel.positionDesired;
                    y_channel.stepWidth = StepWidth;
                }
            }
            y_channel.positionDesired += y_channel.velocity;

            if(z_channel.step) // previous step is One
            {
                if(--z_channel.stepWidth==0) // transitioned to zero
                {
                    z_channel.step = 0;
                    if(z_channel.velocity<0)
                        fifo->dir |= MY;
                }
            }
            else // previous step is Zero
            {
                if( (z_channel.positionDesired ^ z_channel.positionActual ) & STEPMASK )
                {
                    z_channel.step = 1;
                    fifo->step |= MY;
                    z_channel.positionActual = z_channel.positionDesired;
                    z_channel.stepWidth = StepWidth;
                }
            }
            z_channel.positionDesired += z_channel.velocity;

            if(a_channel.step) // previous step is One
            {
                if(--a_channel.stepWidth==0) // transitioned to zero
                {
                    a_channel.step = 0;
                    if(a_channel.velocity<0)
                        fifo->dir |= MY;
                }
            }
            else // previous step is Zero
            {
                if( (a_channel.positionDesired ^ a_channel.positionActual ) & STEPMASK )
                {
                    a_channel.step = 1;
                    fifo->step |= MY;
                    a_channel.positionActual = a_channel.positionDesired;
                    a_channel.stepWidth = StepWidth;
                }
            }
            a_channel.positionDesired += a_channel.velocity;

        }
        else
            break;
    } while(1);
}


inline void stepgen(void)
{
    static step_fifo_t *fifo = step_buffer;
    
    if(buffer_count)
    {
        LED3_LO;
        fifo = (fifo == &step_buffer[STEP_BUFFER_SIZE-1])?fifo ++: step_buffer;
        buffer_count --;

        if(fifo->dir  & MX)  DIR_X_HI; else  DIR_X_LO;
        if(fifo->step & MX) STEP_X_HI; else STEP_X_LO;
        if(fifo->dir  & MY)  DIR_Y_HI; else  DIR_Y_LO;
        if(fifo->step & MY) STEP_Y_HI; else STEP_Y_LO;
        if(fifo->dir  & MZ)  DIR_Z_HI; else  DIR_Z_LO;
        if(fifo->step & MZ) STEP_Z_HI; else STEP_Z_LO;
        if(fifo->dir  & MA)  DIR_A_HI; else  DIR_A_LO;
        if(fifo->step & MA) STEP_A_HI; else STEP_A_LO;
    }
    else
    {
        LED3_HI;
    }
}
