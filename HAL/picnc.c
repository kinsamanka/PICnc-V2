/*    Copyright (C) 2014 GP Orcullo
 *
 *    Portions of this code is based on stepgen.c by John Kasunich
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

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "picnc.h"

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

#define MODNAME "picnc"
#define PREFIX "picnc"

MODULE_AUTHOR("GP Orcullo");
MODULE_DESCRIPTION("Driver for PICnc V2 boards");
MODULE_LICENSE("GPL v2");

static int stepwidth = 1;
RTAPI_MP_INT(stepwidth, "Step width in 1/BASEFREQ");

static long pwmfreq = 1000;
RTAPI_MP_LONG(pwmfreq, "PWM frequency in Hz");

typedef struct {
	hal_float_t *position_cmd[NUMAXES],
	            *position_fb[NUMAXES],
		    *pwm_duty[PWMCHANS];
	hal_bit_t   *pin_output[NUMOUTPUTS],
	            *pin_input[NUMINPUTS],
	            *ready,
		    *spi_fault,
		    pwm_enable[PWMCHANS];
	hal_float_t scale[NUMAXES],
	            maxaccel[NUMAXES],
		    pwm_scale[PWMCHANS];
} data_t;

static data_t *data;

static int comp_id;
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

static platform_t platform;
volatile unsigned *mem1, *mem2;

volatile int32_t txBuf[BUFSIZE], rxBuf[BUFSIZE];
static u32 pwm_period = 0;

static double dt = 0,				/* update_freq period in seconds */
	recip_dt = 0,				/* reciprocal of period, avoids divides */
	scale_inv[NUMAXES] = { 1.0 },		/* inverse of scale */
	old_vel[NUMAXES] = { 0 },
	old_pos[NUMAXES] = { 0 },
	old_scale[NUMAXES] = { 0 },
	max_vel;
static long old_dtns = 0;			/* update_freq funct period in nsec */
static s32 accum_diff = 0,
	old_count[NUMAXES] = { 0 };
static s64 accum[NUMAXES] = { 0 };		/* 64 bit DDS accumulator */

static void read_spi(void *arg, long period);
static void write_spi(void *arg, long period);
static void update(void *arg, long period);
static void update_outputs(data_t *dat);
static void update_inputs(data_t *dat);
static void (*read_buf)();
static void (*write_buf)();
static void rpi_read_buf();
static void rpi_write_buf();
static void c1_read_buf();
static void c1_write_buf();
static int map_gpio();
static void (*setup_gpio)();
static void (*restore_gpio)();
static void rpi_setup_gpio();
static void rpi_restore_gpio();
static void c1_setup_gpio();
static void c1_restore_gpio();

platform_t check_platform(void)
{
	FILE *fp;
	char buf[2048];
	size_t fsize;

	fp = fopen("/proc/cpuinfo", "r");
	fsize = fread(buf, 1, sizeof(buf), fp);
	fclose(fp);

	if (fsize == 0 || fsize == sizeof(buf))
		return 0;

	/* NUL terminate the buffer */
	buf[fsize] = '\0';

	if (NULL != strstr(buf, "BCM2708"))
		return RPI;
	else if (NULL != strstr(buf, "BCM2709"))
		return RPI_2;
	else if (NULL != strstr(buf, "ODROIDC"))
		return ODROID_C1;
	else
		return UNSUPPORTED;
}

int rtapi_app_main(void)
{
	char name[HAL_NAME_LEN + 1];
	int n, retval;

	platform = check_platform();

	switch (platform) {
	case RPI:
	case RPI_2:
		read_buf = rpi_read_buf;
		write_buf = rpi_write_buf;
		setup_gpio = rpi_setup_gpio;
		restore_gpio = rpi_restore_gpio;
		break;
	case ODROID_C1:
		read_buf = c1_read_buf;
		write_buf = c1_write_buf;
		setup_gpio = c1_setup_gpio;
		restore_gpio = c1_restore_gpio;
		break;
	default:
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: This driver is not for this platform.\n",
		        modname);
		return -1;
	}

	/* initialise driver */
	comp_id = hal_init(modname);
	if (comp_id < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n",
		        modname);
		return -1;
	}

	/* allocate shared memory */
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n",
		        modname);
		hal_exit(comp_id);
		return -1;
	}

	/* configure board */
	retval = map_gpio();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: cannot map GPIO memory\n", modname);
		return retval;
	}

	setup_gpio();

	pwm_period = (SYS_FREQ/pwmfreq) - 1;	/* PeripheralClock/pwmfreq - 1 */

	txBuf[0] = CMD_CFG;		/* this is config data (>CFG) */
	txBuf[1] = stepwidth;
	txBuf[2] = pwm_period;
	write_buf();			/* send config data */

	max_vel = BASEFREQ/(stepwidth);	/* calculate velocity limit */

	/* export pins and parameters */
	for (n=0; n<NUMAXES; n++) {
		retval = hal_pin_float_newf(HAL_IN, &(data->position_cmd[n]),
		        comp_id, "%s.axis.%01d.position-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->position_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->position_fb[n]),
		        comp_id, "%s.axis.%01d.position-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->position_fb[n]) = 0.0;

		retval = hal_param_float_newf(HAL_RW, &(data->scale[n]),
		        comp_id, "%s.axis.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->scale[n] = 1.0;

		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
		        comp_id, "%s.axis.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;

		retval = hal_pin_bit_newf(HAL_OUT, &(data->pin_input[n]),
			comp_id, "%s.axis.%01d.home", prefix, n);
		if (retval < 0) goto error;
		*(data->pin_input[n]) = 0;
	}

	retval = hal_pin_bit_newf(HAL_OUT, &(data->pin_input[STOP]),
		comp_id, "%s.in.stop", prefix);
	if (retval < 0) goto error;
	*(data->pin_input[STOP]) = 0;

	retval = hal_pin_bit_newf(HAL_IN, &(data->pin_output[OUTPUT]),
		comp_id, "%s.out.enable", prefix);
	if (retval < 0) goto error;
	*(data->pin_output[OUTPUT]) = 0;

	retval = hal_pin_bit_newf(HAL_IN, &(data->pin_output[SPINDLE]),
		comp_id, "%s.spindle.enable", prefix);
	if (retval < 0) goto error;
	*(data->pin_output[SPINDLE]) = 0;

	retval = hal_param_bit_newf(HAL_RW, &(data->pwm_enable[SPINDLE]),
		comp_id, "%s.spindle.pwm.enable", prefix);
	if (retval < 0) goto error;
	data->pwm_enable[SPINDLE] = 0;

	retval = hal_pin_float_newf(HAL_IN, &(data->pwm_duty[SPINDLE]),
		comp_id, "%s.spindle.pwm.duty", prefix, n);
	if (retval < 0) goto error;
	*(data->pwm_duty[SPINDLE]) = 0.0;

	retval = hal_param_float_newf(HAL_RW, &(data->pwm_scale[SPINDLE]),
		comp_id,"%s.spindle.pwm.scale", prefix);
	if (retval < 0) goto error;
	data->pwm_scale[SPINDLE] = 1.0;

	retval = hal_pin_bit_newf(HAL_IN, &(data->pin_output[MIST]),
		comp_id, "%s.mist.enable", prefix);
	if (retval < 0) goto error;
	*(data->pin_output[MIST]) = 0;

	retval = hal_param_bit_newf(HAL_RW, &(data->pwm_enable[MIST]),
		comp_id, "%s.mist.pwm.enable", prefix);
	if (retval < 0) goto error;
	data->pwm_enable[MIST] = 0;

	retval = hal_pin_float_newf(HAL_IN, &(data->pwm_duty[MIST]),
		comp_id, "%s.mist.pwm.duty", prefix, n);
	if (retval < 0) goto error;
	*(data->pwm_duty[MIST]) = 0.0;

	retval = hal_param_float_newf(HAL_RW, &(data->pwm_scale[MIST]),
		comp_id,"%s.mist.pwm.scale", prefix);
	if (retval < 0) goto error;
	data->pwm_scale[MIST] = 1.0;

	retval = hal_pin_bit_newf(HAL_IN, &(data->pin_output[FLOOD]),
		comp_id, "%s.flood.enable", prefix);
	if (retval < 0) goto error;
	*(data->pin_output[FLOOD]) = 0;

	retval = hal_param_bit_newf(HAL_RW, &(data->pwm_enable[FLOOD]),
		comp_id, "%s.flood.pwm.enable", prefix);
	if (retval < 0) goto error;
	data->pwm_enable[FLOOD] = 0;

	retval = hal_pin_float_newf(HAL_IN, &(data->pwm_duty[FLOOD]),
		comp_id, "%s.flood.pwm.duty", prefix, n);
	if (retval < 0) goto error;
	*(data->pwm_duty[FLOOD]) = 0.0;

	retval = hal_param_float_newf(HAL_RW, &(data->pwm_scale[FLOOD]),
		comp_id,"%s.flood.pwm.scale", prefix);
	if (retval < 0) goto error;
	data->pwm_scale[FLOOD] = 1.0;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->ready), comp_id,
	        "%s.ready", prefix);
	if (retval < 0) goto error;
	*(data->ready) = 0;

	retval = hal_pin_bit_newf(HAL_IO, &(data->spi_fault), comp_id,
	        "%s.spi_fault", prefix);
	if (retval < 0) goto error;
	*(data->spi_fault) = 0;

error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}

	/* export functions */
	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, read_spi, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, write_spi, data, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_snprintf(name, sizeof(name), "%s.update", prefix);
	retval = hal_export_funct(name, update, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: update function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
	return 0;
}

void rtapi_app_exit(void)
{
	restore_gpio();
	munmap((void *)mem1,BLOCK_SIZE);
	munmap((void *)mem2,BLOCK_SIZE);
	hal_exit(comp_id);
}

void read_spi(void *arg, long period)
{
	int i;
	static int startup = 0;
	data_t *dat = (data_t *)arg;

	read_buf();
	/* update input status */
	update_inputs(dat);

	/* command >CM2 */
	txBuf[0] = CMD_CM2;
	update_outputs(dat);

	write_buf();

	/* check for change in period */
	if (period != old_dtns) {
		old_dtns = period;
		dt = period * 0.000000001;
		recip_dt = 1.0 / dt;
	}

	/* check for scale change */
	for (i = 0; i < NUMAXES; i++) {
		if (dat->scale[i] != old_scale[i]) {
			old_scale[i] = dat->scale[i];
			/* scale must not be 0 */
			if ((dat->scale[i] < 1e-20) && (dat->scale[i] > -1e-20))
				dat->scale[i] = 1.0;
			scale_inv[i] = (1.0 / STEP_MASK) / dat->scale[i];
		}
	}

	read_buf();

	/* sanity check */
	if (rxBuf[0] == (CMD_CM1 ^ ~0)) {
		*(dat->ready) = 1;
	} else {
		*(dat->ready) = 0;
		if (!startup)
			startup = 1;
		else
			*(dat->spi_fault) = 1;
	}

	/* update outputs */
	for (i = 0; i < NUMAXES; i++) {
		/* the DDS uses 32 bit counter, this code converts
		   that counter into 64 bits */
		accum_diff = get_position(i) - old_count[i];
		old_count[i] = get_position(i);
		accum[i] += accum_diff;

		*(dat->position_fb[i]) = (float)(accum[i]) * scale_inv[i];
	}
}

void write_spi(void *arg, long period)
{
	write_buf();
}

void update(void *arg, long period)
{
	int i;
	data_t *dat = (data_t *)arg;
	double max_accl, vel_cmd, dv, new_vel,
	       dp, pos_cmd, curr_pos, match_accl, match_time, avg_v,
	       est_out, est_cmd, est_err;

	for (i = 0; i < NUMAXES; i++) {
		/* set internal accel limit to its absolute max, which is
		   zero to full speed in one thread period */
		max_accl = max_vel * recip_dt;

		/* check for user specified accel limit parameter */
		if (dat->maxaccel[i] <= 0.0) {
			/* set to zero if negative */
			dat->maxaccel[i] = 0.0;
		} else {
			/* parameter is non-zero, compare to max_accl */
			if ((dat->maxaccel[i] * fabs(dat->scale[i])) > max_accl) {
				/* parameter is too high, lower it */
				dat->maxaccel[i] = max_accl / fabs(dat->scale[i]);
			} else {
				/* lower limit to match parameter */
				max_accl = dat->maxaccel[i] * fabs(dat->scale[i]);
			}
		}

		/* calculate position command in counts */
		pos_cmd = *(dat->position_cmd[i]) * dat->scale[i];
		/* calculate velocity command in counts/sec */
		vel_cmd = (pos_cmd - old_pos[i]) * recip_dt;
		old_pos[i] = pos_cmd;

		/* apply frequency limit */
		if (vel_cmd > max_vel) {
			vel_cmd = max_vel;
		} else if (vel_cmd < -max_vel) {
			vel_cmd = -max_vel;
		}

		/* determine which way we need to ramp to match velocity */
		if (vel_cmd > old_vel[i])
			match_accl = max_accl;
		else
			match_accl = -max_accl;

		/* determine how long the match would take */
		match_time = (vel_cmd - old_vel[i]) / match_accl;
		/* calc output position at the end of the match */
		avg_v = (vel_cmd + old_vel[i]) * 0.5;
		curr_pos = (double)(accum[i]) * (1.0 / STEP_MASK);
		est_out = curr_pos + avg_v * match_time;
		/* calculate the expected command position at that time */
		est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt);
		/* calculate error at that time */
		est_err = est_out - est_cmd;

		if (match_time < dt) {
			/* we can match velocity in one period */
			if (fabs(est_err) < 0.0001) {
				/* after match the position error will be acceptable */
				/* so we just do the velocity match */
				new_vel = vel_cmd;
			} else {
				/* try to correct position error */
				new_vel = vel_cmd - 0.5 * est_err * recip_dt;
				/* apply accel limits */
				if (new_vel > (old_vel[i] + max_accl * dt)) {
					new_vel = old_vel[i] + max_accl * dt;
				} else if (new_vel < (old_vel[i] - max_accl * dt)) {
					new_vel = old_vel[i] - max_accl * dt;
				}
			}
		} else {
			/* calculate change in final position if we ramp in the
			opposite direction for one period */
			dv = -2.0 * match_accl * dt;
			dp = dv * match_time;
			/* decide which way to ramp */
			if (fabs(est_err + dp * 2.0) < fabs(est_err)) {
				match_accl = -match_accl;
			}
			/* and do it */
			new_vel = old_vel[i] + match_accl * dt;
		}

		/* apply frequency limit */
		if (new_vel > max_vel) {
			new_vel = max_vel;
		} else if (new_vel < -max_vel) {
			new_vel = -max_vel;
		}

		old_vel[i] = new_vel;
		/* calculate new velocity cmd */
		update_velocity(i, (new_vel * VELSCALE));
	}

	/* this is a command (>CM1) */
	txBuf[0] = CMD_CM1;
}

void update_outputs(data_t *dat)
{
	float duty;
	int i;

	/* update pic32 output */
	txBuf[1] = (*(dat->pin_output[OUTPUT]) ? 1l : 0) << 0;

	for (i = 0; i < PWMCHANS; i++) {
		if (dat->pwm_enable[i]) {
			/* update pwm */
			duty = *(dat->pwm_duty[i]) * dat->pwm_scale[i] * 0.01;
			if (duty < 0.0) duty = 0.0;
			if (duty > 1.0) duty = 1.0;

			txBuf[2+i] = (duty * (1.0 + pwm_period));
		} else {
			/* simulate off/on using 0 or 100% duty cycle */
			txBuf[2+i] =  (*(dat->pin_output[i]) ?
						(1 + pwm_period) : 0);
		}
	}
}

static s32 debounce(s32 A)
{
	static s32 B = 0;
	static s32 C = 0;
	static s32 Z = 0;

	Z = (Z & (A | B | C)) | (A & B & C);
	C = B;
	B = A;

	return Z;
}

void update_inputs(data_t *dat)
{
	int i;
	s32 x;

	x = debounce(rxBuf[1]);

	for (i = 0; i < NUMINPUTS; i++) {
		*(dat->pin_input[i])  = (x & (0x1 << i)) ? 1 : 0;
	}
}

void rpi_read_buf()
{
	char *buf;
	int i;

	/* read buffer */
	buf = (char *)rxBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		*buf++ = BCM2835_SPIFIFO;
	}

}

void rpi_write_buf()
{
	char *buf;
	int i;

	/* activate transfer */
	BCM2835_SPICS = BCM_SPI_CS_TA | BCM_SPI_CS_CPHA;

	/* send txBuf */
	buf = (char *)txBuf;
	for (i=0; i<SPIBUFSIZE; i++) {
		BCM2835_SPIFIFO = *buf++;
	}

	/* wait until transfer is finished */
	while (!(BCM2835_SPICS & BCM_SPI_CS_DONE));

	/* deactivate transfer */
	BCM2835_SPICS = 0;

}

void c1_read_buf()
{
	u32 *buf;
	int i;

	/* wait until rx buffer is ready */
	while (!(ODROID_SPI_STAT & (1<<3)));

	/* read buffer */
	buf = (u32 *)rxBuf;
	for (i=0; i<BUFSIZE; i++) {
		*buf++ = __builtin_bswap32(ODROID_SPI_RX);
	}
}

void c1_write_buf()
{
	u32 *buf;
	int i;

	/* copy txBuf */
	buf = (u32 *)txBuf;
	for (i=0; i<BUFSIZE; i++) {
		ODROID_SPI_TX = __builtin_bswap32(*buf++);
	}

 	/* send tx burst */
	ODROID_SPI_CON |= (1<<2);

	/* wait until transfer is finished */
	while (ODROID_SPI_CON & (1<<2));
}

int map_gpio()
{
	int fd;
	static u32 mem1_base, mem2_base;

	switch (platform) {
	case RPI:
		mem1_base = BCM2835_GPIO_BASE;
		mem2_base = BCM2835_SPI_BASE;
		break;
	case RPI_2:
		mem1_base = BCM2835_GPIO_BASE + BCM2709_OFFSET;
		mem2_base = BCM2835_SPI_BASE + BCM2709_OFFSET;
		break;
	case ODROID_C1:
		mem1_base = ODROID_GPIO_BASE;
		mem2_base = ODROID_GCLK_MPEG0;
		break;
	}

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't open /dev/mem \n",modname);
		return -1;
	}

	mem1 = mmap(
	        NULL,
	        BLOCK_SIZE,
	        PROT_READ|PROT_WRITE,
	        MAP_SHARED,
	        fd,
	        mem1_base);

	if (mem1 == MAP_FAILED) {
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't map mem1\n",modname);
		close(fd);
		return -1;
	}

	mem2 = mmap(
	        NULL,
	        BLOCK_SIZE,
	        PROT_READ|PROT_WRITE,
	        MAP_SHARED,
	        fd,
	        mem2_base);

	close(fd);

	if (mem2 == MAP_FAILED) {
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: can't map mem2\n",modname);
		return -1;
	}

	return 0;
}

/*    GPIO USAGE
 *
 *    RPI:
 *
 *	GPIO	Dir	Signal
 *
 *	8	OUT	CE0
 *	9	IN	MISO
 *	10	OUT	MOSI
 *	11	OUT	SCLK
 *
 */

void rpi_setup_gpio()
{
	u32 x;

	/* change SPI pins */
	x = BCM2835_GPFSEL0;
	x &= ~(0x3F000000);
	x |= 0x24000000;
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0x0000003F);
	x |= 0x00000024;
	BCM2835_GPFSEL1 = x;

	/* set up SPI */
	BCM2835_SPICLK = BCM2835_SPICLKDIV;

	BCM2835_SPICS = 0;

	/* clear FIFOs */
	BCM2835_SPICS |= BCM_SPI_CS_CLEAR_RX | BCM_SPI_CS_CLEAR_TX;

}

void rpi_restore_gpio()
{
	u32 x;

	/* change SPI pins to inputs*/
	x = BCM2835_GPFSEL0;
	x &= ~(0x3F000000);
	BCM2835_GPFSEL0 = x;

	x = BCM2835_GPFSEL1;
	x &= ~(0x0000003F);
	BCM2835_GPFSEL1 = x;

}

/*    GPIO USAGE
 *
 *    ODROID C1:
 *
 *	GPIO	 Dir	Signal
 *
 *	GPIOX_20 OUT	CE0
 *	GPIOX_9	 IN	MISO
 *	GPIOX_10 OUT	MOSI
 *	GPIOX_8	 OUT	SCLK
 *
 */

void c1_setup_gpio()
{
	u32 x;

	/* set SPI direction pins */
	x = ODROID_GPIOX_OEN;
	x &= ~(0x00100700);
	x |= 0x00000200;
	ODROID_GPIOX_OEN = x;

	/* enable pull-down on all pins */
	x = ODROID_GPIOX_PUPD;
	x &= ~(0x00100700);
	ODROID_GPIOX_PUPD = x;

	/* activate pull-down */
	x = ODROID_GPIOX_PUEN;
	x |= 0x00100700;
	ODROID_GPIOX_PUEN = x;

	/* enable PSI pinmux */
	ODROID_PPMUX_3 &= ~(0x004003C0);
	ODROID_PPMUX_5 &= ~(0x00000C00);
	ODROID_PPMUX_6 &= ~(0x000F0000);
	ODROID_PPMUX_7 &= ~(0x80000000);
	ODROID_PPMUX_8 &= ~(0x00000003);
	ODROID_PPMUX_9 &= ~(0x00080000);
	ODROID_PPMUX_4 |= 0x03C00000;

	/* enable SPI clk */
	ODROID_SPI_CLKGATE |= (1<<8);
	nanosleep((struct timespec[]){{0, 10000}}, NULL);

	/* disable SPI */
	ODROID_SPI_CON = 0;

	ODROID_SPI_CON = (4<<25) | 	/* 5-1 words/burst */
			 (31<<19) |	/* 32-1 bits */
			 (0x3<<16) | 	/* ~4MHz clock rate */
			 (0<<5) |	/* CKPHA=1 */
			 (1<<4) |	/* CKPOL=0 */
			 (1<<1);	/* master */

	/* enable SPI */
	ODROID_SPI_CON |= 1<<0;

	/* clear SPI RX buffer */
	int i;
	for (i=0; i<ODROID_FIFO_SIZE; i++) x = ODROID_SPI_RX;

}

void c1_restore_gpio()
{
	u32 x;

	/* disable SPI */
	ODROID_SPI_CON &= ~(1<<0);

	/* disable SPI clk */
	ODROID_SPI_CLKGATE &= ~(1<<8);

	/* disable SPI pinmux */
	ODROID_PPMUX_4 &= ~(0x03C00000);

	/* change all used pins back to inputs */
	x = ODROID_GPIOX_OEN;
	x |= 0x00100700;
	ODROID_GPIOX_OEN = x;

	/* restore pull-up/down to defaults */
	x = ODROID_GPIOX_PUPD;
	x &= ~(0x00100000);
	x |= 0x00000700;
	ODROID_GPIOX_PUPD = x;

	x = ODROID_GPIOX_PUEN;
	x |= 0x00100700;
	ODROID_GPIOX_PUEN = x;

}
