/*
 * Rafael Micro R820T/R828D driver
 *
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 *
 * This driver is a heavily modified version of the driver found in the
 * Linux kernel:
 * http://git.linuxtv.org/linux-2.6.git/history/HEAD:/drivers/media/tuners/r820t.c
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "rtlsdr_i2c.h"
#include "tuner_r848.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MHZ(x)		((x)*1000*1000)
#define KHZ(x)		((x)*1000)
/*
 * Static constants
 */

/* Those initial values start from REG_SHADOW_START */
static const uint8_t r848_init_array[R848_NUM_REGS] = {
};

/* Tuner frequency ranges */
static const struct r848_freq_range freq_ranges[] = {
};

static int r848_xtal_capacitor[][2] = {
};

/*
 * I2C read/write code and shadow registers logic
 */
static void shadow_store(struct r848_priv *priv, uint8_t reg, const uint8_t *val,
			 int len)
{
	int r = reg - REG_SHADOW_START;

	if (r < 0) {
		len += r;
		r = 0;
	}
	if (len <= 0)
		return;
	if (len > R848_NUM_REGS - r)
		len = R848_NUM_REGS - r;

	memcpy(&priv->regs[r], val, len);
}

static int r848_write(struct r848_priv *priv, uint8_t reg, const uint8_t *val,
		       unsigned int len)
{
	int rc, size, pos = 0;

	/* Store the shadow registers */
	shadow_store(priv, reg, val, len);

	do {
		if (len > priv->cfg->max_i2c_msg_len - 1)
			size = priv->cfg->max_i2c_msg_len - 1;
		else
			size = len;

		/* Fill I2C buffer */
		priv->buf[0] = reg;
		memcpy(&priv->buf[1], &val[pos], size);

		rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr,
					 priv->buf, size + 1);

		if (rc != size + 1) {
			fprintf(stderr, "%s: i2c wr failed=%d reg=%02x len=%d\n",
				   __FUNCTION__, rc, reg, size);
			if (rc < 0)
				return rc;
			return -1;
		}

		reg += size;
		len -= size;
		pos += size;
	} while (len > 0);

	return 0;
}

static int r848_write_reg(struct r848_priv *priv, uint8_t reg, uint8_t val)
{
	return r848_write(priv, reg, &val, 1);
}

static int r848_read_cache_reg(struct r848_priv *priv, int reg)
{
	reg -= REG_SHADOW_START;

	if (reg >= 0 && reg < R848_NUM_REGS)
		return priv->regs[reg];
	else
		return -1;
}

static int r848_write_reg_mask(struct r848_priv *priv, uint8_t reg, uint8_t val,
				uint8_t bit_mask)
{
	int rc = r848_read_cache_reg(priv, reg);

	if (rc < 0)
		return rc;

	val = (rc & ~bit_mask) | (val & bit_mask);

	return r848_write(priv, reg, &val, 1);
}

static uint8_t r848_bitrev(uint8_t byte)
{
	const uint8_t lut[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
				  0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return (lut[byte & 0xf] << 4) | lut[byte >> 4];
}

static int r848_read(struct r848_priv *priv, uint8_t reg, uint8_t *val, int len)
{
	printf( "reading\n");
	int rc, i;
	uint8_t *p = &priv->buf[1];

	priv->buf[0] = reg;

	rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr, priv->buf, 1);
	if (rc < 1)
		return rc;

	rc = rtlsdr_i2c_read_fn(priv->rtl_dev, priv->cfg->i2c_addr, p, len);

	if (rc != len) {
		fprintf(stderr, "%s: i2c rd failed=%d reg=%02x len=%d\n",
			   __FUNCTION__, rc, reg, len);
		if (rc < 0)
			return rc;
		return -1;
	}

	/* Copy data to the output buffer */
	for (i = 0; i < len; i++)
		val[i] = r848_bitrev(p[i]);

	return 0;
}

/*
 * r848 tuning logic
 */

static int r848_set_mux(struct r848_priv *priv, uint32_t freq)
{

}

static int r848_set_pll(struct r848_priv *priv, uint32_t freq)
{
}

static int r848_sysfreq_sel(struct r848_priv *priv, uint32_t freq,
			     enum r848_tuner_type type,
			     uint32_t delsys)
{
}

static int r848_set_tv_standard(struct r848_priv *priv,
				 unsigned bw,
				 enum r848_tuner_type type,
				 uint32_t delsys)

{
}

static int r848_read_gain(struct r848_priv *priv)
{
}

int r848_set_gain(struct r848_priv *priv, int set_manual_gain, int gain)
{
}

int r848_set_bandwidth(struct r848_priv *priv, int bw, uint32_t rate)
{
}

int r848_set_freq(struct r848_priv *priv, uint32_t freq)
{
}

/*
 * r848 standby logic
 */

int r848_standby(struct r848_priv *priv)
{
}

/*
 * r848 device init logic
 */

static int r848_xtal_check(struct r848_priv *priv)
{
}

int r848_init(struct r848_priv *priv)
{
}

