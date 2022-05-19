// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/i2c/busses/i2c-sunxi.c
 *
 * Copyright (C) 2013 Allwinner.
 * Pan Nan <pannan@reuuimllatech.com>
 *
 * SUNXI TWI Controller Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * 2013.5.3 Mintow <duanmintao@allwinnertech.com>
 *    Adapt to all the new chip of Allwinner. Support sun8i/sun9i
 */

//#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk/sunxi.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "i2c-sunxi.h"
#include <linux/regulator/consumer.h>

#define SUNXI_I2C_OK      0
#define SUNXI_I2C_FAIL   -1
#define SUNXI_I2C_RETRY  -2
#define SUNXI_I2C_SFAIL  -3  /* start fail */
#define SUNXI_I2C_TFAIL  -4  /* stop  fail */

#define DMA_THRESHOLD	32
#define MAX_FIFO	32
#define DMA_TIMEOUT	1000


/* I2C transfer status */
enum {
	I2C_XFER_IDLE    = 0x1,
	I2C_XFER_START   = 0x2,
	I2C_XFER_RUNNING = 0x4,
};

struct sunxi_i2c_dma {
	struct dma_chan *chan;
	dma_addr_t dma_buf;
	unsigned int dma_len;
	enum dma_transfer_direction dma_transfer_dir;
	enum dma_data_direction dma_data_dir;
};

struct sunxi_i2c {
	/* i2c framework datai */
	struct i2c_adapter adap;
	struct platform_device *pdev;
	struct device *dev;
	struct i2c_msg *msg;
	/* the total num of msg */
	unsigned int msg_num;
	/* the current msg index -> msg[msg_idx] */
	unsigned int msg_idx;
	/* the current msg's buf data index -> msg->buf[msg_ptr]*/
	unsigned int msg_ptr;
	unsigned char result;

	/* dts data */
	struct resource *res;
	void __iomem *base_addr;
	struct clk *bus_clk;
	struct reset_control    *reset;
	unsigned int bus_freq;
	struct regulator *regulator;
	struct pinctrl *pctrl;
	int irq;
	int irq_flag;
	unsigned int twi_drv_used;
	unsigned int no_suspend;
	unsigned int pkt_interval;
	struct sunxi_i2c_dma *dma_tx;
	struct sunxi_i2c_dma *dma_rx;
	struct sunxi_i2c_dma *dma_using;
	u8 *dma_buf;

	/* other data */
	int bus_num;
	unsigned int status; /* start, running, idle */
	unsigned int debug_state; /* log the twi machine state */

	spinlock_t lock; /* syn */
	wait_queue_head_t wait;
	struct completion cmd_complete;

	unsigned int reg1[16]; /* store the i2c engined mode resigter status */
	unsigned int reg2[16]; /* store the i2c drv mode regiter status */
};

void dump_reg(struct sunxi_i2c *i2c, u32 offset, u32 len)
{
	u32 i;
	u8 buf[64], cnt = 0;

	for (i = 0; i < len; i = i + REG_INTERVAL) {
		if (i%HEXADECIMAL == 0)
			cnt += sprintf(buf + cnt, "0x%08x: ",
				       (u32)(i2c->res->start + offset + i));

		cnt += sprintf(buf + cnt, "%08x ",
			       readl(i2c->base_addr + offset + i));

		if (i%HEXADECIMAL == REG_CL) {
			pr_warn("%s\n", buf);
			cnt = 0;
		}
	}
}

/* clear the interrupt flag */
static inline void twi_clear_irq_flag(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);
	/* start and stop bit should be 0 */
	reg_val |= TWI_CTL_INTFLG;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP);
	writel(reg_val, base_addr + TWI_CTL_REG);
	/* read two more times to make sure that */
	/* interrupt flag does really be cleared */
	{
		unsigned int temp;

		temp = readl(base_addr + TWI_CTL_REG);
		temp |= readl(base_addr + TWI_CTL_REG);
	}
}

/* get data first, then clear flag */
static inline void
twi_get_byte(void __iomem *base_addr, unsigned char  *buffer)
{
	*buffer = (unsigned char)(TWI_DATA_MASK & readl(base_addr + TWI_DATA_REG));
	twi_clear_irq_flag(base_addr);
}

/* only get data, we will clear the flag when stop */
static inline void
twi_get_last_byte(void __iomem *base_addr, unsigned char  *buffer)
{
	*buffer = (unsigned char)(TWI_DATA_MASK & readl(base_addr + TWI_DATA_REG));
}

/* write data and clear irq flag to trigger send flow */
static inline void
twi_put_byte(struct sunxi_i2c *i2c, const unsigned char *buffer)
{
	dev_dbg(i2c->dev, "send data is %x\n", (unsigned int)*buffer);

	writel((unsigned int)*buffer, i2c->base_addr + TWI_DATA_REG);
	twi_clear_irq_flag(i2c->base_addr);
}

static inline void twi_enable_irq(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	/*
	 * 1 when enable irq for next operation, set intflag to 0 to prevent
	 * to clear it by a mistake (intflag bit is write-1-to-clear bit)
	 * 2 Similarly, mask START bit and STOP bit to prevent to set it
	 * twice by a mistake (START bit and STOP bit are self-clear-to-0 bits)
	 */
	reg_val |= TWI_CTL_INTEN;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP | TWI_CTL_INTFLG);
	writel(reg_val, base_addr + TWI_CTL_REG);
}

static inline void twi_disable_irq(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val &= ~TWI_CTL_INTEN;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP | TWI_CTL_INTFLG);
	writel(reg_val, base_addr + TWI_CTL_REG);
}

static inline void sunxi_i2c_bus_enable(struct sunxi_i2c *i2c)
{
	unsigned int reg_val;

	if (i2c->twi_drv_used) {
		reg_val = readl(i2c->base_addr + TWI_DRIVER_CTRL);
		reg_val |= TWI_DRV_EN;
		writel(reg_val, i2c->base_addr + TWI_DRIVER_CTRL);
	} else {
		reg_val = readl(i2c->base_addr + TWI_CTL_REG);
		reg_val |= TWI_CTL_BUSEN;
		writel(reg_val, i2c->base_addr + TWI_CTL_REG);
	}
}

static inline void sunxi_i2c_bus_disable(struct sunxi_i2c *i2c)
{
	unsigned int reg_val;

	if (i2c->twi_drv_used) {
		reg_val = readl(i2c->base_addr + TWI_DRIVER_CTRL);
		reg_val &= ~TWI_DRV_EN;
		writel(reg_val, i2c->base_addr + TWI_DRIVER_CTRL);
	} else {
		reg_val = readl(i2c->base_addr + TWI_CTL_REG);
		reg_val &= ~TWI_CTL_BUSEN;
		writel(reg_val, i2c->base_addr + TWI_CTL_REG);
	}
}

/* trigger start signal, the start bit will be cleared automatically */
static inline void twi_set_start(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_STA;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get start bit status, poll if start signal is sent */
static inline unsigned int twi_get_start(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val >>= 5;
	return reg_val & 1;
}

/* trigger stop signal, the stop bit will be cleared automatically */
static inline void twi_set_stop(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_STP;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get stop bit status, poll if stop signal is sent */
static inline unsigned int twi_get_stop(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val >>= 4;
	return reg_val & 1;
}

static inline void twi_disable_ack(void __iomem  *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val &= ~TWI_CTL_ACK;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* when sending ack or nack, it will send ack automatically */
static inline void twi_enable_ack(void __iomem  *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_ACK;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get the interrupt flag */
static inline unsigned int twi_query_irq_flag(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	return (reg_val & TWI_CTL_INTFLG);/* 0x 0000_1000 */
}

/* get interrupt status */
static inline unsigned int twi_query_irq_status(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_STAT_REG);

	return (reg_val & TWI_STAT_MASK);
}

/* set twi clock
 *
 * clk_n: clock divider factor n
 * clk_m: clock divider factor m
 */
static void sunxi_i2c_clk_write_reg(struct sunxi_i2c *i2c, unsigned int reg_clk,
		unsigned int sclk_freq,
		unsigned char clk_m, unsigned char clk_n,
		unsigned int mask_clk_m, unsigned int mask_clk_n)
{
	unsigned int reg_val = readl(i2c->base_addr + reg_clk);
#if IS_ENABLED(CONFIG_ARCH_SUN50IW10)
	unsigned int duty;
#endif
	dev_dbg(i2c->dev, "reg_clk = 0x%x, clk_m = %u, clk_n = %u,"
		"mask_clk_m = %x, mask_clk_n = %x\n",
		reg_clk, clk_m, clk_n, mask_clk_m, mask_clk_n);
	if (reg_clk == TWI_DRIVER_BUSC) {
		reg_val &= ~(mask_clk_m | mask_clk_n);
		reg_val |= ((clk_m | (clk_n << 4)) << 8);
#if IS_ENABLED(CONFIG_ARCH_SUN50IW10)
		duty = TWI_DRV_CLK_DUTY;
		if (sclk_freq > STANDDARD_FREQ)
			reg_val |= duty;
		else
			reg_val &= ~duty;
#endif
		writel(reg_val, i2c->base_addr + reg_clk);
		dev_dbg(i2c->dev, "reg: 0x%x value: 0x%x\n",
			reg_clk, readl(i2c->base_addr + reg_clk));
	} else {
		reg_val &= ~(mask_clk_m | mask_clk_n);
		reg_val |= ((clk_m  << 3) | clk_n);
#if IS_ENABLED(CONFIG_ARCH_SUN50IW10)
		duty = TWI_CLK_DUTY;
		if (sclk_freq > STANDDARD_FREQ)
			reg_val |= duty;
		else
			reg_val &= ~duty;
#endif
		writel(reg_val, i2c->base_addr + reg_clk);
		dev_dbg(i2c->dev, "reg: 0x%x value: 0x%x\n",
			reg_clk, readl(i2c->base_addr + reg_clk));
	}
}

/*
 * Fin is APB CLOCK INPUT;
 * Fsample = F0 = Fin/2^CLK_N;
 * F1 = F0/(CLK_M+1);
 * Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10);
 * Foscl is clock SCL;100KHz or 400KHz
 *
 * clk_in: apb clk clock
 * sclk_freq: freqence to set in HZ
 */
static int sunxi_i2c_set_clock(struct sunxi_i2c *i2c, unsigned int reg_clk,
		unsigned int clk_in, unsigned int sclk_freq,
		unsigned int mask_clk_m, unsigned int mask_clk_n)
{
	/*
	 *unsigned int clk_m = 0;
	 *unsigned int clk_n = 0;
	 *unsigned int _2_pow_clk_n = 1;
	 */
	unsigned char clk_m = 0, clk_n = 0, _2_pow_clk_n = 1;
	unsigned int src_clk      = clk_in / 10;
	unsigned int divider      = src_clk / sclk_freq;  /* 400khz or 100khz */
	unsigned int sclk_real    = 0;      /* the real clock frequency */

	if (divider == 0) {
		clk_m = 1;
		goto set_clk;
	}

	/*
	 * search clk_n and clk_m,from large to small value so
	 * that can quickly find suitable m & n.
	 */
	while (clk_n < 8) { /* 3bits max value is 8 */
		/* (m+1)*2^n = divider -->m = divider/2^n -1 */
		clk_m = (divider / _2_pow_clk_n) - 1;
		/* clk_m = (divider >> (_2_pow_clk_n>>1))-1 */
		while (clk_m < 16) { /* 4bits max value is 16 */
			/* src_clk/((m+1)*2^n) */
			sclk_real = src_clk / (clk_m + 1) / _2_pow_clk_n;
			if (sclk_real <= sclk_freq)
				goto set_clk;
			else
				clk_m++;
		}
		clk_n++;
		_2_pow_clk_n *= 2; /* mutilple by 2 */
	}

set_clk:
	sunxi_i2c_clk_write_reg(i2c, reg_clk, sclk_freq, clk_m,
			  clk_n, mask_clk_m, mask_clk_n);
	return 0;
}

/* soft reset twi */
static inline void
twi_soft_reset(void __iomem *base_addr, unsigned int reg, unsigned int mask)
{
	unsigned int reg_val = readl(base_addr + reg);

	reg_val |= mask;
	writel(reg_val, base_addr + reg);
}

/* Enhanced Feature Register */
static inline void twi_set_efr(void __iomem *base_addr, unsigned int efr)
{
	unsigned int reg_val = readl(base_addr + TWI_EFR_REG);

	reg_val &= ~TWI_EFR_MASK;
	efr     &= TWI_EFR_MASK;
	reg_val |= efr;
	writel(reg_val, base_addr + TWI_EFR_REG);
}

static int sunxi_i2c_xfer_complete(struct sunxi_i2c *i2c, int code);
static int
sunxi_i2c_do_xfer(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num);

static int twi_start(struct sunxi_i2c *i2c)
{
	unsigned int timeout = 0xff;

	twi_set_start(i2c->base_addr);
	while ((twi_get_start(i2c->base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		dev_err(i2c->dev, "START can't sendout!\n");
		return SUNXI_I2C_FAIL;
	}

	return SUNXI_I2C_OK;
}

static int twi_restart(struct sunxi_i2c *i2c)
{
	unsigned int timeout = 0xff;

	twi_set_start(i2c->base_addr);
	twi_clear_irq_flag(i2c->base_addr);
	while ((twi_get_start(i2c->base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		dev_err(i2c->dev, "Restart can't sendout!\n");
		return SUNXI_I2C_FAIL;
	}

	return SUNXI_I2C_OK;
}
static int twi_stop(struct sunxi_i2c *i2c)
{
	unsigned int timeout = 0xff;
	void __iomem *base_addr = i2c->base_addr;

	twi_set_stop(base_addr);
	twi_clear_irq_flag(base_addr);

	twi_get_stop(base_addr);/* it must delay 1 nop to check stop bit */
	while ((twi_get_stop(base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		dev_err(i2c->dev, "STOP can't sendout!\n");
		return SUNXI_I2C_TFAIL;
	}

	timeout = 0xff;
	while ((readl(base_addr + TWI_STAT_REG) != TWI_STAT_IDLE) &&
	       (--timeout))
		;
	if (timeout == 0) {
		dev_err(i2c->dev, "i2c state(0x%0x) isn't idle(0xf8)\n",
				readl(base_addr + TWI_STAT_REG));
		return SUNXI_I2C_TFAIL;
	}

	timeout = 0xff;
	while ((readl(base_addr + TWI_LCR_REG) != TWI_LCR_IDLE_STATUS &&
		readl(base_addr + TWI_LCR_REG) != TWI_LCR_NORM_STATUS) &&
		(--timeout))
		;

	if (timeout == 0) {
		dev_err(i2c->dev, "i2c lcr(0x%0x) isn't idle(0x3a)\n",
				readl(base_addr + TWI_LCR_REG));
		return SUNXI_I2C_TFAIL;
	}

	return SUNXI_I2C_OK;
}

/* get SDA state */
static unsigned int twi_get_sda(void __iomem *base_addr)
{
	unsigned int status = 0;

	status = TWI_LCR_SDA_STATE_MASK & readl(base_addr + TWI_LCR_REG);
	status >>= 4;
	return (status & 0x1);
}

/* set SCL level(high/low), only when SCL enable */
static void twi_set_scl(void __iomem *base_addr, unsigned char hi_lo)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	reg_val &= ~TWI_LCR_SCL_CTL;
	hi_lo   &= 0x01;
	reg_val |= (hi_lo << 3);
	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* enable SDA or SCL */
static void twi_enable_lcr(void __iomem *base_addr, unsigned char sda_scl)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	sda_scl &= 0x01;
	if (sda_scl)
		reg_val |= TWI_LCR_SCL_EN;/* enable scl line control */
	else
		reg_val |= TWI_LCR_SDA_EN;/* enable sda line control */

	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* disable SDA or SCL */
static void twi_disable_lcr(void __iomem *base_addr, unsigned char sda_scl)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	sda_scl &= 0x01;
	if (sda_scl)
		reg_val &= ~TWI_LCR_SCL_EN;/* disable scl line control */
	else
		reg_val &= ~TWI_LCR_SDA_EN;/* disable sda line control */

	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* send 9 clock to release sda */
static int twi_send_clk_9pulse(struct sunxi_i2c *i2c)
{
	char twi_scl = 1, low = 0, high = 1, cycle = 0;
	unsigned char status;
	void __iomem *base_addr = i2c->base_addr;

	/* enable scl control */
	twi_enable_lcr(base_addr, twi_scl);

	while (cycle < 9) {
		if (twi_get_sda(base_addr) &&
		    twi_get_sda(base_addr) &&
		    twi_get_sda(base_addr)) {
			break;
		}
		/* twi_scl -> low */
		twi_set_scl(base_addr, low);
		udelay(1000);

		/* twi_scl -> high */
		twi_set_scl(base_addr, high);
		udelay(1000);
		cycle++;
	}

	if (twi_get_sda(base_addr)) {
		twi_disable_lcr(base_addr, twi_scl);
		status =  SUNXI_I2C_OK;
	} else {
		dev_err(i2c->dev, "SDA is still Stuck Low, failed.\n");
		twi_disable_lcr(base_addr, twi_scl);
		status =  SUNXI_I2C_FAIL;
	}

	return status;
}

static unsigned int twi_drv_query_irq_status(void __iomem *base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_DRIVER_INTC);

	return (reg_val & TWI_DRV_STAT_MASK);
}

static void twi_drv_clear_irq_flag(u32 pending_bit, void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_INTC);

	pending_bit &= TWI_DRV_STAT_MASK;
	reg_val |= pending_bit;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

static void i2c_drv_clear_pending(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val |= TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

/* start i2c transfer */
static void i2c_start_xfer(void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val |= START_TRAN;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

/*
 * send DMA RX Req when the data byte number in RECV_FIFO reaches RX_TRIG
 * or Read Packet Tansmission completed with RECV_FIFO not empty
 */
static void i2c_set_rx_trig_level(u32 val, void __iomem *base_addr)
{
	u32 mask = TRIG_MASK;
	u32 reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	val = (val & mask) << 16;
	reg_val &= ~(mask << 16);
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}

/* bytes be send as slave device reg address */
static void i2c_set_packet_addr_byte(u32 val, void __iomem *base_addr)
{
	u32 mask = ADDR_BYTE;
	u32 reg_val = readl(base_addr + TWI_DRIVER_FMT);

	reg_val &= ~mask;
	val = (val << 16) & mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_FMT);
}

/* bytes be send/received as data */
static void i2c_set_packet_data_byte(u32 val, void __iomem *base_addr)
{
	u32 mask = DATA_BYTE;
	u32 reg_val = readl(base_addr + TWI_DRIVER_FMT);

	reg_val &= ~mask;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_FMT);
}

/* interval between each packet in 32*Fscl cycles */
static void i2c_set_packet_interval(u32 val, void __iomem *base_addr)
{
	u32 mask = INTERVAL_MASK;
	u32 reg_val = readl(base_addr + TWI_DRIVER_CFG);

	reg_val &= ~mask;
	val <<= 16;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_CFG);
}

/* FIFO data be transmitted as PACKET_CNT packets in current format */
static void i2c_set_packet_cnt(u32 val, void __iomem *base_addr)
{
	u32 mask = PACKET_MASK;
	u32 reg_val = readl(base_addr + TWI_DRIVER_CFG);

	reg_val &= ~mask;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_CFG);
}

/* do not send slave_id +W */
static void i2c_enable_read_tran_mode(void __iomem *base_addr)
{
	u32 mask = READ_TRAN;
	u32 reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val |= mask;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

/* send slave_id + W */
static void i2c_disable_read_tran_mode(void __iomem *base_addr)
{
	u32 mask = READ_TRAN;
	u32 reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val &= ~mask;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

static void i2c_drv_enable_tran_irq(u32 bitmap, void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val |= bitmap;
	reg_val &= ~TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

static void i2c_drv_disable_tran_irq(u32 bitmap, void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val &= ~bitmap;
	reg_val &= ~TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

static void i2c_drv_enable_dma_irq(u32 bitmap, void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	bitmap &= I2C_DRQEN_MASK;
	reg_val |= bitmap;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}

static void i2c_drv_disable_dma_irq(u32 bitmap, void __iomem *base_addr)
{
	u32 reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	bitmap &= I2C_DRQEN_MASK;
	reg_val &= ~bitmap;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}

static void
sunxi_i2c_drv_slave_addr(struct sunxi_i2c *i2c, struct i2c_msg *msgs)
{
	unsigned int val = 0, cmd = 0;

	/* read, default value is write */
	if (msgs->flags & I2C_M_RD)
		cmd = SLV_RD_CMD;

	if (msgs->flags & I2C_M_TEN) {
		/* SLV_ID | CMD | SLV_ID_X */
		val = ((0x78 | ((msgs->addr >> 8) & 0x03)) << 9) | cmd
			| (msgs->addr & 0xff);
		dev_dbg(i2c->dev, "10bit addr\n");
	} else {
		val = ((msgs->addr & 0x7f) << 9) | cmd;
		dev_dbg(i2c->dev, "7bit addr\n");
	}

	writel(val, i2c->base_addr + TWI_DRIVER_SLV);
	dev_dbg(i2c->dev, "offset: 0x%x value: 0x%x\n",
			TWI_DRIVER_SLV, readl(i2c->base_addr + TWI_DRIVER_SLV));
}

/* the number of data in SEND_FIFO */
static int i2c_query_txfifo(void __iomem *base_addr)
{
	unsigned int reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC) & SEND_FIFO_CONT;

	return reg_val;
}

/* the number of data in RECV_FIFO */
static int i2c_query_rxfifo(void __iomem *base_addr)
{
	unsigned int reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC) & RECV_FIFO_CONT;
	reg_val >>= 16;

	return reg_val;
}

static void i2c_clear_txfifo(void __iomem *base_addr)
{
	unsigned int reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC);
	reg_val |= SEND_FIFO_CLEAR;
	writel(reg_val, base_addr + TWI_DRIVER_FIFOC);
}

static void i2c_clear_rxfifo(void __iomem *base_addr)
{
	unsigned int reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC);
	reg_val |= RECV_FIFO_CLEAR;
	writel(reg_val, base_addr + TWI_DRIVER_FIFOC);
}

static int i2c_sunxi_send_msgs(struct sunxi_i2c *i2c, struct i2c_msg *msgs)
{
	u16 i;
	u8 time = 0xff;

	dev_dbg(i2c->dev, "msgs->len = %d\n", msgs->len);

	for (i = 0; i < msgs->len; i++) {
		while ((i2c_query_txfifo(i2c->base_addr) >= MAX_FIFO) && time--)
			;
		if (time) {
			writeb(msgs->buf[i], i2c->base_addr + TWI_DRIVER_SENDF);
			dev_dbg(i2c->dev, "writeb: Byte[%u] = 0x%x,fifo len = %d\n",
				i, msgs->buf[i], i2c_query_txfifo(i2c->base_addr));
		} else {
			dev_err(i2c->dev, "SEND FIFO overflow. timeout\n");
			return -EINVAL;
		}
	}

	return 0;
}

static unsigned int
i2c_sunxi_recv_msgs(struct sunxi_i2c *i2c, struct i2c_msg *msgs)
{
	u16 i;
	u8 time = 0xff;

	dev_dbg(i2c->dev, "msgs->len = %d\n", msgs->len);

	for (i = 0; i < msgs->len; i++) {
		while (!i2c_query_rxfifo(i2c->base_addr) && time--)
			;
		if (time) {
			msgs->buf[i] = readb(i2c->base_addr + TWI_DRIVER_RECVF);
			dev_dbg(i2c->dev, "readb: Byte[%d] = 0x%x\n", i, msgs->buf[i]);
		} else
			return 0;
	}
	return msgs->len;
}

static int sunxi_i2c_drv_core_process(struct sunxi_i2c *i2c)
{
	void __iomem *base_addr = i2c->base_addr;
	unsigned long flags = 0;
	unsigned int status, code;

	spin_lock_irqsave(&i2c->lock, flags);

	status = twi_drv_query_irq_status(base_addr);
	twi_drv_clear_irq_flag(status, base_addr);
	dev_dbg(i2c->dev, "irq status = 0x%x\n", status);

	if (status & TRAN_COM_PD) {
		i2c_drv_disable_tran_irq(TRAN_COM_INT, i2c->base_addr);
		i2c->result = RESULT_COMPLETE;
		wake_up(&i2c->wait);
		dev_dbg(i2c->dev, "packet transmission completed\n");

		if ((status & RX_REQ_PD) && (i2c->msg->len < DMA_THRESHOLD)) {
			i2c_sunxi_recv_msgs(i2c, i2c->msg);
		}
	}

	if (status & TRAN_ERR_PD) {
		i2c_drv_disable_tran_irq(TRAN_ERR_INT, i2c->base_addr);
		code = readl(base_addr + TWI_DRIVER_CTRL);
		code = (code & TWI_DRV_STA) >> 16;
		switch (code) {
		case 0x00:
			dev_err(i2c->dev, "bus error\n");
			break;
		case 0x01:
			dev_err(i2c->dev, "Timeout when sending 9th SCL clk\n");
			break;
		case 0x20:
			dev_err(i2c->dev, "Address + Write bit transmitted,"
					"ACK not received\n");
			break;
		case 0x30:
			dev_err(i2c->dev, "Data byte transmitted in master mode,"
				"ACK not received\n");
			break;
		case 0x38:
			dev_err(i2c->dev, "Arbitration lost in address, or data byte\n");
			break;
		case 0x48:
			dev_err(i2c->dev, "Address + Read bit transmitted,"
				"ACK not received\\n");
			break;
		case 0x58:
			dev_err(i2c->dev, "Data byte received in master mode,"
				"ACK not received\n");
			break;
		default:
			dev_err(i2c->dev, "unknown error\n");
			break;
		}
		i2c->msg_idx = code;
		i2c->result = RESULT_ERR;
		dev_dbg(i2c->dev, "packet transmission failed\n");
		wake_up(&i2c->wait);
		spin_unlock_irqrestore(&i2c->lock, flags);
		return code;
	}

	spin_unlock_irqrestore(&i2c->lock, flags);
	return 0;
}

/* Functions for DMA support */
static int sunxi_i2c_dma_request(struct sunxi_i2c *i2c, dma_addr_t phy_addr)
{
	struct sunxi_i2c_dma *dma_tx, *dma_rx;
	struct dma_slave_config dma_sconfig;
	int err;

	dma_tx = devm_kzalloc(i2c->dev, sizeof(*dma_tx), GFP_KERNEL);
	dma_rx = devm_kzalloc(i2c->dev, sizeof(*dma_rx), GFP_KERNEL);
	if (IS_ERR_OR_NULL(dma_tx) || IS_ERR_OR_NULL(dma_rx)) {
		dev_err(i2c->dev, "dma kzalloc failed\n");
		return -EINVAL;
	}
	dma_tx->chan = dma_request_chan(i2c->dev, "tx");
	if (IS_ERR(dma_tx->chan)) {
		dev_err(i2c->dev, "can't request DMA tx channel\n");
		err = PTR_ERR(dma_tx->chan);
		goto err0;
	}
	dma_sconfig.dst_addr = phy_addr + TWI_DRIVER_SENDF;
	dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.src_maxburst = 16;
	dma_sconfig.dst_maxburst = 16;
	dma_sconfig.direction = DMA_MEM_TO_DEV;
	err = dmaengine_slave_config(dma_tx->chan, &dma_sconfig);
	if (err < 0) {
		dev_err(i2c->dev, "can't configure tx channel\n");
		goto err1;
	}
	i2c->dma_tx = dma_tx;

	dma_rx->chan = dma_request_chan(i2c->dev, "rx");
	if (IS_ERR(dma_rx->chan)) {
		dev_err(i2c->dev, "can't request DMA rx channel\n");
		goto err1;
	}
	dma_sconfig.src_addr = phy_addr + TWI_DRIVER_RECVF;
	dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.src_maxburst = 16;
	dma_sconfig.dst_maxburst = 16;
	dma_sconfig.direction = DMA_DEV_TO_MEM;
	err = dmaengine_slave_config(dma_rx->chan, &dma_sconfig);
	if (err < 0) {
		dev_err(i2c->dev, "can't configure rx channel\n");
		goto err2;
	}
	i2c->dma_rx = dma_rx;

	init_completion(&i2c->cmd_complete);
	dev_dbg(i2c->dev, "using %s (tx) and %s (rx) for DMA transfers\n",
		dma_chan_name(i2c->dma_tx->chan), dma_chan_name(i2c->dma_rx->chan));

	return 0;

err2:
	dma_release_channel(i2c->dma_rx->chan);

err1:
	dma_release_channel(i2c->dma_tx->chan);

err0:
	return err;
}

static void sunxi_i2c_dma_release(struct sunxi_i2c *i2c)
{
	if (i2c->dma_tx) {
		i2c->dma_tx->dma_buf = 0;
		i2c->dma_tx->dma_len = 0;
		dma_release_channel(i2c->dma_tx->chan);
		i2c->dma_tx->chan = NULL;
	}

	if (i2c->dma_rx) {
		i2c->dma_rx->dma_buf = 0;
		i2c->dma_rx->dma_len = 0;
		dma_release_channel(i2c->dma_rx->chan);
		i2c->dma_rx->chan = NULL;
	}
}


static void sunxi_i2c_dma_callback(void *arg)
{
	struct sunxi_i2c *i2c = (struct sunxi_i2c *)arg;

	if (i2c->dma_using == i2c->dma_tx)
		dev_dbg(i2c->dev, "dma write data end\n");
	else if (i2c->dma_using == i2c->dma_rx)
		dev_dbg(i2c->dev, "dma read data end\n");
	dma_unmap_single(i2c->dma_using->chan->device->dev,
			i2c->dma_using->dma_buf,
			i2c->dma_using->dma_len, i2c->dma_using->dma_data_dir);
	complete(&i2c->cmd_complete);
}

static int i2c_sunxi_drv_complete(struct sunxi_i2c *i2c)
{
	unsigned long flags = 0;
	unsigned long timeout = 0;

	timeout = wait_event_timeout(i2c->wait, i2c->result, i2c->adap.timeout);
	if (timeout == 0) {
		dev_err(i2c->dev, "xfer timeout (dev addr:0x%x)\n", i2c->msg->addr);
		dump_reg(i2c, 0x200, 0x20);
		i2c_drv_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
					 | RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
		i2c_drv_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
		return -ETIME;
	} else if (i2c->result == RESULT_ERR) {
		dev_err(i2c->dev, "incomplete xfer, (status: 0x%x, dev addr: 0x%x)\n",
			i2c->msg_idx, i2c->msg->addr);
		dump_reg(i2c, 0x200, 0x20);
		i2c_drv_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
					 | RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
		i2c_drv_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
		return -ECOMM;
	}

	dev_dbg(i2c->dev, "xfer complete\n");

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->result = 0;
	spin_unlock_irqrestore(&i2c->lock, flags);

	return 0;
}

static int i2c_sunxi_dma_xfer(struct sunxi_i2c *i2c)
{
	unsigned long time_left;
	struct sunxi_i2c_dma *dma = i2c->dma_using;
	struct dma_async_tx_descriptor *dma_desc;
	struct device *chan_dev = dma->chan->device->dev;

	i2c->dma_buf = i2c_get_dma_safe_msg_buf(i2c->msg, 0);
	dma->dma_buf = dma_map_single(chan_dev, i2c->dma_buf,
					dma->dma_len, dma->dma_data_dir);
	if (dma_mapping_error(chan_dev, dma->dma_buf)) {
		dev_err(i2c->dev, "DMA mapping failed\n");
		goto err_map;
	}
	dma_desc = dmaengine_prep_slave_single(dma->chan, dma->dma_buf,
					       dma->dma_len, dma->dma_transfer_dir,
					       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dma_desc) {
		dev_err(i2c->dev, "Not able to get desc for DMA xfer\n");
		goto err_desc;
	}
	dma_desc->callback = sunxi_i2c_dma_callback;
	dma_desc->callback_param = i2c;
	if (dma_submit_error(dmaengine_submit(dma_desc))) {
		dev_err(i2c->dev, "DMA submit failed\n");
		goto err_submit;
	}

	reinit_completion(&i2c->cmd_complete);
	dma_async_issue_pending(dma->chan);
	dev_dbg(i2c->dev, "dma issue pending\n");

	time_left = wait_for_completion_timeout(
				&i2c->cmd_complete,
				msecs_to_jiffies(DMA_TIMEOUT));
	dev_dbg(i2c->dev, "time_left = %lu\n", time_left);

	i2c_put_dma_safe_msg_buf(i2c->dma_buf, i2c->msg, true);

	return 0;

err_submit:
err_desc:
	dma_unmap_single(chan_dev, dma->dma_buf,
			dma->dma_len, dma->dma_data_dir);
err_map:
	return -EINVAL;
}

static int sunxi_i2c_drv_write(struct sunxi_i2c *i2c, struct i2c_msg *msgs)
{
	i2c->msg = msgs;

	sunxi_i2c_drv_slave_addr(i2c, msgs);
	if (msgs->len == 1) {
		i2c_set_packet_addr_byte(0, i2c->base_addr);
		i2c_set_packet_data_byte(msgs->len, i2c->base_addr);
	} else {
		i2c_set_packet_addr_byte(1, i2c->base_addr);
		i2c_set_packet_data_byte(msgs->len - 1, i2c->base_addr);
	}
	i2c_set_packet_cnt(1, i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	i2c_sunxi_send_msgs(i2c, msgs);

	return 0;
}

static int sunxi_i2c_drv_dma_write(struct sunxi_i2c *i2c, struct i2c_msg *msgs)
{
	int ret = 0;

	i2c->msg = msgs;

	sunxi_i2c_drv_slave_addr(i2c, msgs);
	i2c_set_packet_addr_byte(1, i2c->base_addr);
	i2c_set_packet_data_byte(msgs->len - 1, i2c->base_addr);
	i2c_set_packet_cnt(1, i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_drv_enable_dma_irq(DMA_TX, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	i2c->dma_using = i2c->dma_tx;
	i2c->dma_using->dma_transfer_dir = DMA_MEM_TO_DEV;
	i2c->dma_using->dma_data_dir = DMA_TO_DEVICE;
	i2c->dma_using->dma_len = msgs->len;

	ret = i2c_sunxi_dma_xfer(i2c);

	return ret;
}

static int
sunxi_i2c_drv_mulpk_write(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	u16 i;

	i2c->msg = msgs;

	sunxi_i2c_drv_slave_addr(i2c, msgs);
	i2c_set_packet_addr_byte(1, i2c->base_addr);
	i2c_set_packet_data_byte(msgs->len - 1, i2c->base_addr);
	i2c_set_packet_interval(i2c->pkt_interval, i2c->base_addr);
	i2c_set_packet_cnt(num, i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	for (i = 0; i < num; i++)
		i2c_sunxi_send_msgs(i2c, msgs + i);

	return 0;
}

static int
sunxi_i2c_drv_dma_mulpk_write(struct sunxi_i2c *i2c, struct i2c_msg *msgs,
		int num)
{
	int ret = 0;
	unsigned int tlen = num * msgs->len;

	i2c->msg = msgs;

	sunxi_i2c_drv_slave_addr(i2c, msgs);
	i2c_set_packet_addr_byte(1, i2c->base_addr);
	i2c_set_packet_data_byte(msgs->len - 1, i2c->base_addr);
	i2c_set_packet_interval(i2c->pkt_interval, i2c->base_addr);
	i2c_set_packet_cnt(num, i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_drv_enable_dma_irq(DMA_TX, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	i2c->dma_using = i2c->dma_tx;
	i2c->dma_using->dma_transfer_dir = DMA_MEM_TO_DEV;
	i2c->dma_using->dma_data_dir = DMA_TO_DEVICE;
	i2c->dma_using->dma_len = tlen;

	ret = i2c_sunxi_dma_xfer(i2c);

	return ret;
}

static int
sunxi_i2c_drv_read(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	struct i2c_msg *wmsgs, *rmsgs;

	if (num == 1) {
		wmsgs = NULL;
		rmsgs = msgs;
	} else if (num == 2) {
		wmsgs = msgs;
		rmsgs = msgs + 1;
	} else {
		dev_err(i2c->dev, "can not support %d num!\n", num);
		return -EINVAL;
	}

	i2c->msg = rmsgs;

	sunxi_i2c_drv_slave_addr(i2c, rmsgs);
	i2c_set_packet_cnt(1, i2c->base_addr);
	i2c_set_packet_data_byte(rmsgs->len, i2c->base_addr);
	if (rmsgs->len > MAX_FIFO)
		i2c_set_rx_trig_level(MAX_FIFO, i2c->base_addr);
	else
		i2c_set_rx_trig_level(rmsgs->len, i2c->base_addr);
	if (i2c_query_rxfifo(i2c->base_addr))
		i2c_clear_rxfifo(i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	if (wmsgs)
		i2c_sunxi_send_msgs(i2c, wmsgs);

	return 0;
}

static int
sunxi_i2c_drv_dma_read(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	int ret = 0;
	struct i2c_msg *wmsgs, *rmsgs;

	if (num == 1) {
		wmsgs = NULL;
		rmsgs = msgs;
	} else if (num == 2) {
		wmsgs = msgs;
		rmsgs = msgs + 1;
	} else {
		dev_err(i2c->dev, "can not support %d num!\n", num);
		return -EINVAL;
	}

	i2c->msg = rmsgs;

	sunxi_i2c_drv_slave_addr(i2c, rmsgs);
	i2c_set_packet_data_byte(rmsgs->len, i2c->base_addr);
	i2c_set_packet_cnt(1, i2c->base_addr);
	i2c_set_rx_trig_level(MAX_FIFO/2, i2c->base_addr);
	if (i2c_query_rxfifo(i2c->base_addr))
		i2c_clear_rxfifo(i2c->base_addr);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_drv_enable_dma_irq(DMA_RX, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);
	if (wmsgs)
		i2c_sunxi_send_msgs(i2c, wmsgs);

	i2c->dma_using = i2c->dma_rx;
	i2c->dma_using->dma_transfer_dir = DMA_DEV_TO_MEM;
	i2c->dma_using->dma_data_dir = DMA_FROM_DEVICE;
	i2c->dma_using->dma_len = rmsgs->len;

	ret = i2c_sunxi_dma_xfer(i2c);

	return ret;
}

/**
 * sunxi_i2c_drv_do_xfer - twi driver transmission control
 * @i2c: struct of sunxi_i2c
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 */
static int
sunxi_i2c_drv_do_xfer(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	int ret;
	unsigned long flags = 0;

	spin_lock_irqsave(&i2c->lock, flags);
	i2c->result = 0;
	spin_unlock_irqrestore(&i2c->lock, flags);

	i2c_drv_clear_pending(i2c->base_addr);
	i2c_drv_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
			| RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
	i2c_drv_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
	if (i2c_query_txfifo(i2c->base_addr))
		i2c_clear_txfifo(i2c->base_addr);

	if (num == 1) {
		if (msgs->flags & I2C_M_RD) {
			/* 1 msgs read */
			i2c_enable_read_tran_mode(i2c->base_addr);
			i2c_set_packet_addr_byte(0, i2c->base_addr);

			if (i2c->dma_rx && (msgs->len >= DMA_THRESHOLD)) {
				dev_dbg(i2c->dev, "master dma read\n");
				ret =  sunxi_i2c_drv_dma_read(i2c, msgs, num);
			} else {
				dev_dbg(i2c->dev, "master cpu read\n");
				ret = sunxi_i2c_drv_read(i2c, msgs, num);
			}
		} else {
			/* 1 msgs write */
			i2c_disable_read_tran_mode(i2c->base_addr);

			if (i2c->dma_tx && (msgs->len >= DMA_THRESHOLD)) {
				dev_dbg(i2c->dev, "master dma write\n");
				ret =  sunxi_i2c_drv_dma_write(i2c, msgs);
			} else {
				dev_dbg(i2c->dev, "master cpu write\n");
				ret = sunxi_i2c_drv_write(i2c, msgs);
			}
		}
	} else if ((num == 2) && ((msgs + 1)->flags & I2C_M_RD)) {
		/* 2 msgs read */
		i2c_disable_read_tran_mode(i2c->base_addr);
		i2c_set_packet_addr_byte(msgs->len, i2c->base_addr);

		if (i2c->dma_rx && ((msgs + 1)->len >= DMA_THRESHOLD)) {
			dev_dbg(i2c->dev, "master dma read\n");
			ret =  sunxi_i2c_drv_dma_read(i2c, msgs, num);
		} else {
			dev_dbg(i2c->dev, "master cpu read\n");
			ret = sunxi_i2c_drv_read(i2c, msgs, num);
		}
	} else {
		/* multiple write with the same format packet */
		i2c_disable_read_tran_mode(i2c->base_addr);
		i2c_set_packet_addr_byte(1, i2c->base_addr);

		if (i2c->dma_tx && ((num * msgs->len) >= DMA_THRESHOLD)) {
			dev_dbg(i2c->dev, "master dma multiple packet write\n");
			ret = sunxi_i2c_drv_dma_mulpk_write(i2c, msgs, num);
		} else {
			dev_dbg(i2c->dev, "master cpu multiple packet write\n");
			ret = sunxi_i2c_drv_mulpk_write(i2c, msgs, num);
		}
	}
	if (ret)
		return ret;

	ret = i2c_sunxi_drv_complete(i2c);

	if (ret)
		return ret;
	else
		return num;
}


/*
 ****************************************************************************
 *
 *  FunctionName:           sunxi_i2c_addr_byte
 *
 *  Description:
 *         7bits addr: 7-1bits addr+0 bit r/w
 *         10bits addr: 1111_11xx_xxxx_xxxx-->1111_0xx_rw,xxxx_xxxx
 *         send the 7 bits addr,or the first part of 10 bits addr
 *  Parameters:
 *
 *
 *  Return value:
 *           ��
 *  Notes:
 *
 ****************************************************************************
 */
static void sunxi_i2c_addr_byte(struct sunxi_i2c *i2c)
{
	unsigned char addr = 0;
	unsigned char tmp  = 0;

	if (i2c->msg[i2c->msg_idx].flags & I2C_M_TEN) {
		/* 0111_10xx,ten bits address--9:8bits */
		tmp = 0x78 | (((i2c->msg[i2c->msg_idx].addr)>>8) & 0x03);
		addr = tmp << 1;	/*1111_0xx0*/
		/* how about the second part of ten bits addr? */
		/* Answer: deal at twi_core_process() */
	} else
		/* 7-1bits addr, xxxx_xxx0 */
		addr = (i2c->msg[i2c->msg_idx].addr & 0x7f) << 1;

	/* read, default value is write */
	if (i2c->msg[i2c->msg_idx].flags & I2C_M_RD)
		addr |= 1;

	if (i2c->msg[i2c->msg_idx].flags & I2C_M_TEN) {
		dev_dbg(i2c->dev, "first part of 10bits = 0x%x\n", addr);
	} else
		dev_dbg(i2c->dev, "7bits+r/w = 0x%x\n", addr);

	/* send 7bits+r/w or the first part of 10bits */
	twi_put_byte(i2c, &addr);
}


static int sunxi_i2c_core_process(struct sunxi_i2c *i2c)
{
	void __iomem *base_addr = i2c->base_addr;
	int  ret        = SUNXI_I2C_OK;
	int  err_code   = 0;
	unsigned char  state = 0;
	unsigned char  tmp   = 0;
	unsigned long flags = 0;

	state = twi_query_irq_status(base_addr);

	spin_lock_irqsave(&i2c->lock, flags);
	dev_dbg(i2c->dev, "[slave address : (0x%x), irq state : (0x%x)]\n",
		i2c->msg->addr, state);
	if (i2c->msg == NULL) {
		dev_err(i2c->dev, "i2c message is NULL, err_code = 0xfe\n");
		err_code = 0xfe;
		goto msg_null;
	}

	switch (state) {
	case 0xf8:
		/* On reset or stop the bus is idle, use only at poll method */
		err_code = 0xf8;
		goto err_out;
	case 0x08: /* A START condition has been transmitted */
	case 0x10: /* A repeated start condition has been transmitted */
		sunxi_i2c_addr_byte(i2c);/* send slave address */
		break;
	case 0xd8: /* second addr has transmitted, ACK not received!    */
	case 0x20: /* SLA+W has been transmitted; NOT ACK has been received */
		err_code = 0x20;
		goto err_out;
	case 0x18: /* SLA+W has been transmitted; ACK has been received */
		/* if any, send second part of 10 bits addr */
		if (i2c->msg[i2c->msg_idx].flags & I2C_M_TEN) {
			/* the remaining 8 bits of address */
			tmp = i2c->msg[i2c->msg_idx].addr & 0xff;
			twi_put_byte(i2c, &tmp); /* case 0xd0: */
			break;
		}
		/* for 7 bit addr, then directly send data byte--case 0xd0:  */
		/* fall through */
	case 0xd0: /* second addr has transmitted,ACK received!     */
	case 0x28: /* Data byte in DATA REG has been transmitted; */
		   /*  ACK has been received */
		/* after send register address then START send write data  */
		if (i2c->msg_ptr < i2c->msg[i2c->msg_idx].len) {
			twi_put_byte(i2c, &(i2c->msg[i2c->msg_idx].buf[i2c->msg_ptr]));
			i2c->msg_ptr++;
			break;
		}

		i2c->msg_idx++; /* the other msg */
		i2c->msg_ptr = 0;
		if (i2c->msg_idx == i2c->msg_num) {
			err_code = SUNXI_I2C_OK;/* Success,wakeup */
			goto ok_out;
		} else if (i2c->msg_idx < i2c->msg_num) {
			/* for restart pattern, read spec, two msgs */
			ret = twi_restart(i2c);
			if (ret == SUNXI_I2C_FAIL) {
				dev_err(i2c->dev, "twi_regulator error");
				err_code = SUNXI_I2C_SFAIL;
				goto err_out;/* START can't sendout */
			}
		} else {
			err_code = SUNXI_I2C_FAIL;
			goto err_out;
		}
		break;
	case 0x30: /* Data byte in I2CDAT has been transmitted; */
		   /* NOT ACK has been received */
		err_code = 0x30;	/*err,wakeup the thread*/
		goto err_out;
	case 0x38: /* Arbitration lost during SLA+W, SLA+R or data bytes */
		err_code = 0x38;	/*err,wakeup the thread*/
		goto err_out;
	case 0x40: /* SLA+R has been transmitted; ACK has been received */
		/* with Restart,needn't to send second part of 10 bits addr */
		/* refer-"I2C-SPEC v2.1" */
		/* enable A_ACK need it(receive data len) more than 1. */
		if (i2c->msg[i2c->msg_idx].len > 1) {
			/* send register addr complete,then enable the A_ACK */
			/* and get ready for receiving data */
			twi_enable_ack(base_addr);
			twi_clear_irq_flag(base_addr);/* jump to case 0x50 */
		} else if (i2c->msg[i2c->msg_idx].len == 1) {
			twi_clear_irq_flag(base_addr);/* jump to case 0x58 */
		}
		break;
	case 0x48: /* SLA+R has been transmitted; NOT ACK has been received */
		err_code = 0x48;	/*err,wakeup the thread*/
		goto err_out;
	case 0x50: /* Data bytes has been received; ACK has been transmitted */
		/* receive first data byte */
		if (i2c->msg_ptr < i2c->msg[i2c->msg_idx].len) {
			/* more than 2 bytes, the last byte need not to send ACK */
			if ((i2c->msg_ptr + 2) == i2c->msg[i2c->msg_idx].len)
				/* last byte no ACK */
				twi_disable_ack(base_addr);

			/* get data then clear flag,then next data coming */
			twi_get_byte(base_addr, &i2c->msg[i2c->msg_idx].buf[i2c->msg_ptr]);
			i2c->msg_ptr++;
			break;
		}
		/* err process, the last byte should be @case 0x58 */
		err_code = SUNXI_I2C_FAIL;/* err, wakeup */
		goto err_out;
	case 0x58:
		/* Data byte has been received; NOT ACK has been transmitted */
		/* received the last byte  */
		if (i2c->msg_ptr == i2c->msg[i2c->msg_idx].len - 1) {
			twi_get_last_byte(base_addr,
				&i2c->msg[i2c->msg_idx].buf[i2c->msg_ptr]);
			i2c->msg_idx++;
			i2c->msg_ptr = 0;
			if (i2c->msg_idx == i2c->msg_num) {
				/* succeed,wakeup the thread */
				err_code = SUNXI_I2C_OK;
				goto ok_out;
			} else if (i2c->msg_idx < i2c->msg_num) {
				/* repeat start */
				ret = twi_restart(i2c);
				if (ret == SUNXI_I2C_FAIL) {/* START fail */
					dev_err(i2c->dev, "twi_regulator error");
					err_code = SUNXI_I2C_SFAIL;
					goto err_out;
				}
				break;
			}
		} else {
			err_code = 0x58;
			goto err_out;
		}
		/* fall through */
	case 0x00: /* Bus error during master or slave mode due to illegal level condition */
		err_code = 0xff;
		goto err_out;
	default:
		err_code = state;
		goto err_out;
	}
	i2c->debug_state = state;/* just for debug */
	spin_unlock_irqrestore(&i2c->lock, flags);
	return ret;

ok_out:
err_out:
	if (twi_stop(i2c) == SUNXI_I2C_TFAIL)
		dev_err(i2c->dev, "STOP failed!\n");


msg_null:
	ret = sunxi_i2c_xfer_complete(i2c, err_code);/* wake up */
	i2c->debug_state = state;/* just for debug */
	spin_unlock_irqrestore(&i2c->lock, flags);
	return ret;
}

static irqreturn_t sunxi_i2c_handler(int this_irq, void *dev_id)
{
	struct sunxi_i2c *i2c = (struct sunxi_i2c *)dev_id;

	if (i2c->twi_drv_used)
		sunxi_i2c_drv_core_process(i2c);
	else {
		if (!twi_query_irq_flag(i2c->base_addr)) {
			dev_err(i2c->dev, "unknown interrupt!\n");
			return IRQ_NONE;
		}

		/* disable irq */
		twi_disable_irq(i2c->base_addr);

		/* twi core process */
		sunxi_i2c_core_process(i2c);

		/*
		 * enable irq only when twi is transferring,
		 * otherwise disable irq
		 */
		if (i2c->status != I2C_XFER_IDLE)
			twi_enable_irq(i2c->base_addr);
	}
	return IRQ_HANDLED;
}

static int sunxi_i2c_xfer_complete(struct sunxi_i2c *i2c, int code)
{
	int ret = SUNXI_I2C_OK;

	i2c->msg     = NULL;
	i2c->msg_num = 0;
	i2c->msg_ptr = 0;
	i2c->status  = I2C_XFER_IDLE;

	/* i2c->msg_idx  store the information */
	if (code == SUNXI_I2C_FAIL) {
		dev_err(i2c->dev, "Maybe Logic Error, debug it!\n");
		i2c->msg_idx = code;
		ret = SUNXI_I2C_FAIL;
		i2c->result = RESULT_ERR;
	} else if (code != SUNXI_I2C_OK) {
		i2c->msg_idx = code;
		ret = SUNXI_I2C_FAIL;
		i2c->result = RESULT_COMPLETE;
	}

	wake_up(&i2c->wait);

	return ret;
}

static int
sunxi_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct sunxi_i2c *i2c = (struct sunxi_i2c *)adap->algo_data;
	int ret = SUNXI_I2C_FAIL;
	unsigned char i = 0;
	u16 n, m;

	if (IS_ERR_OR_NULL(msgs) || (num <= 0)) {
		dev_err(i2c->dev, "invalid argument\n");
		return -EINVAL;
	}

	dev_dbg(i2c->dev, "current xfer msg num = %d\n", num);
	for (n = 0; n < num; n++) {
		dev_dbg(i2c->dev, "num: %d data is follow: ", n);
		if ((msgs + n)->buf) {
			for (m = 0; m < (msgs + n)->len; m++)
				dev_dbg(i2c->dev, "%02x ", *((msgs + n)->buf + m));
			dev_dbg(i2c->dev, "num: %d data is display end\n", n);
		} else {
			dev_dbg(i2c->dev, "null\n");
		}
	}

	ret = pm_runtime_get_sync(i2c->dev);
	if (ret < 0)
		goto out;
	if (i2c->twi_drv_used) {
		dev_dbg(i2c->dev, "twi driver xfer\n");
		ret = sunxi_i2c_drv_do_xfer(i2c, msgs, num);

	} else {
		dev_dbg(i2c->dev, "twi engine xfer\n");
		for (i = 1; i <= adap->retries; i++) {
			ret = sunxi_i2c_do_xfer(i2c, msgs, num);

			if (ret != SUNXI_I2C_RETRY)
				goto out;

			dev_dbg(i2c->dev, "Retrying transmission %d\n", i);
			udelay(100);
		}

		ret = -EREMOTEIO;
	}

out:
	pm_runtime_mark_last_busy(i2c->dev);
	pm_runtime_put_autosuspend(i2c->dev);

	return ret;
}

static int
sunxi_i2c_do_xfer(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	unsigned long timeout = 0;
	int ret = SUNXI_I2C_FAIL;
	unsigned long flags = 0;

	twi_soft_reset(i2c->base_addr, TWI_SRST_REG, TWI_SRST_SRST);
	udelay(100);

	/* test the bus is free,already protect by the semaphore at DEV layer */
	while (twi_query_irq_status(i2c->base_addr) != TWI_STAT_IDLE &&
		twi_query_irq_status(i2c->base_addr) != TWI_STAT_BUS_ERR &&
		twi_query_irq_status(i2c->base_addr) != TWI_STAT_ARBLOST_SLAR_ACK) {
		dev_dbg(i2c->dev, "bus is busy, status = %x\n",
				twi_query_irq_status(i2c->base_addr));
		if (twi_send_clk_9pulse(i2c) != SUNXI_I2C_OK) {
			ret = SUNXI_I2C_RETRY;
			goto out;
		} else
			break;
	}

	/* may conflict with xfer_complete */
	spin_lock_irqsave(&i2c->lock, flags);
	i2c->msg     = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->status  = I2C_XFER_START;
	twi_enable_irq(i2c->base_addr);  /* enable irq */
	twi_disable_ack(i2c->base_addr); /* disabe ACK */
	/* set the special function register,default:0. */
	twi_set_efr(i2c->base_addr, 0);
	spin_unlock_irqrestore(&i2c->lock, flags);

	/* START signal, needn't clear int flag */
	ret = twi_start(i2c);
	if (ret == SUNXI_I2C_FAIL) {
		dev_err(i2c->dev, "twi_regulator error");
		twi_soft_reset(i2c->base_addr, TWI_SRST_REG, TWI_SRST_SRST);
		twi_disable_irq(i2c->base_addr);  /* disable irq */
		i2c->status  = I2C_XFER_IDLE;
		ret = SUNXI_I2C_RETRY;
		goto out;
	}

	i2c->status  = I2C_XFER_RUNNING;
	/* sleep and wait,do the transfer at interrupt handler,timeout = 5*HZ */
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0,
						i2c->adap.timeout);
	/* return code,if(msg_idx == num) succeed */
	ret = i2c->msg_idx;
	if (timeout == 0) {
		dev_err(i2c->dev, "xfer timeout (dev addr:0x%x)\n", msgs->addr);
		spin_lock_irqsave(&i2c->lock, flags);
		i2c->msg = NULL;
		spin_unlock_irqrestore(&i2c->lock, flags);
		ret = -ETIME;
	} else if (ret != num) {
		dev_err(i2c->dev, "incomplete xfer (status: 0x%x, dev addr: 0x%x)\n",
				ret, msgs->addr);
		ret = -ECOMM;
	}
out:
	return ret;
}

static unsigned int sunxi_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C|I2C_FUNC_10BIT_ADDR|I2C_FUNC_SMBUS_EMUL;
}


static const struct i2c_algorithm sunxi_i2c_algorithm = {
	.master_xfer	  = sunxi_i2c_xfer,
	.functionality	  = sunxi_i2c_functionality,
};

static int sunxi_i2c_regulator_request(struct sunxi_i2c *i2c)
{
	if (i2c->regulator)
		return 0;

	i2c->regulator = regulator_get(i2c->dev, "twi");
	if (IS_ERR(i2c->regulator)) {
		dev_err(i2c->dev, "get supply failed!\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int sunxi_i2c_regulator_release(struct sunxi_i2c *i2c)
{
	if (!i2c->regulator)
		return 0;

	regulator_put(i2c->regulator);

	i2c->regulator = NULL;

	return 0;
}

static int sunxi_i2c_regulator_enable(struct sunxi_i2c *i2c)
{
	if (!i2c->regulator)
		return 0;

	if (regulator_enable(i2c->regulator)) {
		dev_err(i2c->dev, "enable regulator failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_i2c_regulator_disable(struct sunxi_i2c *i2c)
{
	if (!i2c->regulator)
		return 0;

	if (regulator_is_enabled(i2c->regulator))
		regulator_disable(i2c->regulator);

	return 0;
}

static int sunxi_i2c_clk_request(struct sunxi_i2c *i2c)
{
	i2c->bus_clk = devm_clk_get(i2c->dev, "bus");
	if (IS_ERR(i2c->bus_clk)) {
		dev_err(i2c->dev, "request TWI clock failed\n");
		return -EINVAL;
	}

	i2c->reset = devm_reset_control_get(i2c->dev, NULL);
	if (IS_ERR(i2c->reset)) {
		dev_err(i2c->dev, "request TWI reset failed\n");
		return -EINVAL;
	}
	return 0;
}

static int sunxi_i2c_resource_get(struct device_node *np, struct sunxi_i2c *i2c)
{
	int err;

	i2c->bus_num = of_alias_get_id(np, "twi");
	if (i2c->bus_num < 0) {
		dev_err(i2c->dev, "I2C failed to get alias id\n");
		return -EINVAL;
	}
	i2c->pdev->id = i2c->bus_num;

	dev_set_name(i2c->dev, SUNXI_I2C_ID_FORMAT, i2c->bus_num);

	i2c->res = platform_get_resource(i2c->pdev, IORESOURCE_MEM, 0);
	if (!i2c->res) {
		dev_err(i2c->dev, "failed to get MEM res\n");
		return -ENXIO;
	}
	i2c->base_addr = devm_ioremap_resource(&i2c->pdev->dev, i2c->res);
	if (IS_ERR_OR_NULL(i2c->base_addr)) {
		dev_err(i2c->dev, "unable to ioremap\n");
		return PTR_RET(i2c->base_addr);
	}

	err = sunxi_i2c_clk_request(i2c);
	if (err) {
		dev_err(i2c->dev, "request i2c clk failed!\n");
		return err;
	}

	i2c->pctrl = devm_pinctrl_get(i2c->dev);
	if (IS_ERR(i2c->pctrl)) {
		dev_err(i2c->dev, "pinctrl_get failed\n");
		return PTR_ERR(i2c->pctrl);
	}

	i2c->no_suspend = 0;
	of_property_read_u32(np, "no_suspend", &i2c->no_suspend);
	i2c->irq_flag = (i2c->no_suspend) ? IRQF_COND_SUSPEND : 0;

	i2c->irq = platform_get_irq(i2c->pdev, 0);
	if (i2c->irq < 0) {
		dev_err(i2c->dev, "failed to get irq\n");
		return i2c->irq;
	}
	err = devm_request_irq(&i2c->pdev->dev, i2c->irq, sunxi_i2c_handler,
			i2c->irq_flag, dev_name(i2c->dev), i2c);
	if (err) {
		dev_err(i2c->dev, "request irq failed!\n");
		return err;
	}

	err = sunxi_i2c_regulator_request(i2c);
	if (err) {
		dev_err(i2c->dev, "request regulator failed!\n");
		return err;
	}

	err = of_property_read_u32(np, "clock-frequency", &i2c->bus_freq);
	if (err) {
		dev_err(i2c->dev, "failed to get clock frequency\n");
		return err;
	}

	i2c->pkt_interval = 0;
	of_property_read_u32(np, "twi_pkt_interval", &i2c->pkt_interval);
	i2c->twi_drv_used = 0;
	of_property_read_u32(np, "twi_drv_used", &i2c->twi_drv_used);

	if (i2c->twi_drv_used) {
		err = sunxi_i2c_dma_request(i2c, (dma_addr_t)i2c->res->start);
		if (err) {
			dev_err(i2c->dev, "dma request failed\n");
			goto err0;
		}
	}

	return 0;

err0:
	sunxi_i2c_regulator_release(i2c);
	return err;
}

static void sunxi_i2c_resource_put(struct sunxi_i2c *i2c)
{
	if (i2c->twi_drv_used)
		sunxi_i2c_dma_release(i2c);

	sunxi_i2c_regulator_release(i2c);
}

static int sunxi_i2c_select_pin_state(struct sunxi_i2c *i2c, char *name)
{
	int ret;
	struct pinctrl_state *pctrl_state = NULL;

	pctrl_state = pinctrl_lookup_state(i2c->pctrl, name);
	if (IS_ERR(pctrl_state)) {
		dev_err(i2c->dev, "pinctrl_lookup_state(%s) failed! return %p\n",
			name, pctrl_state);
		return -EINVAL;
	}

	ret = pinctrl_select_state(i2c->pctrl, pctrl_state);
	if (ret < 0)
		dev_err(i2c->dev, "pinctrl select state(%s) failed! return %d\n",
				name, ret);

	return ret;
}

static int sunxi_i2c_clk_init(struct sunxi_i2c *i2c)
{
	unsigned long clk_rate;
	int err;

	if (reset_control_deassert(i2c->reset)) {
		dev_err(i2c->dev, "reset control deassert  failed!\n");
		return -EINVAL;
	}

	if (clk_prepare_enable(i2c->bus_clk)) {
		dev_err(i2c->dev, "enable apb_twi clock failed!\n");
		err = -EINVAL;
		goto err0;
	}

	/* set twi module clock */
	clk_rate = clk_get_rate(i2c->bus_clk);
	if (clk_rate == 0) {
		dev_err(i2c->dev, "get source clock frequency failed!\n");
		err = -EINVAL;
		goto err1;
	}

	/* i2c ctroller module clock is controllerd by self */
	if (i2c->twi_drv_used) {
		sunxi_i2c_set_clock(i2c, TWI_DRIVER_BUSC, 24000000, i2c->bus_freq,
				TWI_DRV_CLK_M, TWI_DRV_CLK_N);
		dev_dbg(i2c->dev, "set twi driver clock\n");
	} else {
#if !IS_ENABLED(CONFIG_EVB_PLATFORM)
		clk_rate = 24000000;
#endif
		sunxi_i2c_set_clock(i2c, TWI_CLK_REG, clk_rate, i2c->bus_freq,
				TWI_CLK_DIV_M, TWI_CLK_DIV_N);
		dev_dbg(i2c->dev, "set twi engine clock\n");
	}

	return 0;

err1:
	clk_disable_unprepare(i2c->bus_clk);

err0:
	reset_control_assert(i2c->reset);
	return err;
}

static void sunxi_i2c_clk_exit(struct sunxi_i2c *i2c)
{
	/* disable clk */
	if (!IS_ERR_OR_NULL(i2c->bus_clk) && __clk_is_enabled(i2c->bus_clk)) {
		clk_disable_unprepare(i2c->bus_clk);
	}

	if (!IS_ERR_OR_NULL(i2c->reset)) {
		reset_control_assert(i2c->reset);
	}
}

static int sunxi_i2c_hw_init(struct sunxi_i2c *i2c)
{
	int err;

	err = sunxi_i2c_regulator_enable(i2c);
	if (err) {
		dev_err(i2c->dev, "enable regulator failed!\n");
		goto err0;
	}

	err = sunxi_i2c_select_pin_state(i2c, PINCTRL_STATE_DEFAULT);
	if (err) {
		dev_err(i2c->dev, "request i2c gpio failed!\n");
		goto err1;
	}

	err = sunxi_i2c_clk_init(i2c);
	if (err) {
		dev_err(i2c->dev, "init i2c clock failed!\n");
		goto err2;
	}

	sunxi_i2c_bus_enable(i2c);

	if (!i2c->twi_drv_used)
		twi_soft_reset(i2c->base_addr, TWI_SRST_REG, TWI_SRST_SRST);

	return 0;

err2:
	sunxi_i2c_select_pin_state(i2c, PINCTRL_STATE_SLEEP);

err1:
	sunxi_i2c_regulator_disable(i2c);

err0:
	return err;
}

static void sunxi_i2c_hw_exit(struct sunxi_i2c *i2c)
{
	sunxi_i2c_bus_disable(i2c);

	sunxi_i2c_clk_exit(i2c);

	sunxi_i2c_select_pin_state(i2c, PINCTRL_STATE_SLEEP);

	sunxi_i2c_regulator_disable(i2c);
}

static ssize_t sunxi_i2c_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_i2c *i2c = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,
			"i2c->bus_num = %d\n"
			"i2c->name = %s\n"
			"i2c->irq = %d\n"
			"i2c->freqency = %u\n",
			i2c->bus_num,
			dev_name(i2c->dev),
			i2c->irq,
			i2c->bus_freq);
}
static struct device_attribute sunxi_i2c_info_attr =
	__ATTR(info, S_IRUGO, sunxi_i2c_info_show, NULL);

static ssize_t sunxi_i2c_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_i2c *i2c = dev_get_drvdata(dev);
	static char const *i2c_status[] = {"Unknown", "Idle", "Start",
						"Unknown", "Running"};

	if (i2c == NULL)
		return snprintf(buf, PAGE_SIZE, "%s\n", "sunxi_i2c is NULL!");

	return snprintf(buf, PAGE_SIZE,
			"i2c->bus_num = %d\n"
			"i2c->status  = [%u] %s\n"
			"i2c->msg_num   = %u, ->msg_idx = %u, ->msg_ptr = %u\n"
			"i2c->bus_freq  = %u\n"
			"i2c->irq       = %d\n"
			"i2c->debug_state = %u\n"
			"i2c->base_addr = 0x%p, the TWI control register:\n"
			"[ADDR] 0x%02x = 0x%08x, [XADDR] 0x%02x = 0x%08x\n"
			"[DATA] 0x%02x = 0x%08x, [CNTR] 0x%02x = 0x%08x\n"
			"[STAT]  0x%02x = 0x%08x, [CCR]  0x%02x = 0x%08x\n"
			"[SRST] 0x%02x = 0x%08x, [EFR]   0x%02x = 0x%08x\n"
			"[LCR]  0x%02x = 0x%08x\n",
			i2c->bus_num, i2c->status, i2c_status[i2c->status],
			i2c->msg_num, i2c->msg_idx, i2c->msg_ptr,
			i2c->bus_freq, i2c->irq, i2c->debug_state,
			i2c->base_addr,
			TWI_ADDR_REG,  readl(i2c->base_addr + TWI_ADDR_REG),
			TWI_XADDR_REG, readl(i2c->base_addr + TWI_XADDR_REG),
			TWI_DATA_REG,  readl(i2c->base_addr + TWI_DATA_REG),
			TWI_CTL_REG,   readl(i2c->base_addr + TWI_CTL_REG),
			TWI_STAT_REG,  readl(i2c->base_addr + TWI_STAT_REG),
			TWI_CLK_REG,   readl(i2c->base_addr + TWI_CLK_REG),
			TWI_SRST_REG,  readl(i2c->base_addr + TWI_SRST_REG),
			TWI_EFR_REG,   readl(i2c->base_addr + TWI_EFR_REG),
			TWI_LCR_REG,   readl(i2c->base_addr + TWI_LCR_REG));
}

static struct device_attribute sunxi_i2c_status_attr =
	__ATTR(status, S_IRUGO, sunxi_i2c_status_show, NULL);

static void sunxi_i2c_create_sysfs(struct platform_device *_pdev)
{
	device_create_file(&_pdev->dev, &sunxi_i2c_info_attr);
	device_create_file(&_pdev->dev, &sunxi_i2c_status_attr);
}

static void sunxi_i2c_remove_sysfs(struct platform_device *_pdev)
{
	device_remove_file(&_pdev->dev, &sunxi_i2c_info_attr);
	device_remove_file(&_pdev->dev, &sunxi_i2c_status_attr);
}

static int sunxi_i2c_probe(struct platform_device *pdev)
{
	struct sunxi_i2c *i2c = NULL;
	int err;

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct sunxi_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(i2c->dev, "I2C failed to kzlloc sunxi_i2c struct\n");
		return -ENOMEM;
	}

	i2c->pdev = pdev;
	i2c->dev = &pdev->dev;
	pdev->name = dev_name(i2c->dev);

	/* get dts resource */
	err = sunxi_i2c_resource_get(pdev->dev.of_node, i2c);
	if (err) {
		dev_err(i2c->dev, "I2C failed to get resource\n");
		goto err0;
	}

	/* i2c controller hardware init */
	err = sunxi_i2c_hw_init(i2c);
	if (err) {
		dev_err(i2c->dev, "hw init failed! try again!!\n");
		goto err1;
	}

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	sunxi_i2c_create_sysfs(pdev);

	pm_runtime_set_active(i2c->dev);
	if (i2c->no_suspend) {
		pm_runtime_get_noresume(i2c->dev);
	}
	pm_runtime_set_autosuspend_delay(i2c->dev, AUTOSUSPEND_TIMEOUT);
	pm_runtime_use_autosuspend(i2c->dev);
	pm_runtime_enable(i2c->dev);

	snprintf(i2c->adap.name, sizeof(i2c->adap.name), dev_name(&pdev->dev));
	i2c->status = I2C_XFER_IDLE;
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.nr = i2c->bus_num;
	i2c->adap.retries = 3;
	i2c->adap.timeout = 5*HZ;
	i2c->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->adap.algo = &sunxi_i2c_algorithm;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, i2c);

	/* register i2c adapter should be the ending of probe
	 * before register all the resouce i2c controller need be ready
	 * (i2c_xfer may occur at any time when register)
	 * */
	err = i2c_add_numbered_adapter(&i2c->adap);
	if (err) {
		dev_err(i2c->dev, "failed to add adapter\n");
		goto err2;
	}

	dev_info(i2c->dev, "probe success\n");
	return 0;

err2:
	pm_runtime_set_suspended(i2c->dev);
	pm_runtime_disable(i2c->dev);
	sunxi_i2c_remove_sysfs(pdev);
	sunxi_i2c_hw_exit(i2c);

err1:
	sunxi_i2c_resource_put(i2c);

err0:
	return err;
}

static int sunxi_i2c_remove(struct platform_device *pdev)
{
	struct sunxi_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);

	platform_set_drvdata(pdev, NULL);

	pm_runtime_set_suspended(i2c->dev);
	pm_runtime_disable(i2c->dev);
	sunxi_i2c_remove_sysfs(pdev);
	sunxi_i2c_hw_exit(i2c);

	sunxi_i2c_resource_put(i2c);

	dev_dbg(i2c->dev, "remove\n");

	return 0;
}

#if IS_ENABLED(CONFIG_PM)
static void sunxi_i2c_save_register(struct sunxi_i2c *i2c)
{
	if (i2c->twi_drv_used) {
		i2c->reg2[0] = readl(i2c->base_addr + TWI_DRIVER_CTRL);
		i2c->reg2[1] = readl(i2c->base_addr + TWI_DRIVER_CFG);
		i2c->reg2[2] = readl(i2c->base_addr + TWI_DRIVER_SLV);
		i2c->reg2[3] = readl(i2c->base_addr + TWI_DRIVER_FMT);
		i2c->reg2[4] = readl(i2c->base_addr + TWI_DRIVER_BUSC);
		i2c->reg2[5] = readl(i2c->base_addr + TWI_DRIVER_INTC);
		i2c->reg2[6] = readl(i2c->base_addr + TWI_DRIVER_DMAC);
		i2c->reg2[7] = readl(i2c->base_addr + TWI_DRIVER_FIFOC);
	} else {
		i2c->reg1[0] = readl(i2c->base_addr + TWI_CTL_REG);
		i2c->reg1[1] = readl(i2c->base_addr + TWI_EFR_REG);
		i2c->reg1[2] = readl(i2c->base_addr + TWI_CLK_REG);
		i2c->reg1[3] = readl(i2c->base_addr + TWI_LCR_REG);
	}
}

static void sunxi_i2c_restore_register(struct sunxi_i2c *i2c)
{
	if (i2c->twi_drv_used) {
		writel(i2c->reg2[0], i2c->base_addr + TWI_DRIVER_CTRL);
		writel(i2c->reg2[1], i2c->base_addr + TWI_DRIVER_CFG);
		writel(i2c->reg2[2], i2c->base_addr + TWI_DRIVER_SLV);
		writel(i2c->reg2[3], i2c->base_addr + TWI_DRIVER_FMT);
		writel(i2c->reg2[4], i2c->base_addr + TWI_DRIVER_BUSC);
		writel(i2c->reg2[5], i2c->base_addr + TWI_DRIVER_INTC);
		writel(i2c->reg2[6], i2c->base_addr + TWI_DRIVER_DMAC);
		writel(i2c->reg2[7], i2c->base_addr + TWI_DRIVER_FIFOC);
	} else {
		writel(i2c->reg1[0], i2c->base_addr + TWI_CTL_REG);
		writel(i2c->reg1[1], i2c->base_addr + TWI_EFR_REG);
		writel(i2c->reg1[2], i2c->base_addr + TWI_CLK_REG);
		writel(i2c->reg1[3], i2c->base_addr + TWI_LCR_REG);
	}
}

static int sunxi_i2c_runtime_suspend(struct device *dev)
{
	struct sunxi_i2c *i2c = dev_get_drvdata(dev);

	sunxi_i2c_save_register(i2c);

	sunxi_i2c_hw_exit(i2c);

	dev_dbg(i2c->dev, "runtime suspend finish\n");

	return 0;
}

static int sunxi_i2c_runtime_resume(struct device *dev)
{
	struct sunxi_i2c *i2c = dev_get_drvdata(dev);

	sunxi_i2c_hw_init(i2c);

	sunxi_i2c_restore_register(i2c);

	dev_dbg(i2c->dev, "runtime resume finish\n");

	return 0;
}

static int sunxi_i2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_i2c *i2c = platform_get_drvdata(pdev);

	sunxi_i2c_bus_disable(i2c);

	if (i2c->no_suspend) {
		dev_dbg(i2c->dev, "doesn't need to  suspend\n");
		return 0;
	}

	return pm_runtime_force_suspend(dev);

}

static int sunxi_i2c_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_i2c *i2c = platform_get_drvdata(pdev);

	if (i2c->twi_drv_used) {
		sunxi_i2c_set_clock(i2c, TWI_DRIVER_BUSC, 24000000, i2c->bus_freq,
				TWI_DRV_CLK_M, TWI_DRV_CLK_N);
	}

	sunxi_i2c_bus_enable(i2c);

	if (i2c->no_suspend) {
		dev_dbg(i2c->dev, "doesn't need to resume\n");
		return 0;
	}

	return pm_runtime_force_resume(dev);

}

static const struct dev_pm_ops sunxi_i2c_dev_pm_ops = {
	.suspend_noirq	 = sunxi_i2c_suspend_noirq,
	.resume_noirq	 = sunxi_i2c_resume_noirq,
	.runtime_suspend = sunxi_i2c_runtime_suspend,
	.runtime_resume  = sunxi_i2c_runtime_resume,
};

#define SUNXI_I2C_DEV_PM_OPS (&sunxi_i2c_dev_pm_ops)
#else
#define SUNXI_I2C_DEV_PM_OPS NULL
#endif

static const struct of_device_id sunxi_i2c_match[] = {
	{ .compatible = "allwinner,sun8i-twi", },
	{ .compatible = "allwinner,sun20i-twi", },
	{ .compatible = "allwinner,sun50i-twi", },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_i2c_match);

static struct platform_driver sunxi_i2c_driver = {
	.probe		= sunxi_i2c_probe,
	.remove		= sunxi_i2c_remove,
	.driver		= {
		.name	= SUNXI_I2C_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm		= SUNXI_I2C_DEV_PM_OPS,
		.of_match_table = sunxi_i2c_match,
	},
};

static int __init sunxi_i2c_adap_init(void)
{
	return platform_driver_register(&sunxi_i2c_driver);
}

static void __exit sunxi_i2c_adap_exit(void)
{
	platform_driver_unregister(&sunxi_i2c_driver);
}

subsys_initcall_sync(sunxi_i2c_adap_init);

module_exit(sunxi_i2c_adap_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-sunxi");
MODULE_DESCRIPTION("SUNXI I2C Bus Driver");
MODULE_AUTHOR("pannan");
