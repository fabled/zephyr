/*
 * Copyright (c) 2019 Crypta Labs Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_DOMAIN "GPSPI CEC"
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_cec);

#include <errno.h>
#include <spi.h>
#include <soc.h>

#include "spi_context.h"

typedef void (*irq_config_func_t)(struct device *port);

struct gpspi_data {
	struct spi_context ctx;
};

struct gpspi_config {
	GP_SPI0_INST_Type *spi;
};

static inline struct gpspi_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline const struct gpspi_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
}

static int gpspi_configure(struct device *dev, const struct spi_config *spi_cfg)
{
	const struct gpspi_config *cfg = get_dev_config(dev);
	struct gpspi_data *data = get_dev_data(dev);
	GP_SPI0_INST_Type *spi = cfg->spi;

	if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_SLAVE ||
	    (spi_cfg->operation & SPI_TRANSFER_LSB) ||
	    (spi_cfg->frequency == 0)) {
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8) {
		return -ENOTSUP;
	}

	if ((spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -ENOTSUP;
	}

	if (spi_cfg->frequency < 500000) {
		spi->CLOCK_GENERATOR = 2000000 / 2 / spi_cfg->frequency;
		spi->CLOCK_Control_b.CLKSRC = 1;
	} else {
		spi->CLOCK_GENERATOR = SYSCLK_DEFAULT_IOSC_HZ / 2 / spi_cfg->frequency;
		spi->CLOCK_Control_b.CLKSRC = 0;
	}
	spi->ENABLE = 1;

	spi->CLOCK_Control_b.CLKPOL = SPI_MODE_GET(spi_cfg->operation) ==  SPI_MODE_CPOL;
	spi->CLOCK_Control_b.RCLKPH =
		SPI_MODE_GET(spi_cfg->operation) == SPI_MODE_CPHA;
	spi->CLOCK_Control_b.TCLKPH =
		SPI_MODE_GET(spi_cfg->operation) == SPI_MODE_CPHA;

	/* At this point, it's mandatory to set this on the context! */
	data->ctx.config = spi_cfg;

	spi_context_cs_configure(&data->ctx);

	LOG_DBG("Installed config %p: freq %uHz, mode %u/%u/%u, slave %u",
		spi_cfg, spi_cfg->frequency,
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) ? 1 : 0,
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) ? 1 : 0,
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_LOOP) ? 1 : 0,
		spi_cfg->slave);

	return 0;
}

static void gpspi_complete(struct gpspi_data *data, GP_SPI0_INST_Type *spi, int status)
{
	spi_context_cs_control(&data->ctx, false);
	spi->CONTROL_b.CE = 0;

	if (status) {
		spi->CONTROL_b.SOFT_RESET = 1;
		while (spi->CONTROL_b.SOFT_RESET)
			;
	}
}

static int gpspi_shift(GP_SPI0_INST_Type *spi, struct gpspi_data *data)
{
	u8_t byte = 0;

	if (spi_context_tx_buf_on(&data->ctx)) {
		byte = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
	}
	spi_context_update_tx(&data->ctx, 1, 1);
	while (!spi->STATUS_b.TXBE)
		;
	sys_write8(byte, (mem_addr_t)&spi->TX_DATA);

	while (!spi->STATUS_b.RXBF)
		;

	byte = sys_read8((mem_addr_t)&spi->RX_DATA);
	if (spi_context_rx_buf_on(&data->ctx)) {
		UNALIGNED_PUT(byte, (u8_t *)data->ctx.rx_buf);
	}
	spi_context_update_rx(&data->ctx, 1, 1);

	return 0;
}

static int transceive(struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      struct k_poll_signal *async)
{
	const struct gpspi_config *cfg = get_dev_config(dev);
	struct gpspi_data *data = get_dev_data(dev);
	GP_SPI0_INST_Type *spi = cfg->spi;
	int ret;

	spi_context_lock(&data->ctx, async != NULL, async);

	ret = gpspi_configure(dev, spi_cfg);
	if (ret) {
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);
	spi->CONTROL_b.CE = 1;

	do {
		ret = gpspi_shift(spi, data);
	} while (!ret &&
		 (spi_context_tx_on(&data->ctx) ||
		  spi_context_rx_on(&data->ctx)));

	while (!spi->STATUS_b.TXBE)
		;

	gpspi_complete(data, spi, ret);

	spi_context_release(&data->ctx, ret);

	if (ret) {
		LOG_ERR("error mask 0x%x", ret);
	}

	return ret ? -EIO : 0;
}

static int gpspi_transceive(struct device *dev,
				const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, NULL);
}

#ifdef CONFIG_POLL
static int gpspi_transceive_async(struct device *dev,
				  const struct spi_config *spi_cfg,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs,
				  struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif /* CONFIG_POLL */

static int gpspi_release(struct device *dev, const struct spi_config *config)
{
	const struct gpspi_config *cfg = get_dev_config(dev);
	struct gpspi_data *data = get_dev_data(dev);
	GP_SPI0_INST_Type *spi = cfg->spi;

	spi_context_unlock_unconditionally(&data->ctx);
	spi->ENABLE = 0;

	return 0;
}

static const struct spi_driver_api api_funcs = {
	.transceive = gpspi_transceive,
#ifdef CONFIG_POLL
	.transceive_async = gpspi_transceive_async,
#endif
	.release = gpspi_release,
};

static int gpspi_init(struct device *dev)
{
	const struct gpspi_config *cfg = dev->config->config_info;
	struct gpspi_data *data = dev->driver_data;
	GP_SPI0_INST_Type *spi = cfg->spi;

	spi_context_unlock_unconditionally(&data->ctx);

	/* Reset block */
	spi->ENABLE = 1;
	spi->CONTROL_b.SOFT_RESET = 1;
	while (spi->CONTROL_b.SOFT_RESET)
		;
	spi->ENABLE = 0;

	return 0;
}

#ifdef DT_INST_0_MICROCHIP_CEC_GPSPI

static const struct gpspi_config gpspi_cfg_0 = {
	.spi = (GP_SPI0_INST_Type *) DT_INST_0_MICROCHIP_CEC_GPSPI_BASE_ADDRESS,
};

static struct gpspi_data gpspi_dev_data_0 = {
	SPI_CONTEXT_INIT_LOCK(gpspi_dev_data_0, ctx),
	SPI_CONTEXT_INIT_SYNC(gpspi_dev_data_0, ctx),
};

DEVICE_AND_API_INIT(gpspi_0, DT_INST_0_MICROCHIP_CEC_GPSPI_LABEL,
		    &gpspi_init, &gpspi_dev_data_0, &gpspi_cfg_0,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#endif
