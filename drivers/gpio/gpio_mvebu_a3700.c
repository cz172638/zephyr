/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO of the Marvell Armada 3700 north and south bridges (Linux
 * pinctrl-armada-37xx.c): per bank of 32 pins an output enable register
 * at 0x00, the input values at 0x10 and the output values at 0x18. The
 * second north bridge bank (pins 32..35) is a node at the base + 4.
 * Pin muxing stays with the pin controller; no interrupts yet.
 */

#define DT_DRV_COMPAT marvell_armada_3700_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>

#define GPIO_OUTPUT_EN  0x00
#define GPIO_INPUT_VAL  0x10
#define GPIO_OUTPUT_VAL 0x18

struct a3700_gpio_config {
	struct gpio_driver_config common;
	DEVICE_MMIO_ROM;
};

struct a3700_gpio_data {
	struct gpio_driver_data common;
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
};

static inline uint32_t rd(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static inline void wr(const struct device *dev, uint32_t reg, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + reg);
}

static void rmw(const struct device *dev, uint32_t reg, uint32_t mask, uint32_t val)
{
	struct a3700_gpio_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	wr(dev, reg, (rd(dev, reg) & ~mask) | (val & mask));
	k_spin_unlock(&data->lock, key);
}

static int a3700_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	uint32_t bit = BIT(pin);

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_SINGLE_ENDED)) != 0) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			rmw(dev, GPIO_OUTPUT_VAL, bit, bit);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			rmw(dev, GPIO_OUTPUT_VAL, bit, 0);
		}
		rmw(dev, GPIO_OUTPUT_EN, bit, bit);
	} else if ((flags & GPIO_INPUT) != 0) {
		rmw(dev, GPIO_OUTPUT_EN, bit, 0);
	} else {
		/* Disconnected: the closest is an input */
		rmw(dev, GPIO_OUTPUT_EN, bit, 0);
	}

	return 0;
}

static int a3700_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	*value = rd(dev, GPIO_INPUT_VAL);
	return 0;
}

static int a3700_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	rmw(dev, GPIO_OUTPUT_VAL, mask, value);
	return 0;
}

static int a3700_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	rmw(dev, GPIO_OUTPUT_VAL, pins, pins);
	return 0;
}

static int a3700_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	rmw(dev, GPIO_OUTPUT_VAL, pins, 0);
	return 0;
}

static int a3700_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	struct a3700_gpio_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	wr(dev, GPIO_OUTPUT_VAL, rd(dev, GPIO_OUTPUT_VAL) ^ pins);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int a3700_gpio_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static DEVICE_API(gpio, a3700_gpio_api) = {
	.pin_configure = a3700_gpio_pin_configure,
	.port_get_raw = a3700_gpio_port_get_raw,
	.port_set_masked_raw = a3700_gpio_port_set_masked_raw,
	.port_set_bits_raw = a3700_gpio_port_set_bits_raw,
	.port_clear_bits_raw = a3700_gpio_port_clear_bits_raw,
	.port_toggle_bits = a3700_gpio_port_toggle_bits,
};

#define A3700_GPIO_INIT(n)                                                                         \
	static const struct a3700_gpio_config a3700_gpio_config_##n = {                            \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n)},                   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
	};                                                                                         \
	static struct a3700_gpio_data a3700_gpio_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, a3700_gpio_init, NULL, &a3700_gpio_data_##n,                      \
			      &a3700_gpio_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &a3700_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(A3700_GPIO_INIT)
