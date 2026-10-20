/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 37xx pin group selection bits (north/south bridge "selection"
 * register at +0x30 of the pinctrl block). For most groups a cleared bit
 * selects the peripheral function and a set bit selects GPIO.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ARMADA37XX_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ARMADA37XX_PINCTRL_H_

#define A37XX_NB 0
#define A37XX_SB 1

/* north bridge */
#define A37XX_NB_JTAG     (1 << 0)
#define A37XX_NB_SDIO0    (1 << 1)
#define A37XX_NB_EMMC     (1 << 2)
#define A37XX_NB_I2C2     (1 << 9)
#define A37XX_NB_I2C1     (1 << 10)
#define A37XX_NB_SPI_CS1  (1 << 12)
#define A37XX_NB_ONEWIRE  (1 << 16)
#define A37XX_NB_UART1    (1 << 17)
#define A37XX_NB_SPI_QUAD (1 << 18)

/* south bridge */
#define A37XX_SB_SDIO    (1 << 2)
#define A37XX_SB_RGMII   (1 << 3)
#define A37XX_SB_SMI     (1 << 4)
#define A37XX_SB_PCIE1   (1 << 5)
#define A37XX_SB_PTP     (1 << 11)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ARMADA37XX_PINCTRL_H_ */
