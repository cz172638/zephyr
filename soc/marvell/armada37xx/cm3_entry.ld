/* SPDX-License-Identifier: Apache-2.0 */
/* after the exception vector table, at a small offset */
. = ALIGN(4);
KEEP(*(.a3700_cm3_entry))
