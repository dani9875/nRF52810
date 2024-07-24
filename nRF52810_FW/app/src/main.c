/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{

	printk("Use the sensor to change LED blinking period\n");

	while (1) {

		k_sleep(K_MSEC(100));
	}

	return 0;
}

