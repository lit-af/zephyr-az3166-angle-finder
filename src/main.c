/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>

#include <zsl/orientation/ahrs.h>

const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl) ;

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
	SENSOR_CHAN_GYRO_X,
	SENSOR_CHAN_GYRO_Y,
	SENSOR_CHAN_GYRO_Z
};

static int print_accels(const struct device *dev)
{
	int ret;
	struct sensor_value accel[6];

	ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		printk("%s: sensor_sample_fetch() failed: %d\n", dev->name, ret);
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		ret = sensor_channel_get(dev, channels[i], &accel[i]);
		if (ret < 0) {
			printk("%s: sensor_channel_get(%c) failed: %d\n", dev->name, 'X' + i, ret);
			return ret;
		}
	}

	printk("%16s [m/s^2]:    (%12.6f, %12.6f, %12.6f)", dev->name,
	       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
	       sensor_value_to_double(&accel[2]));
	printk(" (%12.6f, %12.6f, %12.6f)\n", dev->name,
	       sensor_value_to_double(&accel[3]), sensor_value_to_double(&accel[4]),
	       sensor_value_to_double(&accel[5]));

	return 0;
}

int main(void)
{
	int ret;

	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device %s not ready.\n", lsm6dsl_dev->name);
		return 0;
	}

	/// Configuring the Accelerometer ODR to 104 Hz
	struct sensor_value val[1];
	val[0].val1 = 104; /// Hz
	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (ret < 0) {
		printk("Couldn't configure attribute: ACCEL ODR\n");
		return 0;
	}
	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
		if (ret < 0) {
		printk("Couldn't configure attribute: GYRO ODR\n");
		return 0;
	}
	

	while (1) {
		ret = print_accels(lsm6dsl_dev);
		if (ret < 0) {
			return 0;
		}
		k_msleep(500);
	}
	return 0;
}
