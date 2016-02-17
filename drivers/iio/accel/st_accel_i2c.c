/*
 * STMicroelectronics accelerometers driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/of_device.h>

#include <linux/iio/common/st_sensors.h>
#include <linux/iio/common/st_sensors_i2c.h>
#include "st_accel.h"

static const struct of_device_id st_accel_of_table[] = {
	{ .compatible = "st,lis3dh", .data = LIS3DH_ACCEL_DEV_NAME },
	{ .compatible = "st,lis331dlh", .data = LIS331DLH_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm303dl-accel", .data = LSM303DL_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm303dlh-accel", .data = LSM303DLH_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm303dlhc-accel", .data = LSM303DLHC_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm303dlm-accel", .data = LSM303DLM_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm330-accel", .data = LSM330_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm330d-accel", .data = LSM330D_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm330dl-accel", .data = LSM330DL_ACCEL_DEV_NAME },
	{ .compatible = "st,lsm330dlc-accel", .data = LSM330DLC_ACCEL_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, st_accel_of_table);

static int st_accel_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct st_sensor_data *adata;
	int err;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*adata));
	if (!indio_dev)
		return -ENOMEM;

	adata = iio_priv(indio_dev);
	adata->dev = &client->dev;

	st_sensors_i2c_configure(indio_dev, client, adata);

	/*
	 * If we are probed through DT, st_sensors_i2c_configure will
	 * fill the indio_dev->name string with the client->name,
	 * which is the compatible without the vendor prefix.  Since
	 * compatibles separators are usually "-", and that the
	 * convention in this driver is using "_", we obviously have a
	 * problem when the st-sensors core checks that the two
	 * strings matches. We need to set again the indio_dev->name
	 * string to the real value used by the core later on.
	 */
	if (client->dev.of_node) {
		const struct of_device_id *device;
		device = of_match_device(st_accel_of_table, &client->dev);
		indio_dev->name = device->data;
	}

	err = st_accel_common_probe(indio_dev, client->dev.platform_data);
	if (err < 0)
		return err;

	return 0;
}

static int st_accel_i2c_remove(struct i2c_client *client)
{
	st_accel_common_remove(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id st_accel_id_table[] = {
	{ LSM303DLH_ACCEL_DEV_NAME },
	{ LSM303DLHC_ACCEL_DEV_NAME },
	{ LIS3DH_ACCEL_DEV_NAME },
	{ LSM330D_ACCEL_DEV_NAME },
	{ LSM330DL_ACCEL_DEV_NAME },
	{ LSM330DLC_ACCEL_DEV_NAME },
	{ LIS331DLH_ACCEL_DEV_NAME },
	{ LSM303DL_ACCEL_DEV_NAME },
	{ LSM303DLM_ACCEL_DEV_NAME },
	{ LSM330_ACCEL_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_accel_id_table);

static struct i2c_driver st_accel_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-accel-i2c",
		.of_match_table = of_match_ptr(st_accel_of_table),
	},
	.probe = st_accel_i2c_probe,
	.remove = st_accel_i2c_remove,
	.id_table = st_accel_id_table,
};
module_i2c_driver(st_accel_driver);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics accelerometers i2c driver");
MODULE_LICENSE("GPL v2");
