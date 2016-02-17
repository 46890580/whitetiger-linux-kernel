/*
 * Driver for the mxs internal temperature sensor
 *
 * Copyright 2013 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#define DRVNAME "mxs-hwmon"

#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>

#define GAIN_CORRECTION 1012

/* The value we calculate from the ADCs is in Kelvins, don't forget to convert
 * it to Celsius */
#define VALUES_TO_MILLIC(min, max) ((max - min) * GAIN_CORRECTION / 4 - 272150)

struct mxs_hwmon_data {
	struct device *hwmon_dev;
	struct iio_channel *chan_min;
	struct iio_channel *chan_max;
};

static int mxs_hwmon_show_temp(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int err, val_min, val_max;

	struct mxs_hwmon_data *data = dev_get_drvdata(dev);

	err = iio_read_channel_raw(data->chan_min, &val_min);
	if (err < 0)
		return err;

	err = iio_read_channel_raw(data->chan_max, &val_max);
	if (err < 0)
		return err;

	return sprintf(buf, "%u\n", VALUES_TO_MILLIC(val_min, val_max));
}

static DEVICE_ATTR(temp1_input, S_IRUGO, mxs_hwmon_show_temp, NULL);

static int mxs_hwmon_probe(struct platform_device *pdev)
{
	int err;
	struct mxs_hwmon_data *data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	err = device_create_file(&pdev->dev, &dev_attr_temp1_input);
	if (err)
		return err;

	data->chan_min = iio_channel_get(&pdev->dev, "min");
	if (IS_ERR(data->chan_min)) {
		err = PTR_ERR(data->chan_min);
		goto error_chan_min;
	}

	data->chan_max = iio_channel_get(&pdev->dev, "max");
	if (IS_ERR(data->chan_max)) {
		err = PTR_ERR(data->chan_max);
		goto error_chan_max;
	}

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto error_hwmon;
	}

	return 0;

error_hwmon:
	iio_channel_release(data->chan_max);
error_chan_max:
	iio_channel_release(data->chan_min);
error_chan_min:
	device_remove_file(&pdev->dev, &dev_attr_temp1_input);

	return err;
}

static int mxs_hwmon_remove(struct platform_device *pdev)
{
	struct mxs_hwmon_data *data = platform_get_drvdata(pdev);

	iio_channel_release(data->chan_min);
	iio_channel_release(data->chan_max);
	hwmon_device_unregister(data->hwmon_dev);

	device_remove_file(&pdev->dev, &dev_attr_temp1_input);

	return 0;
}

static struct of_device_id mxs_hwmon_of_match[] = {
	{ .compatible = "fsl,mxs-internal-temp", },
	{}
};
MODULE_DEVICE_TABLE(of, mxs_hwmon_of_match);

static struct platform_driver mxs_hwmon_driver = {
	.probe = mxs_hwmon_probe,
	.remove = mxs_hwmon_remove,
	.driver	= {
		.name = DRVNAME,
		.owner = THIS_MODULE,
		.of_match_table = mxs_hwmon_of_match,
	},
};

module_platform_driver(mxs_hwmon_driver);

MODULE_AUTHOR("Alexandre Belloni <alexandre.belloni@free-electrons.com>");
MODULE_DESCRIPTION("Freescale i.MX28 hwmon sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mxs-hwmon");
