/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <soc/sprd/board.h>
#include <linux/delay.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	struct regulator	*power_supply;
	struct gpio_desc	*enable_gpio;
	unsigned int		scale;
	bool			legacy;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
};

static void pwm_backlight_power_on(struct pwm_bl_data *pb)
{
	struct pwm_state state;
	int err;

	pwm_get_state(pb->pwm, &state);
	if (state.enabled)
		return;

	err = regulator_enable(pb->power_supply);
	if (err < 0)
		dev_err(pb->dev, "failed to enable power supply\n");

	state.enabled = true;
	pwm_apply_state(pb->pwm, &state);

	if (pb->enable_gpio)
		gpiod_set_value_cansleep(pb->enable_gpio, 1);
}

static void pwm_backlight_power_off(struct pwm_bl_data *pb)
{
	struct pwm_state state;

	pwm_get_state(pb->pwm, &state);
	if (!state.enabled)
		return;

	if (pb->enable_gpio)
		gpiod_set_value_cansleep(pb->enable_gpio, 0);

	state.enabled = false;
	state.duty_cycle = 0;
	pwm_apply_state(pb->pwm, &state);

	regulator_disable(pb->power_supply);
}

static int compute_duty_cycle(struct pwm_bl_data *pb, int brightness)
{
	unsigned int lth = pb->lth_brightness;
	struct pwm_state state;
	u64 duty_cycle;

	pwm_get_state(pb->pwm, &state);

	if (pb->levels)
		duty_cycle = pb->levels[brightness];
	else
		duty_cycle = brightness;

	duty_cycle *= state.period - lth;
	do_div(duty_cycle, pb->scale);

	return duty_cycle + lth;
}

static bool cali_mode;

static int boot_mode_check(char *str)
{
        if (str != NULL && !strncmp(str, "cali", strlen("cali")))
                cali_mode = true;
        else
                cali_mode = false;
        return 0;
}
__setup("androidboot.mode=", boot_mode_check);

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
	struct pwm_state state;
#ifdef ZCFG_BACKLIGHT_RESUME_DELAY
	static int flag_backlight_state = 0;
#endif
#ifdef ZCFG_PWM_BACKLIGHTNESS_LEVEL
	brightness =( brightness * ZCFG_PWM_BACKLIGHTNESS_LEVEL)/100;
#endif

  if (cali_mode) {
     pr_info("enter calibration mode, disable pwm backlight brightness\n");
     return 0;
  }

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness > 0) {
#ifdef ZCFG_BACKLIGHT_RESUME_DELAY
	if (flag_backlight_state == 1) {
		mdelay(ZCFG_BACKLIGHT_RESUME_DELAY);
		flag_backlight_state = 0;
	}
#endif
		pwm_get_state(pb->pwm, &state);
		state.duty_cycle = compute_duty_cycle(pb, brightness);
		pwm_apply_state(pb->pwm, &state);
		pwm_backlight_power_on(pb);
	} else {
#ifdef ZCFG_BACKLIGHT_RESUME_DELAY
		flag_backlight_state = 1;
#endif
		pwm_backlight_power_off(pb);
	}


	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int length;
	u32 value;
	int ret;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;

		data->dft_brightness = value;
		data->max_brightness--;
	}

	data->enable_gpio = -EINVAL;
	return 0;
}

static const struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int pwm_backlight_initial_power_state(const struct pwm_bl_data *pb)
{
	struct device_node *node = pb->dev->of_node;

	/* Not booted with device tree or no phandle link to the node */
	if (!node || !node->phandle)
		return FB_BLANK_UNBLANK;

	/*
	 * If the driver is probed from the device tree and there is a
	 * phandle link pointing to the backlight node, it is safe to
	 * assume that another driver will enable the backlight at the
	 * appropriate time. Therefore, if it is disabled, keep it so.
	 */

	/* if the enable GPIO is disabled, do not enable the backlight */
	if (pb->enable_gpio && gpiod_get_value_cansleep(pb->enable_gpio) == 0)
		return FB_BLANK_POWERDOWN;

	/* The regulator is disabled, do not enable the backlight */
	if (!regulator_is_enabled(pb->power_supply))
		return FB_BLANK_POWERDOWN;

	/* The PWM is disabled, keep it like this */
	if (!pwm_is_enabled(pb->pwm))
		return FB_BLANK_POWERDOWN;

	return FB_BLANK_UNBLANK;
}

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = dev_get_platdata(&pdev->dev);
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct device_node *node = pdev->dev.of_node;
	struct pwm_bl_data *pb;
	struct pwm_state state;
	int ret;

	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		unsigned int i;

		for (i = 0; i <= data->max_brightness; i++)
			if (data->levels[i] > pb->scale)
				pb->scale = data->levels[i];

		pb->levels = data->levels;
	} else
		pb->scale = data->max_brightness;
	pb->scale = data->max_brightness;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;

	pb->enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
						  GPIOD_ASIS);
	if (IS_ERR(pb->enable_gpio)) {
		ret = PTR_ERR(pb->enable_gpio);
		goto err_alloc;
	}

	/*
	 * Compatibility fallback for drivers still using the integer GPIO
	 * platform data. Must go away soon.
	 */
	if (!pb->enable_gpio && gpio_is_valid(data->enable_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, data->enable_gpio,
					    GPIOF_OUT_INIT_HIGH, "enable");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request GPIO#%d: %d\n",
				data->enable_gpio, ret);
			goto err_alloc;
		}

		pb->enable_gpio = gpio_to_desc(data->enable_gpio);
	}

	/*
	 * If the GPIO is not known to be already configured as output, that
	 * is, if gpiod_get_direction returns either 1 or -EINVAL, change the
	 * direction to output and set the GPIO as active.
	 * Do not force the GPIO to active when it was already output as it
	 * could cause backlight flickering or we would enable the backlight too
	 * early. Leave the decision of the initial backlight state for later.
	 */
	if (pb->enable_gpio &&
	    gpiod_get_direction(pb->enable_gpio) != 0)
		gpiod_direction_output(pb->enable_gpio, 1);

	pb->power_supply = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(pb->power_supply)) {
		ret = PTR_ERR(pb->power_supply);
		goto err_alloc;
	}

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm) && PTR_ERR(pb->pwm) != -EPROBE_DEFER && !node) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");
		pb->legacy = true;
		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
	}

	if (IS_ERR(pb->pwm)) {
		ret = PTR_ERR(pb->pwm);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "unable to request PWM\n");
		goto err_alloc;
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	/* Sync up PWM state. */
	pwm_init_state(pb->pwm, &state);

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data if it has not already been set
	 * via the PWM lookup table.
	 */
	if (!state.period && (data->pwm_period_ns > 0))
		state.period = data->pwm_period_ns;

	ret = pwm_apply_state(pb->pwm, &state);
	if (ret) {
		dev_err(&pdev->dev, "failed to apply initial PWM state: %d\n",
			ret);
		goto err_alloc;
	}

	pb->lth_brightness = data->lth_brightness * (state.period / pb->scale);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		if (pb->legacy)
			pwm_free(pb->pwm);
		goto err_alloc;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	bl->props.power = pwm_backlight_initial_power_state(pb);
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_backlight_power_off(pb);

	if (pb->exit)
		pb->exit(&pdev->dev);
	if (pb->legacy)
		pwm_free(pb->pwm);

	return 0;
}

static void pwm_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	pwm_backlight_power_off(pb);
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	if (pb->notify)
		pb->notify(pb->dev, 0);

	pwm_backlight_power_off(pb);

	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);

	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);

	return 0;
}
#endif

static const struct dev_pm_ops pwm_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = pwm_backlight_suspend,
	.resume = pwm_backlight_resume,
	.poweroff = pwm_backlight_suspend,
	.restore = pwm_backlight_resume,
#endif
};

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.pm		= &pwm_backlight_pm_ops,
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.shutdown	= pwm_backlight_shutdown,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");
