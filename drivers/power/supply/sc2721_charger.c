// SPDX-License-Identifier: GPL-2.0：
// Copyright (C) 2019 Spreadtrum Communications Inc.

#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/charger-manager.h>
#include <linux/usb/phy.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/slab.h>

#define SC2721_BATTERY_NAME			"sc27xx-fgu"

#define SC2721_CHG_CFG0				0x0
#define SC2721_CHG_CFG1				0x4

#define SC2721_TERM_VOLTAGE_MIN			4200
#define SC2721_TERM_VOLTAGE_MAX			4500
#define SC2721_TERM_VOLTAGE_STEP		100
#define SC2721_CCCV_VOLTAGE_STEP		75
#define SC2721_CHG_DPM_4300MV			4300
#define SC2721_CHG_CCCV_MAXIMUM			0x3f
#define SC2721_CHG_CURRENT_MIN			300
#define SC2721_CHG_CURRENT_1400MA		1400
#define SC2721_CHG_CURRENT_1400MA_REG		0x14
#define SC2721_CHG_CURRENT_STEP_50MA		50
#define SC2721_CHG_CURRENT_STEP_100MA		100

#define SC2721_CHG_PD				BIT(0)
#define SC2721_CHG_CC_MODE			BIT(12)

#define SC2721_CHG_DPM_MASK			GENMASK(14, 13)
#define SC2721_CHG_DPM_MASK_SHIT		13

#define SC2721_CHG_OVP_MASK			GENMASK(5, 0)

#define SC2721_CHG_TERMINATION_CURRENT_MASK	GENMASK(2, 1)
#define SC2721_CHG_TERMINATION_CURRENT_MASK_SHIT	1

#define SC2721_CHG_END_V_MASK			GENMASK(4, 3)
#define SC2721_CHG_END_V_MASK_SHIT		3

#define SC2721_CHG_CV_V_MASK			GENMASK(10, 5)
#define SC2721_CHG_CV_V_MASK_SHIT		5

#define SC2721_CHG_CC_I_MASK			GENMASK(14, 10)
#define SC2721_CHG_CC_I_MASK_SHIT		10

#define SC2721_CHG_CCCV_MASK			GENMASK(5, 0)

struct sc2721_charger_info {
	struct device *dev;
	struct regmap *regmap;
	struct usb_phy *usb_phy;
	struct notifier_block usb_notify;
	struct power_supply *psy_usb;
	struct power_supply_charge_current cur;
	struct work_struct work;
	struct mutex lock;
	bool charging;
	u32 limit;
	u32 base;
};

/*****for battry info start*******/
extern int s_max_volt;
/*****for battry info end*******/

static bool sc2721_charger_is_bat_present(struct sc2721_charger_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	bool present = false;
	int ret;

	psy = power_supply_get_by_name(SC2721_BATTERY_NAME);
	if (!psy) {
		dev_err(info->dev, "Failed to get psy of sc27xx_fgu\n");
		return present;
	}
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret == 0 && val.intval)
		present = true;
	power_supply_put(psy);

	if (ret)
		dev_err(info->dev,
			"Failed to get property of present:%d\n", ret);

	return present;
}

static int sc2721_set_termination_voltage(struct sc2721_charger_info *info,
					  u32 vol)
{
	struct nvmem_cell *cell;
	u32 big_level, small_level, cv;
	int ret, calib_data;
	void *buf;
	size_t len;

	cell = nvmem_cell_get(info->dev, "cccv_calib");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	memcpy(&calib_data, buf, min(len, sizeof(u32)));
	calib_data = calib_data & SC2721_CHG_CCCV_MASK;
	kfree(buf);
	
	if (vol > SC2721_TERM_VOLTAGE_MAX)
		vol = SC2721_TERM_VOLTAGE_MAX;

	if (vol >= SC2721_TERM_VOLTAGE_MIN) {
		u32 temp = vol % SC2721_TERM_VOLTAGE_STEP;

		/*
		 * To set 2721 charger cccv point.
		 * following formula:
		 * cccv point = big_level + small_level + efuse value
		 *
		 * the big_level corresponds to 2721 charger spec register
		 * CHGR_END_V
		 * the small_level corresponds to 2721 charger spec register
		 * CHGR_CV_V
		 */	 
		big_level = (vol - SC2721_TERM_VOLTAGE_MIN) / SC2721_TERM_VOLTAGE_STEP;
		cv = DIV_ROUND_CLOSEST(temp * 10, SC2721_CCCV_VOLTAGE_STEP);
		small_level = cv + calib_data;
		if (small_level > SC2721_CHG_CCCV_MAXIMUM) {
			big_level++;
			cv = DIV_ROUND_CLOSEST((SC2721_TERM_VOLTAGE_STEP - temp) * 10,
					       SC2721_CCCV_VOLTAGE_STEP);
			small_level = calib_data - cv;
		}
	} else {
		big_level = 0;
		cv = DIV_ROUND_CLOSEST((SC2721_TERM_VOLTAGE_MIN - vol) * 10,
				       SC2721_CCCV_VOLTAGE_STEP);
		if (cv > calib_data)
			small_level = 0;
		else
			small_level = calib_data - cv;
	}

	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_END_V_MASK,
				 big_level << SC2721_CHG_END_V_MASK_SHIT);
	if (ret) {
		dev_err(info->dev, "failed to set charge end_v\n");
		return ret;
	}
	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_CV_V_MASK,
				 small_level << SC2721_CHG_CV_V_MASK_SHIT);
	if (ret)
		dev_err(info->dev, "failed to set charge end_v\n");
	return ret;
}

static int sc2721_charger_hw_init(struct sc2721_charger_info *info)
{
	struct power_supply_battery_info bat_info = { };
	u32 voltage_max_microvolt;
	int ret;
	ret = power_supply_get_battery_info(info->psy_usb, &bat_info,0);
	if (ret) {
		dev_warn(info->dev, "no battery information is supplied\n");
		info->cur.sdp_cur = 500000;
		info->cur.dcp_cur = 500000;
		info->cur.cdp_cur = 1500000;
		info->cur.unknown_cur = 500000;
	} else {
		info->cur.sdp_cur = bat_info.cur.sdp_cur;
		info->cur.dcp_cur = bat_info.cur.dcp_cur;
		info->cur.cdp_cur = bat_info.cur.cdp_cur;
		info->cur.unknown_cur = bat_info.cur.unknown_cur;

		voltage_max_microvolt =
			bat_info.constant_charge_voltage_max_uv / 1000;
		power_supply_put_battery_info(info->psy_usb, &bat_info);

		ret = regmap_update_bits(info->regmap,
					 info->base + SC2721_CHG_CFG0,
					 SC2721_CHG_CC_MODE,
					 SC2721_CHG_CC_MODE);
		if (ret) {
			dev_err(info->dev, "failed to enable charger cc\n");
			return ret;
		}

		ret = regmap_update_bits(info->regmap,
					 info->base + SC2721_CHG_CFG0,
					 SC2721_CHG_DPM_MASK,
					 0x3 << SC2721_CHG_DPM_MASK_SHIT);
		if (ret) {
			dev_err(info->dev, "failed to set charger dpm\n");
			return ret;
		}

		/* Set charge termination voltage */
		ret = sc2721_set_termination_voltage(info,
						     voltage_max_microvolt);
		if (ret) {
			dev_err(info->dev, "failed to set termination voltage\n");
			return ret;
		}

		/*****for battry info start*******/
		s_max_volt = voltage_max_microvolt;
		/*****for battry info start*******/	

		/* Set charge over voltage protection value 6500mv */
		ret = regmap_update_bits(info->regmap,
					 info->base + SC2721_CHG_CFG1,
					 SC2721_CHG_OVP_MASK,
					 0x09);
		if (ret) {
			dev_err(info->dev, "failed to set charger ovp\n");
			return ret;
		}
	}
		printk("y*%d %s\n", __LINE__, __func__);	
	return ret;
}

static int sc2721_charger_start_charge(struct sc2721_charger_info *info)
{
	int ret;

	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_PD, 0);
	if (ret)
		dev_err(info->dev, "failed to satrt charge\n");

	return ret;
}

static int sc2721_charger_stop_charge(struct sc2721_charger_info *info)
{
	int ret;

	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_PD,
				 SC2721_CHG_PD);
	if (ret)
		dev_err(info->dev, "failed to stop charge\n");

	return ret;
}

static int sc2721_charger_set_current(struct sc2721_charger_info *info, u32 cur)
{
	int temp, ret;
	u32 val;
	printk("y*%d %s cur=%d\n", __LINE__, __func__, cur);
	if(info->usb_phy->chg_type == SDP_TYPE)
	{
		cur = info->cur.sdp_cur;
		printk("usb charger limit cur by sdp_cur in dtsi\n");
	}
	cur = cur / 1000;

	if (cur < SC2721_CHG_CURRENT_MIN)
		cur = SC2721_CHG_CURRENT_MIN;

	if (cur < SC2721_CHG_CURRENT_1400MA) {
		temp = (cur - SC2721_CHG_CURRENT_MIN) /
			SC2721_CHG_CURRENT_STEP_50MA;
	} else {
		temp = (cur - SC2721_CHG_CURRENT_1400MA) /
			SC2721_CHG_CURRENT_STEP_100MA;
		temp += SC2721_CHG_CURRENT_1400MA_REG;
	}
	
	printk("y*%d %s temp=%x\n", __LINE__, __func__, temp);
	/* Disable charge cc mode */
	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_CC_MODE, 0);
	if (ret) {
		dev_err(info->dev, "failed to disable charge cc mode\n");
		return ret;
	}
	/* Set charge current */
	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG1,
				 SC2721_CHG_CC_I_MASK,
				 temp << SC2721_CHG_CC_I_MASK_SHIT);
	if (ret) {
		dev_err(info->dev, "failed to set charge current\n");
		return ret;
	}

	/* Enable charge cc mode */
	ret = regmap_update_bits(info->regmap,
				 info->base + SC2721_CHG_CFG0,
				 SC2721_CHG_CC_MODE,
				 SC2721_CHG_CC_MODE);
	if (ret)
		dev_err(info->dev, "failed to enable charge cc mode\n");
	ret = regmap_read(info->regmap, info->base + SC2721_CHG_CFG0, &val);	
	printk("y*%d %s ctrl0=%x\n", __LINE__, __func__, val);
	ret = regmap_read(info->regmap, info->base + SC2721_CHG_CFG1, &val);	
	printk("y*%d %s ctrl1=%x\n", __LINE__, __func__, val);	
	return ret;
}

static int sc2721_charger_get_current(struct sc2721_charger_info *info,
				      u32 *cur)
{
	int ret;
	u32 val;

	ret = regmap_read(info->regmap, info->base + SC2721_CHG_CFG0, &val);
	if (ret)
		return ret;

	if (!(val & SC2721_CHG_CC_MODE)) {		
		printk("y*%d %s\n", __LINE__, __func__);
		*cur = 0;
		return 0;
	}

	ret = regmap_read(info->regmap, info->base + SC2721_CHG_CFG1, &val);
	if (ret)
		return ret;

	val = (val & SC2721_CHG_CC_I_MASK) >> SC2721_CHG_CC_I_MASK_SHIT;
	if (val >= SC2721_CHG_CURRENT_1400MA_REG) {
		*cur = SC2721_CHG_CURRENT_1400MA +
			(val - SC2721_CHG_CURRENT_1400MA_REG) *
			SC2721_CHG_CURRENT_STEP_100MA;
		printk("y*%d %s cur=%d\n", __LINE__, __func__, *cur);
	} else {
		*cur = SC2721_CHG_CURRENT_MIN +
			val * SC2721_CHG_CURRENT_STEP_50MA;	
		printk("y*%d %s cur=%d\n", __LINE__, __func__, *cur);
	}

	printk("y*%d %s cur=%d\n", __LINE__, __func__, *cur);
	return 0;
}

static int sc2721_charger_get_health(struct sc2721_charger_info *info,
				     u32 *health)
{
	*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sc2721_charger_get_online(struct sc2721_charger_info *info,
				     u32 *online)
{
	if (info->limit)
		*online = true;
	else
		*online = false;

	return 0;
}

static int sc2721_charger_get_status(struct sc2721_charger_info *info)
{
	if (info->charging == true)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int sc2721_charger_set_status(struct sc2721_charger_info *info,
				       int val)
{
	int ret = 0;

	if (!val && info->charging) {
		sc2721_charger_stop_charge(info);
		info->charging = false;
	} else if (val && !info->charging) {
		ret = sc2721_charger_start_charge(info);
		if (ret)
			dev_err(info->dev, "start charge failed\n");
		else
			info->charging = true;
	}

	return ret;
}

static void sc2721_charger_work(struct work_struct *data)
{
	struct sc2721_charger_info *info =
		container_of(data, struct sc2721_charger_info, work);
	int cur, ret;
	bool present = sc2721_charger_is_bat_present(info);

	mutex_lock(&info->lock);

	if (info->limit > 0 && !info->charging && present) {
		/* set charger current and start to charge */
		switch (info->usb_phy->chg_type) {
		case SDP_TYPE:
			cur = info->cur.sdp_cur;
			break;
		case DCP_TYPE:
			cur = info->cur.dcp_cur;
			break;
		case CDP_TYPE:
			cur = info->cur.cdp_cur;
			break;
		default:
			cur = info->cur.unknown_cur;
		}

		ret = sc2721_charger_set_current(info, cur);
		if (ret)
			goto out;

		ret = sc2721_charger_start_charge(info);
		if (ret)
			goto out;

		info->charging = true;
	} else if ((!info->limit && info->charging) || !present) {
		/* Stop charging */
		info->charging = false;
		sc2721_charger_stop_charge(info);
	}

out:
	mutex_unlock(&info->lock);
	dev_info(info->dev, "battery present = %d, charger type = %d\n",
		 present, info->usb_phy->chg_type);
	cm_notify_event(info->psy_usb, CM_EVENT_CHG_START_STOP, NULL);
}

static int sc2721_charger_usb_change(struct notifier_block *nb,
				     unsigned long limit, void *data)
{
	struct sc2721_charger_info *info =
		container_of(nb, struct sc2721_charger_info, usb_notify);

	info->limit = limit;

	schedule_work(&info->work);
	return NOTIFY_OK;
}

static int sc2721_charger_usb_get_property(struct power_supply *psy,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	struct sc2721_charger_info *info = power_supply_get_drvdata(psy);
	u32 cur, online, health;
	enum usb_charger_type type;
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:

		if (info->limit)
			val->intval = sc2721_charger_get_status(info);
					
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			ret = sc2721_charger_get_current(info, &cur);		
			if (ret)
				goto out;

			val->intval = cur * 1000;		
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			switch (info->usb_phy->chg_type) {
			case SDP_TYPE:
				val->intval = info->cur.sdp_cur;
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);				
				break;
			case DCP_TYPE:
				val->intval = info->cur.dcp_cur;
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);				
				break;
			case CDP_TYPE:
				val->intval = info->cur.cdp_cur;
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);				
				break;
			default:
				val->intval = info->cur.unknown_cur;
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);				
			}
		}
		break;
		
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sc2721_charger_get_online(info, &online);
		if (ret)
			goto out;

		val->intval = online;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (info->charging) {
			val->intval = 0;
		} else {
			ret = sc2721_charger_get_health(info, &health);
			if (ret)
				goto out;

			val->intval = health;
		}
		break;

	case POWER_SUPPLY_PROP_USB_TYPE:
		type = info->usb_phy->chg_type;

		switch (type) {
		case SDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case DCP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		case CDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		break;


	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int
sc2721_charger_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sc2721_charger_info *info = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&info->lock);

	if (!info->charging && psp != POWER_SUPPLY_PROP_STATUS) {
		mutex_unlock(&info->lock);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);
		ret = sc2721_charger_set_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current failed\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);	
		ret = sc2721_charger_set_status(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge status failed\n");
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
printk("y*%d %s val->intval=%d\n", __LINE__, __func__, val->intval);	
		ret = sc2721_set_termination_voltage(info, val->intval / 1000);
		if (ret < 0)
			dev_err(info->dev, "failed to set terminate voltage\n");
		break;
	case POWER_SUPPLY_PROP_FEED_WATCHDOG:
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int sc2721_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_usb_type sc2721_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property sc2721_usb_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
};

static const struct power_supply_desc sc2721_charger_desc = {
	.name			= "sc2721_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= sc2721_usb_props,
	.num_properties		= ARRAY_SIZE(sc2721_usb_props),
	.get_property		= sc2721_charger_usb_get_property,
	.set_property		= sc2721_charger_usb_set_property,
	.property_is_writeable	= sc2721_charger_property_is_writeable,
	.usb_types		= sc2721_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(sc2721_charger_usb_types)
};

static void sc2721_charger_detect_status(struct sc2721_charger_info *info)
{
	unsigned int min, max;

	/*
	 * If the USB charger status has been USB_CHARGER_PRESENT before
	 * registering the notifier, we should start to charge with getting
	 * the charge current.
	 */
	if (info->usb_phy->chg_state != USB_CHARGER_PRESENT)
		return;

	usb_phy_get_charger_current(info->usb_phy, &min, &max);

	info->limit = min;
	schedule_work(&info->work);
}

static int sc2721_charger_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sc2721_charger_info *info;
	struct power_supply_config charger_cfg = { };
	int ret;

	printk("sc2721_charger_probe\n");
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mutex_init(&info->lock);
	info->dev = &pdev->dev;
	INIT_WORK(&info->work, sc2721_charger_work);
	info->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!info->regmap) {
		dev_err(&pdev->dev, "failed to get charger regmap\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, info);

	info->usb_phy = devm_usb_get_phy_by_phandle(&pdev->dev, "phys", 0);
	if (IS_ERR(info->usb_phy)) {
		dev_err(&pdev->dev, "failed to find USB phy\n");
		return PTR_ERR(info->usb_phy);
	}

	ret = of_property_read_u32(np, "reg", &info->base);
	if (ret) {
		dev_err(&pdev->dev, "failed to get register address\n");
		return -ENODEV;
	}

	info->usb_notify.notifier_call = sc2721_charger_usb_change;
	ret = usb_register_notifier(info->usb_phy, &info->usb_notify);
	if (ret) {
		dev_err(&pdev->dev, "failed to register notifier:%d\n", ret);
		return ret;
	}

	charger_cfg.drv_data = info;
	charger_cfg.of_node = np;
	info->psy_usb = devm_power_supply_register(&pdev->dev,
						   &sc2721_charger_desc,
						   &charger_cfg);
	if (IS_ERR(info->psy_usb)) {
		dev_err(&pdev->dev, "failed to register power supply\n");
		usb_unregister_notifier(info->usb_phy, &info->usb_notify);
		return PTR_ERR(info->psy_usb);
	}

	ret = sc2721_charger_hw_init(info);
	if (ret)
		return ret;
	sc2721_charger_detect_status(info);
	{
		extern void zyt_info_s2(char* s1,char* s2);
		zyt_info_s2("[CHARGE_IC]:","SC2721");
	}
	printk("y*%d %s\n", __LINE__, __func__);
	return 0;
}

static int sc2721_charger_remove(struct platform_device *pdev)
{
	struct sc2721_charger_info *info = platform_get_drvdata(pdev);

	usb_unregister_notifier(info->usb_phy, &info->usb_notify);

	return 0;
}


static const struct of_device_id sc2721_charger_of_match[] = {
	{ .compatible = "sprd,sc2721-charger", },
	{ }
};

static struct platform_driver sc2721_charger_driver = {
	.driver = {
		.name = "sc2721-charger",
		.of_match_table = sc2721_charger_of_match,
	},
	.probe = sc2721_charger_probe,
	.remove = sc2721_charger_remove,
};

module_platform_driver(sc2721_charger_driver);

MODULE_DESCRIPTION("Spreadtrum SC2721 Charger Driver");
MODULE_LICENSE("GPL v2");
