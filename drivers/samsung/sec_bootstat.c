/* sec_bootstat.c
 *
 * Copyright (C) 2014 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sec_class.h>
#include <linux/slab.h>


struct boot_event {
	const char *string;
	bool triggered;
};

static struct boot_event boot_events[] = {
	{"!@Boot: start init process", false},
	{"!@Boot: Begin of preload()", false},
	{"!@Boot: End of preload()", false},
	{"!@Boot: Entered the Android system server!", false},
	{"!@Boot: Start PackageManagerService", false},
	{"!@Boot: End PackageManagerService", false},
	{"!@Boot: Loop forever", false},
	{"!@Boot: performEnableScreen", false},
	{"!@Boot: Enabling Screen!", false},
	{"!@Boot: bootcomplete", false},
	{"!@Boot: Voice SVC is acquired", false},
	{"!@Boot: Data SVC is acquired", false},
	{"!@Boot_SVC : PhoneApp OnCrate", false},
	{"!@Boot_DEBUG: start networkManagement", false},
	{"!@Boot_DEBUG: end networkManagement", false},
	{"!@Boot_SVC : RIL_UNSOL_RIL_CONNECTED", false},
	{"!@Boot_SVC : setRadioPower on", false},
	{"!@Boot_SVC : setUiccSubscription", false},
	{"!@Boot_SVC : SIM fetchSimRecords", false},
	{"!@Boot_SVC : SIM onAllRecordsLoaded", false},
	{"!@Boot_SVC : RUIM onAllRecordsLoaded", false},
	{"!@Boot_SVC : setupDataCall", false},
	{"!@Boot_SVC : Response setupDataCall", false},
	{"!@Boot_SVC : onDataConnectionAttached", false},
	{"!@Boot_SVC : IMSI Ready", false},
	{"!@Boot_SVC : completeConnection", false},
	{"!@Boot_DEBUG: finishUserUnlockedCompleted", false},
	{"!@Boot: setIconVisibility: ims_volte: [SHOW]", false},
	{"!@Boot_DEBUG: Launcher.onCreate()", false},
	{"!@Boot_DEBUG: Launcher.onResume()", false},
	{"!@Boot_DEBUG: Launcher.LoaderTask.run() start", false},
	{"!@Boot_DEBUG: Launcher - FinishFirstBind", false},
};

void sec_bootstat_add(const char *buf)
{
	size_t i;

	if (!buf)
		return;

	for (i = 0; i < ARRAY_SIZE(boot_events); i++) {
		if (!boot_events[i].triggered && 
		    !strncmp(buf, boot_events[i].string, strlen(boot_events[i].string))) {
			
			boot_events[i].triggered = true;

			if (!strncmp(boot_events[i].string, "!@Boot: bootcomplete", 20))
				pr_info("Android boot complete detected!\n");

			break;
		}
	}
}

static ssize_t store_boot_stat(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	sec_bootstat_add(buf);
	return count;
}
static DEVICE_ATTR(boot_stat, 0220, NULL, store_boot_stat);

static int __init sec_bootstat_init(void)
{
	struct device *dev;

	dev = sec_device_create(NULL, "bsp");
	if (IS_ERR_OR_NULL(dev)) {
		pr_err("Failed to create sec_class bsp device\n");
		return -ENODEV;
	}

	if (device_create_file(dev, &dev_attr_boot_stat) < 0) {
		pr_err("Failed to create boot_stat sysfs file\n");
		return -ENODEV;
	}

	register_hook_bootstat(sec_bootstat_add);

	return 0;
}

late_initcall(sec_bootstat_init);
