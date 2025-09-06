/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2023 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CIS_IMX258_H
#define IS_CIS_IMX258_H

#include "is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_IMX258_MAX_WIDTH		(4208)
#define SENSOR_IMX258_MAX_HEIGHT	(3120)

/* TODO: Check below values are valid */
#define SENSOR_IMX258_FINE_INTEGRATION_TIME_MIN                0x54C
#define SENSOR_IMX258_FINE_INTEGRATION_TIME_MAX                0x54C
#define SENSOR_IMX258_COARSE_INTEGRATION_TIME_MIN              0x1
#define SENSOR_IMX258_COARSE_INTEGRATION_TIME_MAX_MARGIN       0xA

#define USE_GROUP_PARAM_HOLD	(0)

enum sensor_imx258_mode_enum {
	SENSOR_IMX258_4128x3096_30FPS = 0,
	SENSOR_IMX258_4128x2324_30FPS = 1,
	SENSOR_IMX258_3408x2556_30FPS = 2,
	SENSOR_IMX258_3712x2556_30FPS = 3,
	SENSOR_IMX258_2064x1548_30FPS = 4,
	SENSOR_IMX258_2064x1160_30FPS = 5,
	SENSOR_IMX258_1024x768_120FPS = 6,
	SENSOR_IMX258_1856x1044_30FPS = 7,
	SENSOR_IMX258_1696x1272_30FPS = 8,
	SENSOR_IMX258_MODE_MAX,
};

#endif

