/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Exynos 1380 GPU Clocks
 * DVFS table converted to arrays
 */

#ifndef _S5E8835_CCLK_GPU_H_
#define _S5E8835_CCLK_GPU_H_

#ifndef CPU_MAX
#define CPU_MAX INT_MAX
#endif

#define GPU_FREQ_STOCK_KHZ_MAX (949000)

#ifdef CONFIG_SOC_S5E8835_GPU_OC
/*******************************************
 * Overclocked frequencies 
 ******************************************/
#define GPU_FREQ_KHZ_MAX (1053000)
#define GPU_FREQ_KHZ_MIN (221000)

static const unsigned int gpu_custom_clock[] = {
	1053000, 949000, 845000, 754000, 650000, 552000, 455000, 351000, 221000
};
static const unsigned int gpu_custom_min_threshold[] = {
	78, 78, 78, 78, 78, 78, 78, 78, 0
};
static const unsigned int gpu_custom_max_threshold[] = {
	100, 97, 95, 95, 95, 95, 90, 85, 85
};
static const unsigned int gpu_custom_staycount[] = {
	5, 5, 5, 5, 3, 1, 1, 1, 1
};
static const unsigned int gpu_custom_mem_freq[] = {
	1794000, 1794000, 1541000, 1352000, 1352000, 1014000, 1014000, 845000, 676000
};
static const unsigned int gpu_custom_lit[] = {
	1069000, 1069000, 1069000, 1069000, 1069000, 1069000, 0, 0, 0
};
static const unsigned int gpu_custom_mid = 0;
static const unsigned int gpu_custom_big = CPU_MAX;
#else
/*******************************************
 * Stock frequencies 
 ******************************************/
#define GPU_FREQ_KHZ_MAX (949000)
#define GPU_FREQ_KHZ_MIN (221000)

static const unsigned int gpu_custom_clock[] = {
	949000, 845000, 754000, 650000, 552000, 455000, 351000, 221000
};
static const unsigned int gpu_custom_min_threshold[] = {
	78, 78, 78, 78, 78, 78, 78, 0
};
static const unsigned int gpu_custom_max_threshold[] = {
	100, 95, 95, 95, 95, 90, 85, 85
};
static const unsigned int gpu_custom_staycount[] = {
	5, 5, 5, 3, 1, 1, 1, 1
};
static const unsigned int gpu_custom_mem_freq[] = {
	1794000, 1541000, 1352000, 1352000, 1014000, 1014000, 845000, 676000
};
static const unsigned int gpu_custom_lit[] = {
	1069000, 1069000, 1069000, 1069000, 1069000, 0, 0, 0
};
static const unsigned int gpu_custom_mid = 0;
static const unsigned int gpu_custom_big = CPU_MAX;
#endif

static const int gpu_custom_array_size = sizeof(gpu_custom_clock) / sizeof(gpu_custom_clock[0]);

#endif /* _S5E8835_CCLK_GPU_H_ */
