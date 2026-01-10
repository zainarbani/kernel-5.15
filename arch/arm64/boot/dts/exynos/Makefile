# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ARCH_EXYNOS),y)
ifeq ($(CONFIG_SOC_S5E8835),y)
dtb-y += s5e8835.dtb

# A546B - EU
dtbo-y += \
	samsung/a54x/a54x_eur_open_w00_r00.dtbo \
	samsung/a54x/a54x_eur_open_w00_r01.dtbo \
	samsung/a54x/a54x_eur_open_w00_r02.dtbo \
	samsung/a54x/a54x_eur_open_w00_r03.dtbo \
	samsung/a54x/a54x_eur_open_w00_r05.dtbo \
	samsung/a54x/a54x_eur_open_w00_r10.dtbo

# A546E - SEA
dtbo-y += \
	samsung/a54x/a54x_sea_open_w00_r00.dtbo \
	samsung/a54x/a54x_sea_open_w00_r02.dtbo \
	samsung/a54x/a54x_sea_open_w00_r03.dtbo \
	samsung/a54x/a54x_sea_open_w00_r05.dtbo

else
dtb-y += \
	exynos5433-tm2.dtb	\
	exynos5433-tm2e.dtb	\
	exynos7-espresso.dtb
endif
endif

targets += dtbs
DTB_LIST  := $(dtb-y) $(dtbo-y)
always-y    := $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb*