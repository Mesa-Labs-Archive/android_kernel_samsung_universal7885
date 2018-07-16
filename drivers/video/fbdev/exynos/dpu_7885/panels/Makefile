obj-y += dsim_panel.o
obj-$(CONFIG_DISPLAY_USE_INFO) += dpui.o
obj-$(CONFIG_EXYNOS_DECON_MDNIE_LITE)	+= mdnie_lite.o

obj-$(and $(CONFIG_EXYNOS_DECON_MDNIE_LITE),$(CONFIG_SEC_GPIO_DVS),$(subst undefined,y,$(origin CONFIG_SAMSUNG_PRODUCT_SHIP))) += dd_mdnie.o
obj-$(and $(CONFIG_SEC_GPIO_DVS),$(subst undefined,y,$(origin CONFIG_SAMSUNG_PRODUCT_SHIP))) += dd_lcd.o dd_backlight.o dd_dpu.o

obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA0)	+= s6e3fa0_mipi_lcd.o s6e3fa0_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA3)	+= s6e3fa3_mipi_lcd.o s6e3fa3_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7)	+= s6e3fa7_mipi_lcd.o s6e3fa7_lcd_ctrl.o

obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A5Y18)	+= s6e3fa7_a7y18_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A7Y18)	+= s6e3fa7_a7y18_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_TD4100_J3TOPE)	+= td4100_j3tope_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6D7AT0B_J7TOPE)	+= s6d7at0b_j7tope_lcd_ctrl.o
obj-$(CONFIG_EXYNOS_DECON_LCD_EA8061S_J7DUO)	+= ea8061s_j7duo_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_A6ELTE)	+= s6e8aa5_a6elte_lcd_ctrl.o dynamic_aid.o
obj-$(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_FEEL2)	+= s6e8aa5_a6elte_lcd_ctrl.o dynamic_aid.o
