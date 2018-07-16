/* dsim_panel.h
 *
 * Copyright (c) 2017 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __DSIM_PANEL__
#define __DSIM_PANEL__


extern unsigned int lcdtype;

extern struct dsim_lcd_driver *mipi_lcd_driver;

#if IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA0)
extern struct dsim_lcd_driver s6e3fa0_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA7)
extern struct dsim_lcd_driver s6e3fa7_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A7Y18) || IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA7_A5Y18)
extern struct dsim_lcd_driver s6e3fa7_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_TD4100_J3TOPE)
extern struct dsim_lcd_driver td4100_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6D7AT0B_J7TOPE)
extern struct dsim_lcd_driver s6d7at0b_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_EA8061S_J7DUO)
extern struct dsim_lcd_driver ea8061s_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_A6ELTE)
extern struct dsim_lcd_driver s6e8aa5_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E8AA5_FEEL2)
extern struct dsim_lcd_driver s6e8aa5_mipi_lcd_driver;
#else
extern struct dsim_lcd_driver s6e3fa3_mipi_lcd_driver;
#endif

extern void dsim_register_panel(struct dsim_device *dsim);
extern int register_lcd_driver(struct dsim_lcd_driver *drv);

#endif
