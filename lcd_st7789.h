/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-03-23     Vandoul       First version
 */
#ifndef __LCD_ST7789_H__
#define __LCD_ST7789_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "drivers/spi.h"

/* 0-0 angle|1-90 angle|2-180 angle|-270 angle */
#define USE_DIRECTION   0

/* lcd size */
#define LCD_W PKG_ST_7789_WIDTH
#define LCD_H PKG_ST_7789_HEIGHT

#define LCD_DC_CLR  rt_pin_write(PKG_ST_7789_DC_PIN, PIN_LOW)
#define LCD_DC_SET  rt_pin_write(PKG_ST_7789_DC_PIN, PIN_HIGH)
#define LCD_RES_CLR rt_pin_write(PKG_ST_7789_RES_PIN, PIN_LOW)
#define LCD_RES_SET rt_pin_write(PKG_ST_7789_RES_PIN, PIN_HIGH)
#define LCD_BLK_CLR rt_pin_write(PKG_ST_7789_BLK_PIN, PIN_HIGH)
#define DELAY       rt_thread_mdelay

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

void LCD_Clear(uint16_t Color);
void LCD_direction(uint8_t direction);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_fill_array_spi(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *pcolor);

rt_err_t spi_lcd_init(uint32_t freq);

#ifdef __cplusplus
}
#endif
#endif
