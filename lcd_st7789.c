/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-06-10     Vandoul       First version
 */

#include "rtthread.h"
#include "lcd_st7789.h"

#define DBG_TAG "st7789"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef PKG_USING_ST7789

#define LCD_CMD_COL_ADDR_SET                0x2A
#define LCD_CMD_ROW_ADDR_SET                0x2B
#define LCD_CMD_MEMORY_WRITE                0x2C
/**
 * MADCTL(36H):
 * +---------------------------------------+
 * | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
 * | MY | MX | MV | ML | RGB| MH |  - |  - |
 * +---------------------------------------+
 * [3] 0=RGB,1=BGR
*/
#define LCD_CMD_MEMORY_DATA_ACCESS_CTRL     0x36
/**
 * COLMOD(3AH):
 * +---------------------------------------+
 * | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
 * |  0 | D6 | D5 | D4 |  0 | D2 | D1 | D0 |
 * +---------------------------------------+
 * [6:4] RGB interface Color format:
 *      101=65K RGB
 *      110=262K RGB
 * [2:0] Control interface color format:
 *      011=12bit/pixel
 *      101=16bit/pixel
 *      110=18bit/pixel
 *      111=16M truncated
*/
#define LCD_CMD_INTERFACE_PIXEL_FORMAT      0x3A


typedef struct
{
    struct rt_device parent;
    uint16_t width;   /* LCD width */
    uint16_t height;  /* LCD high */
    uint32_t id;      /* LCD ID */
    uint8_t dir;      /* 0:Vertical | 1:Vertical */
} _lcd_dev;

/* LCD param */
_lcd_dev lcddev;
static struct rt_spi_device *lcd_dev;
static uint16_t frame_buf[LCD_H][LCD_W];
static void LCD_RESET(void)
{
    LCD_RES_CLR;
    DELAY(100);
    LCD_RES_SET;
    DELAY(100);
}

static void LCD_WR_REG(uint8_t reg)
{
    LCD_DC_CLR;
    rt_spi_send(lcd_dev, &reg, 1);
    LCD_DC_SET;
}

static void LCD_WR_DATA(uint8_t data)
{
    LCD_DC_SET;
    rt_spi_send(lcd_dev, &data, 1);
}

static void LCD_ReadData(uint8_t *data, uint16_t length)
{
    LCD_DC_SET;
    rt_spi_transfer(lcd_dev, RT_NULL, &data, length);
}

static void LCD_WriteReg(uint8_t reg, uint16_t regdata)
{
    LCD_WR_REG(reg);
    LCD_WR_DATA(regdata);
}

static void LCD_WriteRAM_Prepare(void)
{
    LCD_WR_REG(LCD_CMD_MEMORY_WRITE);
}

static void LCD_WriteData_16Bit(uint16_t Data)
{
    uint8_t buf[2];
    LCD_DC_SET;
    buf[0] = Data >> 8;
    buf[1] = Data & 0xff;
    rt_spi_send(lcd_dev, buf, 2);
}

void LCD_direction(uint8_t direction)
{
    switch (direction)
    {
    case 0:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        LCD_WriteReg(LCD_CMD_MEMORY_DATA_ACCESS_CTRL, (0 << 3) | (0 << 5) | (0 << 6) | (0 << 7)); /* BGR==0,MV==0,MX==0,MY==0 */
        break;
    case 1:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        LCD_WriteReg(LCD_CMD_MEMORY_DATA_ACCESS_CTRL, (0 << 3) | (1 << 5) | (1 << 6) | (0 << 7)); /* BGR==0,MV==1,MX==0,MY==1 */
        break;
    case 2:
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        LCD_WriteReg(LCD_CMD_MEMORY_DATA_ACCESS_CTRL, (0 << 3) | (0 << 5) | (1 << 6) | (1 << 7)); /* BGR==0,MV==0,MX==1,MY==1 */
        break;
    case 3:
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        LCD_WriteReg(LCD_CMD_MEMORY_DATA_ACCESS_CTRL, (0 << 3) | (1 << 5) | (0 << 6) | (1 << 7)); /* BGR==0,MV==1,MX==0,MY==1 */
        break;
    default:
        break;
    }
}

void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
    LCD_WR_REG(LCD_CMD_COL_ADDR_SET);
    LCD_WR_DATA(xStar >> 8);
    LCD_WR_DATA(0x00FF & xStar);
    LCD_WR_DATA(xEnd >> 8);
    LCD_WR_DATA(0x00FF & xEnd);

    LCD_WR_REG(LCD_CMD_ROW_ADDR_SET);
    LCD_WR_DATA(yStar >> 8);
    LCD_WR_DATA(0x00FF & yStar);
    LCD_WR_DATA(yEnd >> 8);
    LCD_WR_DATA(0x00FF & yEnd);

    LCD_WriteRAM_Prepare();
}

void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    LCD_SetWindows(Xpos, Ypos, Xpos, Ypos);
}

void LCD_Clear(uint16_t Color)
{
    unsigned int i, m;

    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);

    LCD_DC_SET;
    for (i = 0; i < lcddev.height; i++)
    {
        for (m = 0; m < lcddev.width; m++)
        {
            frame_buf[i][m] = Color;
        }
        for (m = 0; m < lcddev.width;)
        {
            m += 40;
            rt_spi_send(lcd_dev, (uint8_t *)&frame_buf[i][m], 80);
        }
    }
}

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t i, j;
    LCD_SetWindows(xsta, ysta, xend - 1, yend - 1);
    for (i = ysta; i < yend; i++)
    {
        for (j = xsta; j < xend; j++)
        {
            frame_buf[i][j] = color;
            LCD_WriteData_16Bit(color);
        }
    }
}

void LCD_Sync_fb(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint16_t i, j;
    LCD_SetWindows(x, y, x + w - 1, y + h - 1);
    LCD_DC_SET;
    for (i = 0; i < h; i++)
    {
        rt_spi_send(lcd_dev, (uint8_t *)&frame_buf[y+i][x], w * 2);
    }
}

void lcd_fill_array_spi(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t *Image)
{
    rt_uint32_t size = 0;
    uint16_t *pImage = Image;

    size = (Xend - Xstart + 1) * (Yend - Ystart + 1) * 2;/*16bit*/
    LCD_SetWindows(Xstart, Ystart, Xend, Yend);
    LCD_DC_SET;

    for(int i=0; i<(Yend - Ystart); i++)
    {
        for(int j=0; j<(Xend - Xstart); j++)
        {
            frame_buf[i + Ystart][j + Xstart] = *pImage++;
        }
    }
    rt_spi_send(lcd_dev, Image, size);
}

static void _st7789_init(void)
{
    LCD_WR_REG(LCD_CMD_MEMORY_DATA_ACCESS_CTRL);
    LCD_WR_DATA(0x00);

    LCD_WR_REG(LCD_CMD_INTERFACE_PIXEL_FORMAT);
    LCD_WR_DATA(0X05); //03=RGB444,05=RGB565,06=RGB666

    LCD_WR_REG(0xB2); //Porch Setting
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0X00);
    LCD_WR_DATA(0X33);
    LCD_WR_DATA(0X33);

    LCD_WR_REG(0xB7); //Gate Control
    LCD_WR_DATA(0x46);

    LCD_WR_REG(0xBB); //VCOM Setting
    LCD_WR_DATA(0x1B);

    LCD_WR_REG(0xC0); //LCM Control
    LCD_WR_DATA(0x2C);

    LCD_WR_REG(0xC2); //VDV and VRH Command Enable
    LCD_WR_DATA(0x01);

    LCD_WR_REG(0xC3); //VRH set
    LCD_WR_DATA(0x0F);

    LCD_WR_REG(0xC4); //VDV set
    LCD_WR_DATA(0x20);

    LCD_WR_REG(0xC6); //Frame Rate Control
    LCD_WR_DATA(0x0F);

    LCD_WR_REG(0xD0); //Power Control 1
    LCD_WR_DATA(0xA4);
    LCD_WR_DATA(0xA1);

    LCD_WR_REG(0xD6);
    LCD_WR_DATA(0xA1);

    LCD_WR_REG(0xE0);   /* set Positive Voltage Gamma */
    LCD_WR_DATA(0xF0);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x06);
    LCD_WR_DATA(0x04);
    LCD_WR_DATA(0x05);
    LCD_WR_DATA(0x05);
    LCD_WR_DATA(0x31);
    LCD_WR_DATA(0x44);
    LCD_WR_DATA(0x48);
    LCD_WR_DATA(0x36);
    LCD_WR_DATA(0x12);
    LCD_WR_DATA(0x12);
    LCD_WR_DATA(0x2B);
    LCD_WR_DATA(0x34);

    LCD_WR_REG(0xE1);   /* set Negative Voltage Gamma */
    LCD_WR_DATA(0XF0);
    LCD_WR_DATA(0X0B);
    LCD_WR_DATA(0X0F);
    LCD_WR_DATA(0X0F);
    LCD_WR_DATA(0X0D);
    LCD_WR_DATA(0X26);
    LCD_WR_DATA(0X31);
    LCD_WR_DATA(0X43);
    LCD_WR_DATA(0X47);
    LCD_WR_DATA(0X38);
    LCD_WR_DATA(0X14);
    LCD_WR_DATA(0X14);
    LCD_WR_DATA(0X2C);
    LCD_WR_DATA(0X32);

    LCD_WR_REG(0x21); // Dispaly Inversion On

    LCD_WR_REG(0x29); // Display On

    LCD_WR_REG(LCD_CMD_MEMORY_WRITE); // Memory Write
}

static void Lcd_pin_init(void)
{
    rt_pin_mode(PKG_ST_7789_DC_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(PKG_ST_7789_RES_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(PKG_ST_7789_BLK_PIN, PIN_MODE_OUTPUT);
}

static void LCD_Init(void)
{
    Lcd_pin_init();

    LCD_RESET();        /* LCD Hardware Reset */
    DELAY(120);         /* Delay 120ms */
    LCD_WR_REG(0x11);   /* Sleep out */
    DELAY(120);         /* Delay 120ms */
    _st7789_init();    /* IlI9341 init */
    LCD_BLK_CLR;        /* Open Backlight */

    LCD_direction(USE_DIRECTION);
}

rt_err_t spi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result = RT_EOK;
    struct rt_spi_device *spi_device;

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);

    result = rt_spi_bus_attach_device_cspin(spi_device, device_name, bus_name, cs_pin, RT_NULL);
    if (RT_EOK != result)
    {
        LOG_E("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }
    else
    {
        LOG_I("%s attach to %s done", device_name, bus_name);
    }

    return result;
}

rt_err_t spi_lcd_init(uint32_t freq)
{
    rt_err_t res = RT_EOK;

    spi_device_attach(PKG_ST_7789_SPI_BUS_NAME, PKG_ST_7789_SPI_DEVICE_NAME, PKG_ST_7789_CS_PIN);
    lcd_dev = (struct rt_spi_device *)rt_device_find(PKG_ST_7789_SPI_DEVICE_NAME);
    if (lcd_dev != RT_NULL)
    {
        struct rt_spi_configuration spi_config;
        spi_config.data_width = 8;
        spi_config.max_hz = freq * 1000 * 1000;
        spi_config.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        rt_spi_configure(lcd_dev, &spi_config);
    }
    else
    {
        res = -RT_ERROR;
    }

    LCD_Init();

    return res;
}
#if 0
static void lcd_ops_set_pixel (const char *pixel, int x, int y)
{
}
static void lcd_ops_get_pixel (char *pixel, int x, int y)
{
}
static void lcd_ops_draw_hline(const char *pixel, int x1, int x2, int y)
{
}
static void lcd_ops_draw_vline(const char *pixel, int x, int y1, int y2)
{
}
#endif
static void lcd_ops_blit_line(const char *pixel, int x, int y, rt_size_t size)
{
    LCD_SetWindows(x, y, x + size, y);
    rt_spi_send(lcd_dev, pixel, size * 2);
}
struct rt_device_graphic_ops lcd_graphic_ops =
{
    .set_pixel  = RT_NULL,//lcd_ops_set_pixel ,
    .get_pixel  = RT_NULL,//lcd_ops_get_pixel ,
    .draw_hline = RT_NULL,//lcd_ops_draw_hline,
    .draw_vline = RT_NULL,//lcd_ops_draw_vline,
    .blit_line  = lcd_ops_blit_line ,
};
rt_err_t lcd_dev_ops_init(rt_device_t dev)
{
    spi_lcd_init(25);
    return RT_EOK;
}
rt_err_t lcd_dev_ops_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}
rt_err_t lcd_dev_ops_close(rt_device_t dev)
{
    return RT_EOK;
}
rt_ssize_t lcd_dev_ops_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_memcpy(buffer, &((uint16_t *)frame_buf)[pos], size * 2);
    return RT_EOK;
}
rt_ssize_t lcd_dev_ops_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_memcpy(&((uint16_t *)frame_buf)[pos], buffer, size * 2);
    return RT_EOK;
}
rt_err_t lcd_dev_ops_control(rt_device_t dev, int cmd, void *args)
{
    switch(cmd)
    {
        case RTGRAPHIC_CTRL_RECT_UPDATE:
        {
            struct rt_device_rect_info *rect_info = (struct rt_device_rect_info *)args;
            LCD_Sync_fb(rect_info->x, rect_info->y, rect_info->width, rect_info->height);
        }
        case RTGRAPHIC_CTRL_GET_INFO:
        {
            struct rt_device_graphic_info *info = args;
            if(info == RT_NULL)
            {
                return -RT_ERROR;
            }
            info->pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565; /**< graphic format */
            info->bits_per_pixel = 16; /**< bits per pixel */
            info->pitch          = 0; /**< bytes per line */
            info->width          = lcddev.width; /**< width of graphic device */
            info->height         = lcddev.height; /**< height of graphic device */
            info->framebuffer    = (void *)frame_buf; /**< frame buffer */
            info->smem_len       = sizeof(frame_buf); /**< allocated frame buffer size */
        }
    }
    return RT_EOK;
}
#ifdef RT_USING_DEVICE_OPS
const struct rt_device_ops lcd_dev_ops =
{
    .init    = lcd_dev_ops_init   ,
    .open    = lcd_dev_ops_open   ,
    .close   = lcd_dev_ops_close  ,
    .read    = lcd_dev_ops_read   ,
    .write   = lcd_dev_ops_write  ,
    .control = lcd_dev_ops_control,
};
#endif
static int lcd_dev_init(void)
{
    #ifdef RT_USING_DEVICE_OPS
    lcddev.parent.ops = &lcd_dev_ops;
    #else
    lcddev.parent.init    = lcd_dev_ops_init   ;
    lcddev.parent.open    = lcd_dev_ops_open   ;
    lcddev.parent.close   = lcd_dev_ops_close  ;
    lcddev.parent.read    = lcd_dev_ops_read   ;
    lcddev.parent.write   = lcd_dev_ops_write  ;
    lcddev.parent.control = lcd_dev_ops_control;
    #endif
    lcddev.parent.type = RT_Device_Class_Graphic;
    lcddev.parent.user_data = &lcd_graphic_ops;
    rt_device_register(&lcddev.parent, "lcd", RT_DEVICE_FLAG_RDWR);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(lcd_dev_init);
static uint16_t color_array[] =
{
    WHITE, BLACK, RED, GREEN, BLUE, BRED,
    GRED, GBLUE, YELLOW
};

static int lcd_spi_test(int argc, char *argv[])
{
    rt_device_t lcd = rt_device_find("lcd");
    if(lcd)
    {
        uint8_t index = 0;
        rt_device_open(lcd, RT_DEVICE_FLAG_RDWR);
        if(rt_strcmp(argv[1], "clear") == 0)
        {
            for (index = 0; index < sizeof(color_array) / sizeof(color_array[0]); index++)
            {
                LCD_Clear(color_array[index]);
                LOG_I("lcd clear color: 0x%04x", color_array[index]);
                DELAY(2000);
            }
        }
        else if(rt_strcmp(argv[1], "line") == 0)
        {
            extern rt_uint32_t str2hex(const char *str);
            if(argc > 5)
            {
                int x = str2hex(argv[2]);
                int y = str2hex(argv[3]);
                int size = str2hex(argv[4]);
                volatile uint8_t buf[size*2];
                uint16_t color = str2hex(argv[5]);
                for(int i=0; i<size; i++)
                {
                    buf[i*2] = color>>8;
                    buf[i*2+1] = color&0xFF;
                }
                lcd_ops_blit_line((void *)buf, x, y, size);
            }
            else
            {
                rt_kprintf("<> line <x> <y> <size> <color>\r\n");
            }
        }
        rt_device_close(lcd);
    }
    else
    {
        LOG_E("lcd not found.");
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(lcd_spi_test, lcd will fill color => you need init lcd first);
#endif
