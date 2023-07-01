# ST7789

## 简介

本软件包是 TFT-LCD-ST7789 SPI接口屏幕的驱动包，本软件包已经对接到 SPI 框架。通过 SPI 框架 API，开发者可以快速的将此屏幕驱动起来。

## 使用说明

### 依赖

- RT-Thread 5.0.0+
- SPI 驱动：ST7789 屏幕使用 SPI 进行数据通讯，需要系统SPI 驱动支持；

### 获取软件包

使用ST7789 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```shell
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    TFT-LCD ST7789 SPI screen driver software package --->
        (spi0) spi bus name
        (spi00) spi device name
        (240) Width of the LCD display
        (320) Height of the LCD display
        (-1) DC pin connected to the LCD display
    (-1) RESET pin connected to the LCD display
    (-1) CS pin connected to the LCD display
    (-1) Backlight pin connected to the LCD display
        Version (latest)  --->
```

**spi bus name**：连接 LCD 所使用的 SPI 总线名称

**spi device name**：连接 LCD 所使用的 SPI 设备名称

**Width of the LCD display**： LCD  屏幕宽度参数

**Height of the LCD display**： LCD 屏幕高度参数

**DC pin connected to the LCD display**： LCD 屏幕数据引脚

**RESET pin connected to the LCD display**： LCD 屏幕复位引脚

**CS pin connected to the LCD display**： LCD 屏幕片选引脚（若硬件无CS引脚则填写 -1）

**Backlight pin connected to the LCD display**： LCD 屏幕背光引脚

### 使用软件包

ST7789 软件包初始化函数如下所示：

```c
rt_err_t spi_lcd_init(uint32_t freq)
```

该函数需要由用户调用，函数主要完成的功能有：

- 设备配置和初始化（根据传入的配置信息，配置 SPI 的频率，初始化 LCD 参数）；
- 注册相应的传感器设备，完成 SPI 设备的注册；

> 参数：SPI 的频率。例如：单片机的 SPI 外设频率为 20 MHZ，则填写 20

```c
void lcd_fill_array_spi(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, void *Image)
```

该函数需要由用户调用，函数主要完成的功能有：

- 填充颜色数组到 LCD 显存

> 参数：要填充的x起始坐标，y起始坐标，x终止坐标，y终止坐标，颜色数组

#### RTT图形设备操作示例

```c
#include "rtthread.h"

static void lcd_fill_color(rt_device_t device, rt_uint16_t color)
{
    struct rt_device_graphic_info info;
    if(RT_EOK != rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &info))
    {
        return ;
    }
    rt_uint16_t *fb = info.framebuffer;
    if(fb == RT_NULL)
    {
        return ;
    }
    for(int i=0; i<info.width*info.height; i++)
    {
        fb[i] = 0x001F;
    }
    struct rt_device_rect_info rect_info =
    {
        .x = 0, .y = 0,
        .width = info.width,
        .height = info.height,
    };
    rt_device_control(device, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);
}
/**
 * @brief lcd test.
 *
 * @return 0=success, -1=failed
*/
int st7789_test(void)
{
    rt_device_t device = rt_device_find("lcd");
    RT_ASSERT(device != RT_NULL);
    if (rt_device_open(device, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        return -1;
    }
    /* LCD fill blue color */
    lcd_fill_color(device, 0x001F);
    rt_thread_mdelay(1000);
    /* LCD fill red color */
    lcd_fill_color(device, 0xF800);
    return 0;
}
MSH_CMD_EXPORT(st7789_test, lcd will fill color.);
```

## 注意事项

暂无

## 联系人信息

维护人:

- [vandoul](https://github.com/Vandoul)
- 主页：[tft lcd st7789 rtthread graphic device](https://github.com/Vandoul/ST7789)
