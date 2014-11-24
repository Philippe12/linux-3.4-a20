/* drivers/input/touchscreen/gt818x.h
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.2
 * Revision record:
 *      V1.0:2012/06/08,create file.
 *      V1.4:2012/09/20,G868 sensor ID & coor key suppoert
 */

#ifndef _LINUX_GOODIX_TOUCH_H
#define	_LINUX_GOODIX_TOUCH_H

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
//#include <mach/gpio.h>
#include <linux/earlysuspend.h>

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/delay.h>
 
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>

#include <asm/io.h>

#include <mach/gpio.h> 
#include <linux/gpio.h>
#include <linux/init-input.h>
#include <linux/pm.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif
struct goodix_ts_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    struct early_suspend early_suspend;
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  green_wake_mode;
    u8  chip_type;
    u8  enter_update;
    u8  gtp_is_suspend;
    u8  gtp_rawdiff_mode;
    u8  driver_cnt;
    u8  sensor_cnt;
    u8  coor_div_2;
};

enum CHIP_TYPE
{
    GT818X,
    GT868
};

extern u16 show_len;
extern u16 total_len;
void gtp_set_io_int(void);
void gtp_set_int_value(int status);

//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  0
#define GTP_AUTO_UPDATE       1
#define GTP_CHANGE_X2Y        1
#define GTP_ESD_PROTECT       0
#define GTP_CREATE_WR_NODE    1
#define GTP_ICS_SLOT_REPORT   0
#define GTP_USE_868_968M      0
#define GUP_USE_HEADER_FILE   1

#define GTP_DEBUG_ON          0
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

//***************************PART2:TODO define**********************************
//STEP_1(REQUIRED):Change config table.
/*TODO: puts the config info corresponded to your TP here, the following is just 
a sample config, send this config should cause the chip cannot work normally*/
//yiju dpt 800*480
#define CTP_CFG_GROUP1 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x02,0x22,0x12,0x22,0x22,0x22,0x32,0x22,0x42,0x22,0x52,0x22,0x62,0x22,0x72,0x22,0x82,0x22,0x92,0x22,0xA2,0x22,0xB2,0x22,0xC2,0x22,0xD2,0x22,0xE2,0x22,0xF2,0x22,0x07,0x03,0xE0,0xE0,0xE0,0x19,0x19,0x19,0x10,0x10,0x0A,0x40,0x30,0x05,0x03,0x00,0x05,0xE0,0x01,0x20,0x03,0x00,0x00,0x38,0x38,0x35,0x30,0x00,0x00,0x05,0x19,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}

//TODO puts your group2 config info here,if need.
//VDDIO
//chuanzuoyue 800*480
#define CTP_CFG_GROUP2 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x01,0x11,0x11,0x11,0x21,0x11,0x31,0x11,0x41,0x11,0x51,0x11,0x61,0x11,0x71,0x11,0x81,0x11,0x91,0x11,0xA1,0x11,0xB1,0x11,0xC1,0x11,0xD1,0x11,0xE1,0x11,0xF1,0x11,0x07,0x03,0xE0,0xE0,0xE0,0x1A,0x1A,0x1A,0x10,0x10,0x0A,0x40,0x30,0x05,0x03,0x00,0x05,0xE0,0x01,0x20,0x03,0x00,0x00,0x38,0x38,0x35,0x30,0x00,0x00,0x03,0x19,0x05,0x08,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}

//chuyi 800*480
#define CTP_CFG_GROUP3 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x02,0x22,0x12,0x22,0x22,0x22,0x32,0x22,0x42,0x22,0x52,0x22,0x62,0x22,0x72,0x22,0x82,0x22,0x92,0x22,0xA2,0x22,0xB2,0x22,0xC2,0x22,0xD2,0x22,0xE2,0x22,0xF2,0x22,0x37,0x13,0x80,0x80,0x80,0x13,0x13,0x13,0x10,0x10,0x0A,0x40,0x30,0x45,0x03,0x00,0x05,0xE0,0x01,0x20,0x03,0x00,0x00,0x38,0x38,0x35,0x35,0x00,0x00,0x25,0x19,0x25,0x0F,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}


//TODO puts your group4 config info here,if need.
//yiju gao qin
#define CTP_CFG_GROUP4 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x02,0x22,0x12,0x22,0x22,0x22,0x32,0x22,0x42,0x22,0x52,0x22,0x62,0x22,0x72,0x22,0x82,0x22,0x92,0x22,0xA2,0x22,0xB2,0x22,0xC2,0x22,0xD2,0x22,0xE2,0x22,0xF2,0x22,0x07,0x03,0xE0,0xE0,0xE0,0x19,0x19,0x19,0x10,0x10,0x0A,0x40,0x30,0x05,0x03,0x00,0x05,0x58,0x02,0x00,0x04,0x00,0x00,0x38,0x38,0x35,0x30,0x00,0x00,0x05,0x19,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}

//chuanzuoyue gao qin
#define CTP_CFG_GROUP5 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x01,0x11,0x11,0x11,0x21,0x11,0x31,0x11,0x41,0x11,0x51,0x11,0x61,0x11,0x71,0x11,0x81,0x11,0x91,0x11,0xA1,0x11,0xB1,0x11,0xC1,0x11,0xD1,0x11,0xE1,0x11,0xF1,0x11,0x07,0x03,0xE0,0xE0,0xE0,0x1A,0x1A,0x1A,0x10,0x10,0x0A,0x40,0x30,0x05,0x03,0x00,0x05,0x58,0x02,0x00,0x04,0x00,0x00,0x38,0x38,0x35,0x30,0x00,0x00,0x03,0x19,0x05,0x08,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}

//chuyi gaoqin 
#define CTP_CFG_GROUP6 {\
0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x02,0x22,0x12,0x22,0x22,0x22,0x32,0x22,0x42,0x22,0x52,0x22,0x62,0x22,0x72,0x22,0x82,0x22,0x92,0x22,0xA2,0x22,0xB2,0x22,0xC2,0x22,0xD2,0x22,0xE2,0x22,0xF2,0x22,0x37,0x13,0x80,0x80,0x80,0x13,0x13,0x13,0x10,0x10,0x0A,0x40,0x30,0x45,0x03,0x00,0x05,0x58,0x02,0x00,0x04,0x00,0x00,0x38,0x38,0x35,0x35,0x00,0x00,0x25,0x19,0x25,0x0F,0x00,0x00,0x00,0x00,0x00,0x14,0x10,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x20,0x30,0x22,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x01}


//STEP_2(REQUIRED):Change I/O define & I/O operation mode.
#define GTP_RST_PORT    0//S5PV210_GPJ3(6)
#define GTP_INT_PORT    1//S5PV210_GPH1(3)
#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)

#define GTP_GPIO_AS_INPUT(pin)          /*do{\
                                            gpio_direction_input(pin);\
                                            s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);\
                                        }while(0)*/
#define GTP_GPIO_AS_INT(pin)           /* do{\
                                       
                                        }while(0)*/
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)     /* __gpio_set_value(pin, level);*/
#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)              gpio_free(pin)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_FALLING,IRQ_TYPE_EDGE_RISING}

//STEP_3(optional):Custom set some config by themself,if need.
#if GTP_CUSTOM_CFG
  #define GTP_MAX_HEIGHT   800			
  #define GTP_MAX_WIDTH    480
  #define GTP_INT_TRIGGER  1    //0:Falling 1:Rising
#else
  #define GTP_MAX_HEIGHT   800
  #define GTP_MAX_WIDTH    480
  #define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH      5
#define GTP_ESD_CHECK_CIRCLE  2000

//STEP_4(optional):If this project have touch key,Set touch key config.
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB	 {KEY_HOME, KEY_MENU}
    //define your key_center & key_width\key_height here
    #if GTP_USE_868_968M
        #define KEY_CNTR_PNT_X 	      {216, 246}
        #define KEY_CNTR_PNT_Y 	      {856, 856}
        #define KEY_AREA_HEIGHT_H     10
        #define KEY_AREA_WIDTH_H      20
    #endif
#endif

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION    "V1.4<2012/09/20>"
#define GTP_I2C_NAME          "gt818_ts"
#define GTP_POLL_TIME         10
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_LENGTH     106
#define FAIL                  0
#define SUCCESS               1

//Register define
#define GTP_REG_CONFIG_DATA   0x6A2
#define GTP_REG_INDEX         0x712
#define GTP_REG_SLEEP         0x692
#define GTP_REG_SENSOR_ID     0x710
#define GTP_REG_VERSION       0x715
#define GTP_READ_COOR_ADDR    0x721

#define DRVCNT_LOC            51
#define RESOLUTION_LOC        61
#define TRIGGER_LOC           57

//Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->>[%d]"fmt"\n", __LINE__, ##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->>[%d]"fmt"\n",__LINE__, ##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->>[%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->>[%d]Func:%s\n", __LINE__, __func__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)
#define ABS_VAL(x)                     ((x < 0) ? -x : x)
//****************************PART4:UPDATE define*******************************
#define PACK_SIZE            64                    //update file package size
#define SEARCH_FILE_TIMES    50
#define UPDATE_FILE_PATH_2   "/data/goodix/_goodix_update_.bin"
#define UPDATE_FILE_PATH_1   "/sdcard/goodix/_goodix_update_.bin"

//Error no
#define ERROR_NO_FILE           2   //ENOENT
#define ERROR_FILE_READ         23  //ENFILE
#define ERROR_FILE_TYPE         21  //EISDIR
#define ERROR_GPIO_REQUEST      4   //EINTR
#define ERROR_I2C_TRANSFER      5   //EIO
#define ERROR_NO_RESPONSE       16  //EBUSY
#define ERROR_TIMEOUT           110 //ETIMEDOUT

//*****************************End of Part III********************************

#endif /* _LINUX_GOODIX_TOUCH_H */