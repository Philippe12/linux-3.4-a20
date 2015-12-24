/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/time.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/gpio.h> 
#include <linux/init-input.h>
#include <linux/input/mt.h>

//FT5X02_CONFIG
#define FT5X02_CONFIG_NAME "fttpconfig_5x02public.ini"
extern int ft5x02_Init_IC_Param(struct i2c_client * client);
extern int ft5x02_get_ic_param(struct i2c_client * client);
extern int ft5x02_Get_Param_From_Ini(char *config_name);
        
struct i2c_dev{
        struct list_head list;	
        struct i2c_adapter *adap;
        struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

#define FT5X_NAME	"ft5x_ts"

static struct i2c_client *this_client;

/*********************************************************************************************/
#define CTP_IRQ_NUMBER                  (config_info.int_number)
#define CTP_IRQ_MODE			(TRIG_EDGE_NEGATIVE)
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define CTP_NAME			 FT5X_NAME
#define PRESS_MAX			(255)

static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static u32 int_handle = 0;
static __u32 twi_id = 0;
static bool is_suspend = false;

static struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
};

static u32 debug_mask = 0;
#define dprintk(level_mask,fmt,arg...)    if(unlikely(debug_mask & level_mask)) \
        printk("[CTP]:"fmt, ## arg)
module_param_named(debug_mask,debug_mask,int,S_IRUGO | S_IWUSR | S_IWGRP);
/*********************************************************************************************/
/*------------------------------------------------------------------------------------------*/        
/* Addresses to scan */
static const unsigned short normal_i2c[2] = {0x38,I2C_CLIENT_END};
static const int chip_id_value[] = {0x55,0x06,0x08,0x02,0xa3};
static int chip_id = 0;

static void ft5x_resume_events(struct work_struct *work);
struct workqueue_struct *ft5x_resume_wq;
static DECLARE_WORK(ft5x_resume_work, ft5x_resume_events);

static void ft5x_init_events(struct work_struct *work);
struct workqueue_struct *ft5x_wq;
static DECLARE_WORK(ft5x_init_work, ft5x_init_events);
/*------------------------------------------------------------------------------------------*/ 

static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = 0, i = 0; 
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if(twi_id == adapter->nr){
		ret = i2c_smbus_read_byte_data(client,0xA3);
		if(ret == -70) {
			msleep(10);
			ret = i2c_smbus_read_byte_data(client,0xA3);
		}
		dprintk(DEBUG_INIT,"addr:0x%x,chip_id_value:0x%x\n",client->addr,ret);
		while((chip_id_value[i++]) && (i <= sizeof(chip_id_value)/sizeof(chip_id_value[0]))){
			if(ret == chip_id_value[i - 1]){
				strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
				chip_id = ret;
				return 0;
			}                   
		}
        printk("%s:I2C connection might be something wrong ! \n",__func__);
        return -ENODEV;
	}else{
		return -ENODEV;
	}
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);
	
	list_for_each_entry(i2c_dev,&i2c_dev_list,list){
		dprintk(DEBUG_OTHERS_INFO,"--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",\
		        __LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index){
		     goto found;
		}
	}
	i2c_dev = NULL;
	
found: 
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap) 
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS){
		dprintk(DEBUG_OTHERS_INFO,"i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev){
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev;
}

static int ft5x_i2c_rxdata(char *rxdata, int length);

#define NB_TOUCH 5

struct ts_event {
	u16	x[NB_TOUCH];
	u16	y[NB_TOUCH];
	u16	pressure;
	s16 touch_ID[NB_TOUCH];
	u8 touch_state[NB_TOUCH];
	u8 touch_point;
	u8 touch_gesture;
};

struct ft5x_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};


/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit 

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

static void delay_ms(FTS_WORD  w_ms)
{
	//platform related, please implement this function
	msleep( w_ms );
}

/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_read_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret != dw_lenth){
		printk("ret = %d. \n", ret);
		printk("i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret != dw_lenth){
		printk("i2c_write_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]: 
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;        
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
u8 fts_register_read(u8 e_reg_name, u8* pbt_buf, u8 bt_len)
{
	u8 read_cmd[3]= {0};
	u8 cmd_len     = 0;

	read_cmd[0] = e_reg_name;
	cmd_len = 1;    

	/*call the write callback function*/
	//    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
	//    {
	//        return FTS_FALSE;
	//    }


	if(!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))	{//change by zhengdixu
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/        
	if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len)){
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

/*
[function]: 
    write a value to register.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[in]        :the returned register value;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int fts_register_write(u8 e_reg_name, u8 bt_value)
{
	FTS_BYTE write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = bt_value;

	/*call the write callback function*/
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, 2);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2); //change by zhengdixu
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, num);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);//change by zhengdixu
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_write(u8* pbt_buf, u16 dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_read(u8* pbt_buf, u8 bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
	//ft5x_i2c_rxdata
}

static int ft5x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		printk("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void ft5x_ts_release(void)
{
	int id;
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_report_key(data->input_dev, BTN_TOOL_FINGER, 0);
	input_mt_sync(data->input_dev);

	input_sync(data->input_dev);
	return;

}

static int ft5x_read_data(void)
{
	int id;
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	unsigned char buf[32]={0};
	int ret = -1;
        
	ret = ft5x_i2c_rxdata(buf, 31);
	if (ret < 0) {
		dprintk(DEBUG_X_Y_INFO,"%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[2] & 0x07;// 000 0111
	dprintk(DEBUG_X_Y_INFO,"touch point = %d\n",event->touch_point);

	event->touch_gesture = buf[1];
	dprintk(DEBUG_X_Y_INFO,"touch gesture = 0x%X\n",event->touch_gesture);

	if (event->touch_point == 0) {
		ft5x_ts_release();
		return 0;
	}

	for( id = 0; id < event->touch_point; id++ ){
		int offset = 6*id;
		event->x[id] = (s16)(buf[3+offset] & 0x0F)<<8 | (s16)buf[4+offset];
		event->y[id] = (s16)(buf[5+offset] & 0x0F)<<8 | (s16)buf[6+offset];
		dprintk(DEBUG_X_Y_INFO,"source data:event->x%d = %d, event->y%d = %d. \n", 
			id+1, event->x[id], id+1, event->y[id]);
		if(1 == exchange_x_y_flag){
			swap(event->x[id], event->y[id]);
		}
		if(1 == revert_x_flag){
			event->x[id] = SCREEN_MAX_X - event->x[id];
		}
		if(1 == revert_y_flag){
			event->y[id] = SCREEN_MAX_Y - event->y[id];
		}
		event->touch_ID[id]=(s16)(buf[5+offset] & 0xF0)>>4;
		event->touch_state[id] = (u8)(((buf[3+offset])>>6)&0x3);
		dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID[id]);
	}
	
	event->pressure = 30;
	return 0;
}

static void ft5x_report_multitouch(void)
{
	int id;
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	for( id = 0; id < event->touch_point; id++ ){
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID[id]);
		if( event->touch_state[id]>0 ){
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE, 200);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x[id]);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y[id]);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_state[id]>0);
		input_report_key(data->input_dev, BTN_TOOL_FINGER, event->touch_state[id]>0);
		}
		input_mt_sync(data->input_dev);
		dprintk(DEBUG_X_Y_INFO,"report data:===x%d = %d,y%d = %d, touch%d=%d ====\n",
				id+1,event->x[id],id+1,event->y[id],id+1,event->touch_state[id]);
	}
	
	input_sync(data->input_dev);
	return;
}

static void ft5x_report_value(void)
{
	ft5x_report_multitouch();
	return;
}	

static void ft5x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	ret = ft5x_read_data();
	if (ret == 0) {
		ft5x_report_value();
	}
	dprintk(DEBUG_INT_INFO,"%s:ret:%d\n",__func__,ret);
}

static u32 ft5x_ts_interrupt(struct ft5x_ts_data *ft5x_ts)
{
	dprintk(DEBUG_INT_INFO,"==========ft5x_ts TS Interrupt============\n"); 
	queue_work(ft5x_ts->ts_workqueue, &ft5x_ts->pen_event_work);
	return 0;
}

static void ft5x_resume_events (struct work_struct *work)
{
	int i = 0;
	ctp_wakeup(config_info.wakeup_number, 0, 20);

#ifdef CONFIG_HAS_EARLYSUSPEND	
	if(STANDBY_WITH_POWER_OFF != standby_level){
		goto standby_with_power_on; 
	}
#endif

	if(chip_id == 0x02 ){
		msleep(200);	/*wait...*/
		while(i<5){
			dprintk(DEBUG_INIT,"-----------------------------------------Init ic param\r\n");
			if (ft5x02_Init_IC_Param(this_client) >=0 ){
				dprintk(DEBUG_INIT,"---------------------------------------get ic param\r\n");
				if(ft5x02_get_ic_param(this_client) >=0)
					break;
			}
			i++;
		}
	}
standby_with_power_on:
	sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,1);
}



static int ft5x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	dprintk(DEBUG_SUSPEND,"==ft5x_ts_suspend=\n");
	dprintk(DEBUG_SUSPEND,"CONFIG_PM: write FT5X0X_REG_PMODE .\n");
	
	is_suspend = false; 
	
	flush_workqueue(ft5x_resume_wq);
	sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,0);
	cancel_work_sync(&data->pen_event_work);
	flush_workqueue(data->ts_workqueue);
	ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	
	return 0;
}
static int ft5x_ts_resume(struct i2c_client *client)
{
	dprintk(DEBUG_SUSPEND,"==CONFIG_PM:ft5x_ts_resume== \n");
	if(is_suspend == false)
	        queue_work(ft5x_resume_wq, &ft5x_resume_work);
	        
	return 0;		
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	dprintk(DEBUG_SUSPEND,"==ft5x_ts_suspend=\n");
	dprintk(DEBUG_SUSPEND,"CONFIG_HAS_EARLYSUSPEND: write FT5X0X_REG_PMODE .\n");
	
#ifndef CONFIG_HAS_EARLYSUSPEND
        is_suspend = true;
#endif  
        if(is_suspend == true){ 
	        flush_workqueue(ft5x_resume_wq);
	        sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,0);
	        cancel_work_sync(&data->pen_event_work);
	        flush_workqueue(data->ts_workqueue);
	        ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	}
	
	is_suspend = true;
}

static void ft5x_ts_late_resume(struct early_suspend *handler)
{
	dprintk(DEBUG_SUSPEND,"==CONFIG_HAS_EARLYSUSPEND:ft5x_ts_resume== \n");
	
	queue_work(ft5x_resume_wq, &ft5x_resume_work);	
	is_suspend = true;
}
#endif

static void ft5x_init_events (struct work_struct *work)
{
	int i = 0;
	int ret; 
	dprintk(DEBUG_INIT,"====%s begin=====.  \n", __func__);

	while(chip_id == 0xa3){
		delay_ms(5);
		ret = i2c_smbus_read_byte_data(this_client,0xA3);
        	dprintk(DEBUG_INIT,"addr:0x%x,chip_id_value:0x%x\n",this_client->addr,ret);
		if(ret != 0xa3) {
			chip_id = ret;
			break;
		}
		if((i++)>10) {
			break;
		}	
	}
	dprintk(DEBUG_INIT,"read chip_id timers,timers=%d\n",i);
	i = 0;

	if(chip_id == 0x02 ){
		msleep(1000);	/*wait...*/
		while(i<5){
			dprintk(DEBUG_INIT,"-----------------------------------------Init ic param\r\n");
			if (ft5x02_Init_IC_Param(this_client) >=0 ){
				dprintk(DEBUG_INIT,"---------------------------------------get ic param\r\n");
				if(ft5x02_get_ic_param(this_client) >=0)
					break;
			}
			i++;
		}
	}
}

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x_ts_data *ft5x_ts;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int err = 0;       

	dprintk(DEBUG_INIT,"====%s begin=====.  \n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk("check_functionality_failed\n");
		goto exit_check_functionality_failed;
	}

	ft5x_ts = kzalloc(sizeof(*ft5x_ts), GFP_KERNEL);
	if (!ft5x_ts)	{
		err = -ENOMEM;
		printk("alloc_data_failed\n");
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ft5x_ts);

	ft5x_wq = create_singlethread_workqueue("ft5x_init");
	if (ft5x_wq == NULL) {
		printk("create ft5x_wq fail!\n");
		return -ENOMEM;
	}

	queue_work(ft5x_wq, &ft5x_init_work);

	INIT_WORK(&ft5x_ts->pen_event_work, ft5x_ts_pen_irq_work);
	ft5x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x_ts->ts_workqueue) {
		err = -ESRCH;
		printk("ts_workqueue fail!\n");
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x_ts->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);	
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(ABS_PRESSURE, input_dev->absbit);	
	set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);	
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name	= CTP_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,"ft5x_ts_probe: failed to register input device: %s\n",
		        dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	ft5x_resume_wq = create_singlethread_workqueue("ft5x_resume");
	if (ft5x_resume_wq == NULL) {
		printk("create ft5x_resume_wq fail!\n");
		return -ENOMEM;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x_ts->early_suspend.suspend = ft5x_ts_early_suspend;
	ft5x_ts->early_suspend.resume	= ft5x_ts_late_resume;
	register_early_suspend(&ft5x_ts->early_suspend);
#endif

	dprintk(DEBUG_INIT,"CONFIG_FT5X0X_MULTITOUCH is defined. \n");
    int_handle = sw_gpio_irq_request(CTP_IRQ_NUMBER,CTP_IRQ_MODE,(peint_handle)ft5x_ts_interrupt,ft5x_ts);
	if (!int_handle) {
		printk("ft5x_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
	ctp_set_int_port_rate(config_info.int_number, 1);
	ctp_set_int_port_deb(config_info.int_number, 0x07);
	dprintk(DEBUG_INIT,"reg clk: 0x%08x\n", readl(0xf1c20a18));

    	i2c_dev = get_free_i2c_dev(client->adapter);	
	if (IS_ERR(i2c_dev)){	
		err = PTR_ERR(i2c_dev);	
		printk("i2c_dev fail!");	
		return err;	
	}
	
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,client->adapter->nr),
	         NULL, "aw_i2c_ts%d", client->adapter->nr);	
	if (IS_ERR(dev))	{		
			err = PTR_ERR(dev);
			printk("dev fail!\n");		
			return err;	
	}

	device_enable_async_suspend(&client->dev);
	dprintk(DEBUG_INIT,"==%s over =\n", __func__);

	return 0;

exit_irq_request_failed:
        sw_gpio_irq_free(int_handle);
        cancel_work_sync(&ft5x_resume_work);
	destroy_workqueue(ft5x_resume_wq);	
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
        i2c_set_clientdata(client, NULL);
        cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
exit_create_singlethread:
	kfree(ft5x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:        
	cancel_work_sync(&ft5x_init_work);
	destroy_workqueue(ft5x_wq);

	return err;
}

static int __devexit ft5x_ts_remove(struct i2c_client *client)
{

	struct ft5x_ts_data *ft5x_ts = i2c_get_clientdata(client);
	ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	
	printk("==ft5x_ts_remove=\n");
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR,client->adapter->nr));
	sw_gpio_irq_free(int_handle);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x_ts->early_suspend);
#endif
	cancel_work_sync(&ft5x_resume_work);
	destroy_workqueue(ft5x_resume_wq);
	input_unregister_device(ft5x_ts->input_dev);
	input_free_device(ft5x_ts->input_dev);
	cancel_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	kfree(ft5x_ts);
    
	i2c_set_clientdata(this_client, NULL);

	return 0;

}

static const struct i2c_device_id ft5x_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);

static struct i2c_driver ft5x_ts_driver = {
	.class          = I2C_CLASS_HWMON,
	.probe		= ft5x_ts_probe,
	.remove		= __devexit_p(ft5x_ts_remove),
	.id_table	= ft5x_ts_id,
	.suspend        = ft5x_ts_suspend,
	.resume         = ft5x_ts_resume,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= normal_i2c,

};

static int aw_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;	
	struct i2c_client *client;
	struct i2c_adapter *adapter;	
	struct i2c_dev *i2c_dev;	

	printk("====%s======.\n", __func__);
	dprintk(DEBUG_OTHERS_INFO,"enter aw_open function\n");
	subminor = iminor(inode);
	dprintk(DEBUG_OTHERS_INFO,"subminor=%d\n",subminor);
		
	i2c_dev = i2c_dev_get_by_minor(2);	
	if (!i2c_dev)	{	
		printk("error i2c_dev\n");		
		return -ENODEV;	
	}	
	adapter = i2c_get_adapter(i2c_dev->adap->nr);	
	if (!adapter)	{		
		return -ENODEV;	
	}	
	
	client = kzalloc(sizeof(*client), GFP_KERNEL);	
	
	if (!client)	{		
		i2c_put_adapter(adapter);		
		ret = -ENOMEM;	
	}	
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	client->driver = &ft5x_ts_driver;
	client->adapter = adapter;		
	file->private_data = client;
		
	return 0;
}

static long aw_ioctl(struct file *file, unsigned int cmd,unsigned long arg ) 
{
	dprintk(DEBUG_OTHERS_INFO,"====%s====\n",__func__);
	dprintk(DEBUG_OTHERS_INFO,"line :%d,cmd = %d,arg = %ld.\n",__LINE__,cmd,arg);
	
	switch (cmd) {
	default:
		break;			 
	}	
	return 0;
}

static int aw_release (struct inode *inode, struct file *file) 
{
	struct i2c_client *client = file->private_data;
	dprintk(DEBUG_OTHERS_INFO,"enter aw_release function.\n");		
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;	  
}

static const struct file_operations aw_i2c_ts_fops ={	
	.owner = THIS_MODULE, 		
	.open = aw_open, 	
	.unlocked_ioctl = aw_ioctl,	
	.release = aw_release, 
};
static int ctp_get_system_config(void)
{  
        twi_id = config_info.twi_id;
        screen_max_x = config_info.screen_max_x;
        screen_max_y = config_info.screen_max_y;
        revert_x_flag = config_info.revert_x_flag;
        revert_y_flag = config_info.revert_y_flag;
        exchange_x_y_flag = config_info.exchange_x_y_flag;
        if((twi_id == 0) || (screen_max_x == 0) || (screen_max_y == 0)){
                printk("%s:read config error!\n",__func__);
                return 0;
        }
        return 1;
}
static int __init ft5x_ts_init(void)
{ 
	int ret = -1;      
	dprintk(DEBUG_INIT,"***************************init begin*************************************\n");
	if (input_fetch_sysconfig_para(&(config_info.input_type))) {
		printk("%s: ctp_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(config_info.input_type));
		if (0 != ret) {
			printk("%s:ctp_ops.init_platform_resource err. \n", __func__);    
		}
	}	
	
	if(config_info.ctp_used == 0){
	        printk("*** ctp_used set to 0 !\n");
	        printk("*** if use ctp,please put the sys_config.fex ctp_used set to 1. \n");
	        return 0;
	}
	
        if(!ctp_get_system_config()){
                printk("%s:read config fail!\n",__func__);
                return ret;
        }

	ctp_wakeup(config_info.wakeup_number, 0, 20);
	//msleep(80);
	ft5x_ts_driver.detect = ctp_detect;

	ret= register_chrdev(I2C_MAJOR,"aw_i2c_ts",&aw_i2c_ts_fops );	
	if(ret) {	
		printk("%s:register chrdev failed\n",__FILE__);	
		return ret;
	}
	
	i2c_dev_class = class_create(THIS_MODULE,"aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {		
		ret = PTR_ERR(i2c_dev_class);		
		class_destroy(i2c_dev_class);	
	}
        ret = i2c_add_driver(&ft5x_ts_driver);
        
        dprintk(DEBUG_INIT,"****************************init end************************************\n");
	return ret;
}

static void __exit ft5x_ts_exit(void)
{
	printk("==ft5x_ts_exit==\n");
	i2c_del_driver(&ft5x_ts_driver);
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR, "aw_i2c_ts");
	input_free_platform_resource(&(config_info.input_type));
}

late_initcall(ft5x_ts_init);
module_exit(ft5x_ts_exit);
MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");

