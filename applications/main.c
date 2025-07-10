/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-5-10      ShiHao       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_lcd.h>
#include <rttlogo.h>
#include "ap3216c.h"
#include "mg90s.h"
#include <drv_common.h>
#include <stdio.h>
#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>


/* 配置蜂鸣器引脚 */
#define PIN_BEEP        GET_PIN(B, 0)      // PA1:  BEEP         --> BEEP (PB1)

/* 配置 LED 灯引脚 */
#define PIN_LED_B              GET_PIN(F, 11)      // PF11 :  LED_B        --> LED
#define PIN_LED_R              GET_PIN(F, 12)      // PF12 :  LED_R        --> LED

/* 电机1引脚 1 2 */
#define PIN_MOTOR_1        GET_PIN(A, 5)      // PA5:
#define PIN_MOTOR_2        GET_PIN(A, 6)      // PA6:

/* 电机1引脚 1 2 */
#define PIN_MOTOR_3        GET_PIN(A, 4)      // PA4:
#define PIN_MOTOR_4        GET_PIN(A, 7)      // PA7:

#define fan_P     GET_PIN(D, 12)      // PD12

/* 电机1 电机2配置引脚 输出模式 */
static void Motor_Iint_all()
{

    /* 设置电机1引脚为输出模式 */
    rt_pin_mode(PIN_MOTOR_1, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_MOTOR_2, PIN_MODE_OUTPUT);

    /* 设置电机2引脚为输出模式 */
    rt_pin_mode(PIN_MOTOR_3, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_MOTOR_4, PIN_MODE_OUTPUT);
}
/* 电机1 控制函数: x :1 电机正转，2 电机反转 0电机停止*/
static void Motor_Run_SYS1(int x)
{
    if (x ==1) {
        /*电机1正转打开 */
       rt_pin_write(PIN_MOTOR_1,PIN_HIGH);
       rt_pin_write(PIN_MOTOR_2,PIN_LOW);
    }
    else if(x ==2) {

        /*电机1反转转打开 */
      rt_pin_write(PIN_MOTOR_1,PIN_LOW);
      rt_pin_write(PIN_MOTOR_2,PIN_HIGH);
    }else{

        /*电机1停止 */
         rt_pin_write(PIN_MOTOR_1,PIN_LOW);
         rt_pin_write(PIN_MOTOR_2,PIN_LOW);
    }

}

/* 电机2 控制函数: x :1 电机正转，2 电机反转 0电机停止*/
static void Motor_Run_SYS2(int x)
{
    if (x ==1) {
        /*电机2正转打开 */
       rt_pin_write(PIN_MOTOR_3,PIN_HIGH);
       rt_pin_write(PIN_MOTOR_4,PIN_LOW);
    }
    else if(x ==2) {

        /*电机2反转转打开 */
      rt_pin_write(PIN_MOTOR_3,PIN_LOW);
      rt_pin_write(PIN_MOTOR_4,PIN_HIGH);
    }else{

        /*电机2停止 */
         rt_pin_write(PIN_MOTOR_3,PIN_LOW);
         rt_pin_write(PIN_MOTOR_4,PIN_LOW);
    }

}


/* aht 20 温湿度传感器驱动 */

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;
#define AHT_I2C_BUS_NAME    "i2c3"  //AHT20 挂载的I2C总线
#define AHT_ADDR            0x38    //AHT20 I2C地址
#define AHT_CALIBRATION_CMD 0xBE    //AHT20 初始化命令
#define AHT_NORMAL_CMD      0xA8    //AHT20 正常工作模式命令
#define AHT_GET_DATA_CMD    0xAC    //AHT20 获取结果命令

//I2C_BUS设备指针，用于等会寻找与记录AHT挂载的I2C总线
//struct rt_i2c_bus_device *i2c_bus = RT_NULL;

//AHT命令的空参数
rt_uint8_t Parm_Null[2]={0,0};

//写命令（主机向从机传输数据）
rt_err_t write_reg(struct rt_i2c_bus_device *Device, rt_uint8_t reg, rt_uint8_t* data)
{
    //代写入的数据
    //数组大小为3的原因：buf[0]--命令（即上面的AHT_CALIBRATION_CMD、AHT_NORMAL_CMD、AHT_GET_DATA_CMD
    //                  buf[1]/buf[2]为命令后跟的参数，AHT有些命令后面需要加上参数，具体可查看数据手册
    rt_uint8_t buf[3];

    //记录数组大小
    rt_uint8_t buf_size;

    //I2C传输的数据结构体
    struct rt_i2c_msg msgs;

    buf[0] = reg;
    if(data != RT_NULL)
    {
        buf[1] = data[0];
        buf[2] = data[1];
        buf_size = 3;
    }
    else
    {
        buf_size = 1;
    }

    msgs.addr = AHT_ADDR;   //消息要发送的地址：即AHT地址
    msgs.flags = RT_I2C_WR; //消息的标志位：读还是写，是否需要忽视ACK回应，是否需要发送停止位，是否需要发送开始位(用于拼接数据使用)...
    msgs.buf = buf;         //消息的缓冲区：待发送/接收的数组
    msgs.len = buf_size;    //消息的缓冲区大小：待发送/接收的数组的大小

    //传输信息
    //这里i2c.core层提供给我们三个API去进行I2C的数据传递：
    /*
     * 1.发送API
     * rt_size_t rt_i2c_master_send(struct rt_i2c_bus_device *bus,
                             rt_uint16_t               addr,
                             rt_uint16_t               flags,
                             const rt_uint8_t         *buf,
                             rt_uint32_t               count)

       2.接收API
       rt_size_t rt_i2c_master_recv(struct rt_i2c_bus_device *bus,
                             rt_uint16_t               addr,
                             rt_uint16_t               flags,
                             rt_uint8_t               *buf,
                             rt_uint32_t               count)
       3.传输API
       rt_size_t rt_i2c_transfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg         msgs[],
                          rt_uint32_t               num)
      * 实际上1跟2最后都会调用回3，大家可以按照自己需求进行调用
    */
    if(rt_i2c_transfer(Device, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

//读数据（从机向主机返回数据）
rt_err_t read_reg(struct rt_i2c_bus_device *Device, rt_uint8_t len, rt_uint8_t* buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr = AHT_ADDR;       //消息要发送的地址：即AHT地址
    msgs.flags = RT_I2C_RD;     //消息的标志位：读还是写，是否需要忽视ACK回应，是否需要发送停止位，是否需要发送开始位(用于拼接数据使用)...
    msgs.buf = buf;             //消息的缓冲区：待发送/接收的数组
    msgs.len = len;             //消息的缓冲区大小：待发送/接收的数组的大小

    //传输函数，上面有介绍
    if(rt_i2c_transfer(Device, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

//读取AHT的温湿度数据
void read_temp_humi(float* Temp_Data, float* Humi_Data)
{
    //根据数据手册我们可以看到要读取一次数据需要使用到的数组大小为6
    rt_uint8_t Data[6];


    write_reg(i2c_bus, AHT_GET_DATA_CMD, Parm_Null);      //发送一个读取命令，让AHT进行一次数据采集
    rt_thread_mdelay(500);                          //等待采集
    read_reg(i2c_bus, 6, Data);                     //读取数据

    //根据数据手册进行数据处理
    *Humi_Data = (Data[1] << 12 | Data[2] << 4 | (Data[3] & 0xf0) >> 4) * 100.0 / (1 << 20);
    *Temp_Data = ((Data[3] & 0x0f) << 16 | Data[4] << 8 | Data[5]) * 200.0 / (1 << 20) - 50;
}

//AHT进行初始化
void AHT_Init(const char* name)
{
    //寻找AHT的总线设备
    i2c_bus = rt_i2c_bus_device_find(name);

    if(i2c_bus == RT_NULL)
    {
        rt_kprintf("Can't Find I2C_BUS Device");    //找不到总线设备
    }
    else
    {
        write_reg(i2c_bus, AHT_NORMAL_CMD, Parm_Null);    //设置为正常工作模式
        rt_thread_mdelay(400);

        rt_uint8_t Temp[2];     //AHT_CALIBRATION_CMD需要的参数
        Temp[0] = 0x08;
        Temp[1] = 0x00;
        write_reg(i2c_bus, AHT_CALIBRATION_CMD, Temp);
        rt_thread_mdelay(400);
    }

}

//AHT设备测试线程
void AHT_test(void)
{
    float humidity, temperature;

    AHT_Init(AHT_I2C_BUS_NAME);     //进行设备初始化

    while(1)
    {
        read_temp_humi(&temperature, &humidity);    //读取数据
        rt_kprintf("humidity   : %d.%d %%\n", (int)humidity, (int)(humidity * 10) % 10);
        rt_kprintf("temperature: %d.%d\n", (int)temperature, (int)(temperature * 10) % 10);
        rt_thread_mdelay(1000);
    }
}
MSH_CMD_EXPORT(AHT_test, AHT_test);     //将命令到出到MSH列表


#define PWM_DEV_NAME "pwm2"  // 根据实际PWM设备修改
#define PWM_CHANNEL  3       // 使用通道1

struct rt_device_pwm *pwm_dev;

/* 初始化PWM */
int pwm_servo_init(void)
{
    /* 查找PWM设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL) {
        rt_kprintf("Cannot find %s device!\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    /* 设置PWM周期为20ms(50Hz)，舵机标准周期 */
    rt_pwm_set(pwm_dev, PWM_CHANNEL, 20000000, 1500000);  // 初始位置1.5ms(90度)

    /* 使能PWM */
    rt_pwm_enable(pwm_dev, PWM_CHANNEL);

    return RT_EOK;
}

/* 设置舵机角度(0-180度) */
void servo_set_angle(int angle)
{
    rt_uint32_t pulse;

    /* 限制角度范围 */
    angle = angle > 180 ? 180 : (angle < 0 ? 0 : angle);

    /* 将角度转换为脉宽(500-2500us) */
    pulse = 500000 + angle * (2000000 / 180);

    /* 设置PWM(周期20ms不变) */
    rt_pwm_set(pwm_dev, PWM_CHANNEL, 20000000, pulse);
}

/* 测试命令 */
static void servo_test(int argc, char *argv[])
{
    if (argc != 2) {
        rt_kprintf("Usage: servo_test [angle(0-180)]\n");
        return;
    }

    servo_set_angle(atoi(argv[1]));
}
MSH_CMD_EXPORT(servo_test, servo_set_angle);



#define HCSR501_PIN_PORT "PA8"  // 根据实际连接端口修改

//#define HCSR501_PIN 0  // 实际连接的引脚号
#define HCSR501_PIN        GET_PIN(D, 8)      // PC1:  KEY1         --> KEY
static rt_base_t hcsr501_pin =NULL;

/* 初始化HC-SR501 */
int hcsr501_init(void)
{
//    /* 获取引脚编号 */
//    hcsr501_pin = rt_pin_get("PA8");
//    if (hcsr501_pin == NULL) {
//        rt_kprintf("Get pin failed!\n");
//        return -RT_ERROR;
//    }
    /* 设置按键引脚为输入模式 */
    //rt_pin_mode(PIN_KEY1, PIN_MODE_INPUT_PULLUP);
    /* 设置为输入模式 */
    rt_pin_mode(HCSR501_PIN, PIN_MODE_INPUT_PULLDOWN);

    rt_kprintf("HC-SR501 init success!\n");
    return RT_EOK;
}
INIT_APP_EXPORT(hcsr501_init);  // 自动初始化

/* 读取人体检测状态 */
int hcsr501_read(void)
{
    if (HCSR501_PIN == NULL) {
        rt_kprintf("Device not initialized!\n");
        return -1;
    }

    return rt_pin_read(HCSR501_PIN);
}







/*
 * 程序清单：这是一个 PWM 设备使用例程
 * 例程导出了 pwm_led_sample 命令到控制终端
 * 命令调用格式：pwm_led_sample
 * 程序功能：通过 PWM 设备控制 LED 灯的亮度，可以看到LED不停的由暗变到亮，然后又从亮变到暗。
*/

#include <rtthread.h>
#include <rtdevice.h>

#define PWM_DEV_NAME        "pwm2"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     3       /* PWM通道 */

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

static int pwm_sg90_sample(int argc, char *argv[])
{
    rt_uint32_t period, pulse, dir;

    period = 500000;    /* 周期为0.5ms，单位为纳秒ns */
    dir = 1;            /* PWM脉冲宽度值的增减方向 */
    pulse = 0;          /* PWM脉冲宽度值，单位为纳秒ns */

    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);

    while (1)
    {
        rt_thread_mdelay(50);
        if (dir)
        {
            pulse += 5000;      /* 从0值开始每次增加5000ns */
        }
        else
        {
            pulse -= 5000;      /* 从最大值开始每次减少5000ns */
        }
        if (pulse >= period)
        {
            dir = 0;
        }
        if (0 == pulse)
        {
            dir = 1;
        }

        /* 设置PWM周期和脉冲宽度 */
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    }
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(pwm_sg90_sample, pwm sample);
/*
 * 目前用星火一号开发板，用其自带的温度传感器完成温湿度检测，
 * 以及光敏传感器检测光照，当湿度大于一定值的时候，打开一
 * 个小风扇同时打开紫光灯和热源(分别用蓝色和黄色led替代)，
 * 若光照充足大于一定的值时，打开窗户(控制一个电机旋转¼)，
 * 并将晾衣杆伸出去(控制另一个电机旋转¼)，当光照不足时则先后收回
 */
/*
 *
 * 1、lcd点亮显示，数字和字符串
 * 2、aht20 i2c设备点亮
 * 3、ap3216 接近传感器 驱动成功，数据不正常，正在改进
 * 4、蜂鸣器驱动 成功
 * 5、待完成，pwm驱动mg90 舵机
 * 硬件连接
 *MG90S接线：

 *红色线 → 5V电源 (建议使用外部电源)

 *棕色线 → GND

 *橙色信号线 → 星火一号的PWM1通道1引脚 (如PA8)
 *橙色信号线
 * hcsr-501
 * gnd-gnd
 * vcc-5v
 * dout -pd8;
 */
int main(void)
{
    char war[16];

    /*舵机*/
    rt_uint8_t angles[] = {0, 45, 90, 135, 180};
    rt_uint8_t i = 0;
    rt_uint16_t ps_data;//距离数据变量
    //光照、湿度阈值
    int humi_max=80;
    int light_max=80;
    //光照变量
    float brightness;
    float last_brightness = 0;  // 记录上一次的光照值
    // 电机控制状态变量
    int motor_state = -1;  // -1: 未初始化, 0: 光照低于阈值状态, 1: 光照高于阈值状态
    int last_motor_state = -1;  // 记录上一次的电机状态
    int motor_initialized = 0;  // 标记是否已经初始化
    /*aht20*/
    float humidity, temperature;

    /* 设置 RGB 灯引脚为输出模式 */
      rt_pin_mode(PIN_LED_R, PIN_MODE_OUTPUT);
      rt_pin_mode(PIN_LED_B, PIN_MODE_OUTPUT);
      rt_pin_write(PIN_LED_R, 1);
      rt_pin_write(PIN_LED_B, 1);
    /*光照、距离  ap3216 操作句柄*/
    ap3216c_device_t dev;



    /* 设置蜂鸣器引脚为输出模式 */
    rt_pin_mode(PIN_BEEP, PIN_MODE_OUTPUT);

    rt_pin_mode(fan_P, PIN_MODE_OUTPUT);

    /* 电机1 电机2配置引脚 输出模式 */
     Motor_Iint_all();
    /* 设置蜂鸣器引脚为输出模式 */
    rt_pin_mode(HCSR501_PIN, PIN_MODE_INPUT_PULLDOWN);

    /* 进行AHT20设备初始化 */
    AHT_Init(AHT_I2C_BUS_NAME);

    /* 进行ap3216c设备初始化 */
    dev =ap3216c_init("i2c2");

    if (dev == RT_NULL)
        {
            LOG_E("The sensor initializes failure.");
            return 0;
        }
//    /* 进行timer1初始化 */

    //pwm_sg90_sample()
//    timer_init();
    /* 进行mg90s1设备初始化 */
//    if (mg90s_init() != RT_EOK)
//      {
//          rt_kprintf("MG90S1 init failed!\n");
//          return 0;
//      }else{
//
//
//          rt_kprintf("MG90S1 init ok!\n");
//      }
//    /* 进行mg90s2设备初始化 */
//    if (mg90s2_init() != RT_EOK)
//     {
//             rt_kprintf("MG90S2 init failed!\n");
//             return 0;
//       }else{
//
//
//             rt_kprintf("MG90S2 init ok!\n");
//         }

    /* 初始化HC-SR501 */
   hcsr501_init();




    lcd_clear(WHITE);
//
//    /* show RT-Thread logo */
//    lcd_show_image(0, 0, 240, 69, image_rttlogo);

    /* set the background color and foreground color */
    lcd_set_color(WHITE, RED);

    /* show some string on lcd */
    lcd_show_string(2, 0, 24, "RT-Thread! 2025-7-7");
//    lcd_show_string(10, 69 + 16, 24, "RT-Thread");
//    lcd_show_string(10, 69 + 16 + 24, 32, "RT-Thread");

//    /* draw a line on lcd */
//    lcd_draw_line(0, 69 + 16 + 24 + 32, 240, 69 + 16 + 24 + 32);
    /* 蜂鸣器打开 */


    while(1)
    {
        //读取温湿度数据
        read_temp_humi(&temperature, &humidity);    //读取数据
                 rt_kprintf("humidity   : %d.%d %%\n", (int)humidity, (int)(humidity * 10) % 10);
                 rt_kprintf("temperature: %d.%d\n", (int)temperature, (int)(temperature * 10) % 10);
        rt_thread_mdelay(1000);

        sprintf(war ,"Temp:%dC Humi:%d%%",(int)temperature,(int)humidity);
        lcd_show_string(2, 39, 24, war);

     /* 读接近感应值 */
         ps_data = ap3216c_read_ps_data(dev);
         if (ps_data == 0)
         {
             LOG_D("object is not proximity of sensor.");
         }
         else
         {
             LOG_D("current ps data   : %d.", ps_data);
         }

         /* 读光照强度值 */
         brightness = ap3216c_read_ambient_light(dev);
         LOG_D("current brightness: %d.%d(lux).", (int)brightness, ((int)(10 * brightness) % 10));

         sprintf(war ,"Ps:%04d Lux:%03dlux",(int)ps_data,(int)brightness);
         lcd_show_string(2, 69, 24, war);

         /*舵机驱动*/

        // mg90s_set_angle(angles[i]);
//         rt_kprintf("Set angle: %d\n", angles[i]);

//         i = (i + 1) % (sizeof(angles)/sizeof(angles[0]));
//         sprintf(war ,"mg90s1 angle:%d",angles[i]);
//           lcd_show_string(2, 99, 24, war);
//              rt_thread_mdelay(1000);
         //人体红外感应检测
         int ir_value = hcsr501_read();
         if(ir_value == PIN_HIGH)
         {
             sprintf(war ,"hcsr501:close");
             rt_kprintf("hcsr501:close (PIN_HIGH)\n");
              lcd_show_string(2, 129, 24, war);
         }else{

             sprintf(war ,"hcsr501:have people");
             rt_kprintf("hcsr501:have people (PIN_LOW)\n");
              lcd_show_string(2, 129, 24, war);
              /* 蜂鸣器打开 */
              rt_pin_write(PIN_BEEP,PIN_LOW);
              rt_thread_mdelay(500);
              /* 蜂鸣器关闭 */
              rt_pin_write(PIN_BEEP,PIN_HIGH);
         }
         // 显示光照变化状态和电机状态
         if (brightness > last_brightness) {
             sprintf(war ,"Light:UP  Lmax:%03d",light_max);
         } else if (brightness < last_brightness) {
             sprintf(war ,"Light:DOWN Lmax:%03d",light_max);
         } else {
             sprintf(war ,"Light:STABLE Lmax:%03d",light_max);
         }
         lcd_show_string(2, 159, 24, war);
         
         // 显示电机状态
         sprintf(war ,"Motor:%s Init:%s", 
                 motor_state >= 0 ? (motor_state ? "HIGH" : "LOW") : "NONE",
                 motor_initialized ? "OK" : "NO");
         lcd_show_string(2, 189, 24, war);
         rt_thread_mdelay(100);

         /*电机控制逻辑1 当湿度大于一定值的时候，打开一
          * 个小风扇同时打开紫光灯和热源(分别用红色led替代)，
          */

         if((int)humidity >humi_max)
         {
             rt_kprintf("湿度过大\n");

            //红灯亮
            rt_pin_write(PIN_LED_R, 0);

            //风扇打开
            rt_pin_write(fan_P,PIN_HIGH);

         }
         else {
             //红灯灭
             rt_pin_write(PIN_LED_R, 1);
             //风扇关闭
             rt_pin_write(fan_P,PIN_LOW);
         }
         
         /*蓝光LED控制逻辑：只受红外线传感器控制
          * 检测到红外线时：蓝灯不变（保持当前状态）
          * 检测不到红外线时：蓝灯闪烁*/
         static int blue_led_state = 1;  // 蓝灯状态，1为关闭，0为开启
         static int flash_counter = 0;   // 闪烁计数器
         
         if(hcsr501_read() == PIN_HIGH) {
             // 检测到红外线，蓝灯保持当前状态不变
             rt_kprintf("检测到红外线，蓝灯状态保持\n");
         } else {
             // 检测不到红外线，蓝灯闪烁
             flash_counter++;
             if(flash_counter >= 5) {  // 每5个循环切换一次状态，实现闪烁效果
                 blue_led_state = !blue_led_state;  // 切换状态
                 flash_counter = 0;
                 rt_kprintf("未检测到红外线，蓝灯闪烁状态: %s\n", blue_led_state ? "关闭" : "开启");
             }
             rt_pin_write(PIN_LED_B, blue_led_state);
         }
         /*电机控制逻辑：根据光照变化控制电机转向
         * 光照从低到高：电机正转（打开窗户/伸出晾衣杆）
         * 光照从高到低：电机反转（关闭窗户/收回晾衣杆）*/
         
         // 检测光照状态变化
         int current_light_state = ((int)brightness > light_max) ? 1 : 0;
         
         // 初始化：第一次运行时，只记录状态，不执行动作
         if (!motor_initialized) {
             last_motor_state = current_light_state;
             motor_state = current_light_state;
             motor_initialized = 1;
             rt_kprintf("电机控制初始化完成，当前状态:%s\n", current_light_state ? "HIGH" : "LOW");
         } else {
             // 调试信息
             rt_kprintf("当前光照:%d, 阈值:%d, 当前状态:%s, 上次状态:%s, 跨过阈值:%s\n",
                       (int)brightness, light_max, 
                       current_light_state ? "HIGH" : "LOW",
                       last_motor_state ? "HIGH" : "LOW",
                       (current_light_state != last_motor_state) ? "YES" : "NO");
             
             // 检测是否跨过阈值：当前状态与上次状态不同
             if (current_light_state != last_motor_state) {
                 // 跨过阈值，执行相应动作
                 if (current_light_state == 1 && last_motor_state == 0) {
                     // 光照从低于阈值跨过阈值变为高于阈值，电机正转
                     rt_kprintf("光照跨过阈值(低->高)，电机正转 - 电机1先转\n");
                     
                     // 控制电机1正转1秒后停止（先转）
                     Motor_Run_SYS1(1);
                     rt_thread_mdelay(1000);
                     Motor_Run_SYS1(0);
                     
                     // 控制电机2正转1秒后停止（后转）
                     Motor_Run_SYS2(1);
                     rt_thread_mdelay(1000);
                     Motor_Run_SYS2(0);
                     
                 } else if (current_light_state == 0 && last_motor_state == 1) {
                     // 光照从高于阈值跨过阈值变为低于阈值，电机反转
                     rt_kprintf("光照跨过阈值(高->低)，电机反转 - 电机2先转\n");
                     
                     // 控制电机2反转1秒后停止（先转）
                     Motor_Run_SYS2(2);
                     rt_thread_mdelay(1000);
                     Motor_Run_SYS2(0);
                     
                     // 控制电机1反转1秒后停止（后转）
                     Motor_Run_SYS1(2);
                     rt_thread_mdelay(1000);
                     Motor_Run_SYS1(0);
                 }
                 // 注意：如果状态相同，不执行任何动作
                 
                 // 更新状态
                 last_motor_state = current_light_state;
                 motor_state = current_light_state;
             }
         }


        // 更新上一次的光照值（无论是否超过阈值都要更新）
        last_brightness = brightness;

    }
    return 0;
}

