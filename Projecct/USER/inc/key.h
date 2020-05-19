#ifndef _KEY_H
#define _KEY_H

#include "headfile.h"
#include "common.h"

//下面是定义按键的时间，单位为 ： 10ms（中断时间）
#define KEY_DOWN_TIME           1       //消抖确认按下时间
#define KEY_HOLD_TIME           50      //长按hold确认时间，最多253，否则需要修改 keytime 的类型
                                        //如果按键一直按下去，则每隔 KEY_HOLD_TIME - KEY_DOWN_TIME 时间会发送一个 KEY_HOLD 消息

//定义按键消息FIFO大小
#define KEY_MSG_FIFO_SIZE       20      //最多 255，否则需要修改key_msg_front/key_msg_rear类型

//按键端口的枚举
typedef enum
{
    
    
    KEY_R,  //右
    KEY_L,  //左
    
    KEY_U,  //上
    KEY_D,  //下



    KEY_A,  //取消
    KEY_B,  //选择


    KEY_MAX,
} KEY_e;


//key状态枚举定义
typedef enum
{
    KEY_DOWN  =   0,         //按键按下时对应电平
    KEY_UP    =   1,         //按键弹起时对应电平

    KEY_HOLD,               //长按按键(用于定时按键扫描)

} KEY_STATUS_e;


//按键消息结构体
typedef struct
{
    KEY_e           key;        //按键编号
    KEY_STATUS_e    status;     //按键状态
} KEY_MSG_t;


extern void            key_init(KEY_e key);            // KEY初始化函数(key 小于 KEY_MAX 时初始化 对应端口，否则初始化全部端口)
extern KEY_STATUS_e    key_get(KEY_e key);             //检测key状态（不带延时消抖）
extern KEY_STATUS_e    key_check(KEY_e key);           //检测key状态（带延时消抖）

//定时扫描按键
extern uint8   get_key_msg(KEY_MSG_t *keymsg);         //获取按键消息，返回1表示有按键消息，0为无按键消息
extern void    key_IRQHandler(void);                   //需要定时扫描的中断服务函数（定时时间为10ms）


#endif  //__VCAN_KEY_H__