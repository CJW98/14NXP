/********************************************
逐飞科技 总钻风-摄像头  历程
Designed by Fly Sir
软件版本:V1.1
最后更新:2016年5月3日
相关信息参考下列地址：
淘宝店：https://seekfree.taobao.com/
------------------------------------
软件版本： IAR 7.2 or MDK 5.17
目标核心： MK60DN512VLL10
============================================
MT9V032接线定义：
------------------------------------ 
模块管脚            单片机管脚
SDA(51的RX)         PTC17
SCL(51的TX)         PTC16
场中断(VSY)         PTC6
像素中断(PCLK)      PTC18
数据口(D0-D7)       PTC8-PTC15 

串口  
波特率 115200 
数据位 8 
校验位 无
停止位 1位
流控   无
串口连接注意事项：切勿使用蓝牙等无线串口连接
RX                  PTD3
TX                  PTD2
============================================

分辨率是                188*120
摄像头参数设置可以到    SEEKFREE-->h_file-->SEEKFREE_MT9V032.h

总钻风-摄像头测试步骤：
1.下载程序到开发板
2.插上串口线或者USB转TTL
3.接好MT9V032模块接线
4.通电在TFT液晶上即可观看    
*********************************************/  
#include "headfile.h"
void System_Init();


uint16 dat;

float sys_time;


int main(void)
{  
	
	float var[5]; 
	char a;
	get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
	Car_Debug();
	System_Init();
	//Mode_Set();
	float x=1;
	while(1)
	{
		
		
		if(mt9v032_finish_flag)
		{
			//	mt9v032_finish_flag = 0;
			//	pit_time_start(pit2); 
			Image_Handle();
			//    Otsu();         //大津法
			//	sys_time=pit_time_get(pit2)/(bus_clk_mhz*1000);
			
			Car_Run();
		}
		
		//seekfree_sendimg_032();
		/***ADC数据发送*****/
#if 0//上位机波形
		var[0]=ADC_End[1];
		var[1]=ADC_End[2];
		var[2]=ADC_End[3];
		var[3]=ADC_End[4];
		var[4]=ADC_End[5];
		
		
		/***陀螺仪数据发送****/
		//	var[0]=Measure_Speed_R;                  //加速度角度
		//	var[1]= Speed_Set;   //互补角速度角度
		//	var[2]=Measure_Speed_R;
		//	var[3]=-Angle[0];
		
		vcan_sendware( var ,  sizeof(var));      //山外虚拟示波器    
#endif
		
		
		
#if 1//通信	
		/****通信函数****/
		//  Twocar_Train_RX();
		//       if(stop_Flag)
		if(Measure_Speed_L>=20 && A_Start_Ok==0)
			A_Start_Ok = 1;
		Twocar_Train_TX();
		Twocar_Train_RX();
#endif
		
	}
}



void System_Init()
{	
	
	NVIC_SetPriorityGrouping(4);        //设置中断抢占优先级
	NVIC_SetPriority(PORTC_IRQn,1);     //场中断优先级
	NVIC_SetPriority(DMA0_IRQn,0);      //DMA中断优先级
	NVIC_SetPriority(UART3_RX_TX_IRQn,2);
	NVIC_SetPriority(PIT0_IRQn,3);      //定时器中断优先级
	NVIC_SetPriority(PIT1_IRQn,5);      //定时器中断优先级
	
	//	   NVIC_SetPriority(UART1_RX_TX_IRQn,0);  //通信串口
	//	  NVIC_SetPriority(UART0_RX_TX_IRQn,0); //壁障串口
	PID_Parameter_Init(&Direct_Out_PID);		// 摄像头PID初始化
	PID_Parameter_Init(&Direct_In_PID);		// 摄像头PID初始化
	Motor_PID_Init();                     //电机PID初始化   Direct_Out
	Disable=50;   //起跑线防止误判5s
	/***陀螺仪初始化****/
	I2C_Init();
	
	
	/***ADC初始化****/
	// adc_init(AD6);
	adc_init(AD5);
	adc_init(AD4);
	adc_init(AD3);
	adc_init(AD2);
	adc_init(AD1);  
	
	/***Flash初始化*****/
	FLASH_Init();
	/****虚拟示波器初始化***/
	DATA_init();
	/******电机初始化******/
	ftm_pwm_init(ftm0,ftm_ch1,17*1000,0);
	ftm_pwm_init(ftm0,ftm_ch5,17*1000,0);//占空比控制引脚 PTA10 控制频率 17K     注意：CH1的口子还没有查
	gpio_init (C1, GPO,1);
	gpio_init (C3, GPO,1);
	
	gpio_init (C5, GPO,0);
	/***编码器初始化***/   
	ftm_quad_init(ftm1); 
	ftm_quad_init(ftm2);
	
	/*******蜂鸣器初始化****/
	gpio_init (D10, GPO,0);
	
	/****按键初始化*****/
	key_init (KEY_MAX);

	pit_init_ms(pit1,100);                                     //初始化pit中断6ms
	enable_irq (PIT1_IRQn); 
	
	Speed_jiajia=1;
	lcd_init();   //TFT初始化
	camera_init(); //摄像头初始化
	
	pit_init_ms(pit0,3);                                     //初始化pit中断6ms
	//set_irq_priority(PIT0_IRQn,0);						//设置优先级,根据自己的需求设置优先级范围0-15 
	enable_irq (PIT0_IRQn); 
	
	EnableInterrupts;									//打开总的中断开关
}





































//uint8  data1[8] = {1,2,3,4,5,6,7,8};
//uint16 data2 = 60000;
//uint32 data3 = 600051;
//
//uint8  data11[8];
//uint16 data22; 
//uint32 data33;
//
//int main(void)
//{
//	get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
//	
//	//相关的库函数在 MK60DN10_flash.c 里面
//	FLASH_Init();			//初始化flash模块
//	FLASH_EraseSector(10);	//擦除扇区
//	FLASH_WriteSector(10,(const uint8 *)data1,8,0);		
//	FLASH_WriteSector(10,(const uint8 *)&data2,8,8);
//	FLASH_WriteSector(10,(const uint8 *)&data3,8,16);
//    for(;;)
//	{
//		//读取保存的data1数组数据
//		data11[0] = flash_read(10,0,uint8);	
//		data11[1] = flash_read(10,1,uint8);
//		data11[2] = flash_read(10,2,uint8);
//		data11[3] = flash_read(10,3,uint8);
//        data11[4] = flash_read(10,4,uint8);
//        data11[5] = flash_read(10,5,uint8);
//        data11[6] = flash_read(10,6,uint8);
//        data11[7] = flash_read(10,7,uint8);
//		
//		//读取保存的data2数组数据
//		data22 = flash_read(10,8,uint16);
//		
//		//读取保存的data3变量数据
//		data33 = flash_read(10,16,uint32);
//	}
//}


