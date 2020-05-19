#include "flash.h"
#include "font.h"
float PID_Motor_L[4],PID_Motor_R[4];

float DIRECT_IN[4]={1, 0 ,10, 0};      //摄像头    电磁1   0  25  
//float DIRECT_OUT[4]={0.02, 0, 0.01, 70};       // 转向外环
//float DIRECT_IN[4]={1, 0 ,20, 0};      //转向内环    电磁1   0  25  
float DIRECT_OUT[4]={1, 0, 5, 70};       // 转向外环
float PID_MOTOR_L[4]={6, 2, 0, 0};    //电机环
float PID_MOTOR_R[4]={6,2,0,0};
uint8_t RING[4]={1,1};
uint8_t RING_SPEED[4]={1,2};
uint16_t SPEED_SET=45;
uint8_t  OB_WAY[4]={0,50,50,40};         //避障个数   正向第二个角   方向  第一个角的距离
uint8_t Chang_Speed[4]={45,45,3,1};   //max  min   up

//摄像头
uint8_t CAMERA_DATA[4]={32,25,25,110};   //摄像头增益  边线阈值  壁障阈值  环岛最后的积分
uint8_t LINE_DATA[4]={20,4,200,200};    //环岛出环积分  进环积分  进环积分   坡道积分 
uint8_t  saidao_data[4]={10,10,50,60};//第一个壁障线  第二个壁障线  起跑线间隔  断道路径
uint8_t CESHI_DATA[4]={0,25,35,0};  //测距监测   起跑线阈值  坡道电感阈值
/******因为Flash不能存储浮点型的数组，故转化为整数后存到Flash里面*****/
uint8_t Int_MOTOR_L[4];              //直立电机pid
uint8_t Int_MOTOR_R[4];              //直立电机pid
uint8_t Int_DIRECT_IN[4];             //直立方向环
uint8_t Int_DIRECT_OUT[4];

/******因为Flash不能存储浮点型的数组，故转化为整数后存到Flash里面*****/
uint8_t Int_Motor_L[4];                  //直立电机pid
uint8_t Int_Motor_R[4];                  //直立电机pid
uint8_t Int_Direct_In[4];                //直立方向环
uint8_t Int_Direct_Out[4];
uint16_t  Int_Speed_set;
uint8_t   Int_Ring[2];
uint8_t   Int_Ring_Speed[2];
uint8_t   Int_ob_way[4];
uint8_t   Int_Chang_Speed[4];
uint8_t   Int_CAMERA_DATA[4];
uint8_t   Int_LINE_DATA[4];    //环岛出环积分  进环积分  进环积分   坡道积分  
 uint8_t  Int_saidao_data[4];//第一个壁障线  第二个壁障线  起跑线间隔  断道路径
 uint8_t Int_CESHI_DATA[4];  //测距监测   起跑线阈值  坡道电感阈值
#define Clear 0

void Car_Debug()
{
	key_init (KEY_MAX);
	FLASH_Init();
	lcd_init();   //TFT初始化
	dsp_single_colour(BLACK);
	int m;
	int set = 1;
	Int_DIRECT_IN[0] = (uint8)(DIRECT_IN[0]*1);
	Int_DIRECT_IN[1] = (uint8)(DIRECT_IN[1]*1);
	Int_DIRECT_IN[2] = (uint8)(DIRECT_IN[2]*1);
	Int_DIRECT_IN[3] = (uint8)(DIRECT_IN[3]*1);
	
	Int_DIRECT_OUT[0] = (uint8)(DIRECT_OUT[0]*10);
	Int_DIRECT_OUT[1] = (uint8)(DIRECT_OUT[1]*1);
	Int_DIRECT_OUT[2] = (uint8)(DIRECT_OUT[2]*10);
	Int_DIRECT_OUT[3] = (uint8)(DIRECT_OUT[3]*1);
	
	Int_MOTOR_L[0] = (uint8)(PID_MOTOR_L[0]*10);
	Int_MOTOR_L[1] = (uint8)(PID_MOTOR_L[1]*10);
	Int_MOTOR_L[2] = (uint8)(PID_MOTOR_L[2]*10);
	Int_MOTOR_L[3] = (uint8)(PID_MOTOR_L[3]*10);
	
	Int_MOTOR_R[0] = (uint8)(PID_MOTOR_R[0]*10);
	Int_MOTOR_R[1] = (uint8)(PID_MOTOR_R[1]*10);
	Int_MOTOR_R[2] = (uint8)(PID_MOTOR_R[2]*10);
	Int_MOTOR_R[3] = (uint8)(PID_MOTOR_R[3]*10);
	
	
	Int_Chang_Speed[0] = (uint8)(Chang_Speed[0]*1);
	Int_Chang_Speed[1] = (uint8)(Chang_Speed[1]*1);
	Int_Chang_Speed[2] = (uint8)(Chang_Speed[2]*1);
	Int_Chang_Speed[3] = (uint8)(Chang_Speed[3]*1);
	
	
	if(Clear)
	{
		FLASH_EraseSector(SectorNum);
		FLASH_WriteSector(SectorNum, (const uint8*)Int_DIRECT_IN, 8, 8);    //转向内环
		FLASH_WriteSector(SectorNum, (const uint8*)Int_DIRECT_OUT, 8, 16);  // 转向外环
		FLASH_WriteSector(SectorNum, (const uint8*)Int_MOTOR_L, 8, 24);       //电机环 
		FLASH_WriteSector(SectorNum, (const uint8*)Int_MOTOR_R, 8, 32);       //电机环 
		FLASH_WriteSector(SectorNum, (const uint8*)&SPEED_SET, 8, 40);       //电机环 
		FLASH_WriteSector(SectorNum, (const uint8*)RING, 8, 48);       //电机环 
		FLASH_WriteSector(SectorNum, (const uint8*)RING_SPEED, 8, 56);       //电机环 
		FLASH_WriteSector(SectorNum, (const uint8*)OB_WAY, 8, 64);       //电机环
		FLASH_WriteSector(SectorNum, (const uint8*)Int_Chang_Speed, 8, 72);       //电机环
		FLASH_WriteSector(SectorNum, (const uint8*)CAMERA_DATA, 8, 80);       //电机环
		FLASH_WriteSector(SectorNum, (const uint8*)LINE_DATA, 8, 88);       //电机环
		FLASH_WriteSector(SectorNum, (const uint8*)saidao_data, 8, 96);       //电机环
		FLASH_WriteSector(SectorNum, (const uint8*)CESHI_DATA, 8, 104);       //电机环
	}
	
	
	
	Int_Direct_In[0] = flash_read(SectorNum,8,uint8);	
	Int_Direct_In[1] = flash_read(SectorNum,9,uint8);
	Int_Direct_In[2] = flash_read(SectorNum,10,uint8);
	Int_Direct_In[3] = flash_read(SectorNum,11,uint8);
	
	Int_Direct_Out[0] = flash_read(SectorNum,16,uint8);	
	Int_Direct_Out[1] = flash_read(SectorNum,17,uint8);
	Int_Direct_Out[2] = flash_read(SectorNum,18,uint8);
	Int_Direct_Out[3] = flash_read(SectorNum,19,uint8);
	
	Int_Motor_L[0] = flash_read(SectorNum,24,uint8);	
	Int_Motor_L[1] = flash_read(SectorNum,25,uint8);
	Int_Motor_L[2] = flash_read(SectorNum,26,uint8);
	Int_Motor_L[3] = flash_read(SectorNum,27,uint8);
	
	Int_Motor_R[0] = flash_read(SectorNum,32,uint8);	
	Int_Motor_R[1] = flash_read(SectorNum,33,uint8);
	Int_Motor_R[2] = flash_read(SectorNum,34,uint8);
	Int_Motor_R[3] = flash_read(SectorNum,35,uint8);
	Int_Speed_set = flash_read(SectorNum,40,uint16);
	Int_Ring[0]= flash_read(SectorNum,48,uint8);
	Int_Ring[1]= flash_read(SectorNum,49,uint8);
	Int_Ring_Speed[0]= flash_read(SectorNum,56,uint8);
	Int_Ring_Speed[1]= flash_read(SectorNum,57,uint8);
	
	
	
	Int_ob_way[0] = flash_read(SectorNum,64,uint8);	
	Int_ob_way[1] = flash_read(SectorNum,65,uint8);
	Int_ob_way[2] = flash_read(SectorNum,66,uint8);
	Int_ob_way[3] = flash_read(SectorNum,67,uint8);
	
	
	Int_Chang_Speed[0] = flash_read(SectorNum,72,uint8);	
	Int_Chang_Speed[1] = flash_read(SectorNum,73,uint8);	
	Int_Chang_Speed[2] = flash_read(SectorNum,74,uint8);	
	//	Int_Chang_Speed[3] = (uint8)(Chang_Speed[3]*1);
	
	Int_CAMERA_DATA[0] = flash_read(SectorNum,80,uint8);	
	Int_CAMERA_DATA[1] = flash_read(SectorNum,81,uint8);
	Int_CAMERA_DATA[2] = flash_read(SectorNum,82,uint8);
	Int_CAMERA_DATA[3] = flash_read(SectorNum,83,uint8);
	
	Int_LINE_DATA[0] = flash_read(SectorNum,88,uint8);	
	Int_LINE_DATA[1] = flash_read(SectorNum,89,uint8);
	Int_LINE_DATA[2] = flash_read(SectorNum,90,uint8);
	Int_LINE_DATA[3] = flash_read(SectorNum,91,uint8);
	
	Int_saidao_data[0] = flash_read(SectorNum,96,uint8);	
	Int_saidao_data[1] = flash_read(SectorNum,97,uint8);
	Int_saidao_data[2] = flash_read(SectorNum,98,uint8);
	Int_saidao_data[3] = flash_read(SectorNum,99,uint8);
	
	Int_CESHI_DATA[0] = flash_read(SectorNum,104,uint8);	
	Int_CESHI_DATA[1] = flash_read(SectorNum,105,uint8);
	Int_CESHI_DATA[2] = flash_read(SectorNum,106,uint8);
	//Int_saidao_data[3] = flash_read(SectorNum,99,uint8);
	
	while(set)
	{    
		if(m<10)
		{
			/*********电机环PID参数***********/
			
			
			
			//
			
			lcd_showstr(10,0,"ring_1:");
			lcd_showint32(70,0, Ring[0],4);
			
			
			
			lcd_showstr(10,1,"ring_2:");
			lcd_showint32(70,1, Ring[1],4);
			
			lcd_showstr(10,2,"R1_s:");
			lcd_showint32(70,2, Ring_Speed[0],4);
			
			lcd_showstr(10,3,"R2_s:");
			lcd_showint32(70,3, Ring_Speed[1],4);
			
			
			lcd_showstr(10,4,"w_num:");
			lcd_showint32(70,4, Way_Flag,4);
			
			

			
			lcd_showstr(10,5,"ob_1:");
			lcd_showint32(70,5, Obstacle_Angle[0],4);
			
			lcd_showstr(10,6,"ob_2:");
			lcd_showint32(70,6, Obstacle_Angle[1],4);
			
			lcd_showstr(10,7,"ob_line:");
			lcd_showint32(70,7, Obstacle_Line,4);
			
			lcd_showstr(10,8,"S_max:");
			lcd_showint32(70,8, Speed_Nor_High,4);
			
			lcd_showstr(10,9,"S_min:");
			lcd_showint32(70,9, Speed_Nor_Low,4);
		}
		else if(m<20)
		{
			
			
			lcd_showstr(10,0,"S_UP:");
			lcd_showint32(70,0, Speed_Up,4);
			
			
			lcd_showstr(10,1,"L_S_KP:");
			lcd_showfloat(70,1,PID_Motor_L[0],2,1);
			
			lcd_showstr(10,2,"L_S_Ki:");
			lcd_showfloat(70,2,PID_Motor_L[1],2,1);
			
			
			lcd_showstr(10,3,"R_S_KP:");
			lcd_showfloat(70,3,PID_Motor_R[0],2,1);
			
			lcd_showstr(10,4,"R_S_Ki:");
			lcd_showfloat(70,4,PID_Motor_R[1],2,1);
			/*********转向内环PID参数***********/
			lcd_showstr(10,5,"Dir_IKp:");
			lcd_showfloat(70,5,Direct_In[0],2,1);
			
			lcd_showstr(10,6,"Dir_IKd:");
			lcd_showfloat(70,6,Direct_In[2],2,1);
			
			
			/*********转向外环PID参数***********/
			lcd_showstr(10,7,"Ang_SKp:");
			lcd_showfloat(70,7,Direct_Out[0],2,3);
			
			lcd_showstr(10,8,"Ang_SKD:");
			lcd_showfloat(70,8,Direct_Out[2],2,3);
			
			
			lcd_showstr(10,9,"SPEED:");
			lcd_showint32(70,9,Speed_Set,4);
			
			
			
		}
		
		
				else if(m<30)
		{
			
			
			lcd_showstr(10,0,"zengyi;");
			lcd_showint32(70,0, Camera_data,4);
			
			
			lcd_showstr(10,1,"yuzhi:");
			lcd_showint32(70,1,Side_Thres,4);
			
			lcd_showstr(10,2,"ob_yuzhi:");
			lcd_showint32(70,2,ob_Side_Thres,4);
			
			
			lcd_showstr(10,3,"R_out_C:");
			lcd_showint32(70,3,yuanhuan_line,4);
			
			
			
			
			lcd_showstr(10,4,"R_out:");
			lcd_showint32(70,4,ring_distance[0],4);
			/*********转向内环PID参数***********/
			lcd_showstr(10,5,"R_int:");
			lcd_showint32(70,5,ring_distance[1],4);
			
			lcd_showstr(10,6,"R_int_C:");
			lcd_showint32(70,6,ring_distance[2],4);
			
			
			/*********转向外环PID参数***********/
			lcd_showstr(10,7,"podao:");
			lcd_showint32(70,7,ring_distance[3],4);
			
			
			
			
			lcd_showstr(10,8,"OB_L1:");
			lcd_showint32(70,8,SAIDAO_DATA[0],4);
			

			lcd_showstr(10,9,"OB_L2:");
			lcd_showint32(70,9,SAIDAO_DATA[1],4);
						
			
		}
		
						else if(m<40)
		{
			
			
			lcd_showstr(10,0,"START;");
			lcd_showint32(70,0, SAIDAO_DATA[2],4);
			
			
			lcd_showstr(10,1,"duandao:");
			lcd_showint32(70,1,SAIDAO_DATA[3],4);
			
			
			lcd_showstr(10,2,"CeLiang:");
			lcd_showint32(70,2,ceshi_data[0],4);
			/*********转向内环PID参数***********/
			lcd_showstr(10,3,"S_YUZHI:");
			lcd_showint32(70,3,ceshi_data[1],4);
			
			lcd_showstr(10,4,"ad5:");
			lcd_showint32(70,4,ceshi_data[2],4);
			
			
//			/*********转向外环PID参数***********/
//			lcd_showstr(10,7,"podao:");
//			lcd_showint32(70,7,ring_distance[3],4);
			
//			lcd_showstr(10,2,"ob_yuzhi:");
//			lcd_showint32(70,2,ob_Side_Thres,4);
//			
//			
//			lcd_showstr(10,3,"R_out_C:");
//			lcd_showint32(70,3,yuanhuan_line,4);
//			
//			
//			
//			
//			lcd_showstr(10,4,"R_out:");
//			lcd_showint32(70,4,ring_distance[0],4);
//			/*********转向内环PID参数***********/
//			lcd_showstr(10,5,"R_int:");
//			lcd_showint32(70,5,ring_distance[1],4);
//			
//			lcd_showstr(10,6,"R_int_C:");
//			lcd_showint32(70,6,ring_distance[2],4);
//			
//			
//			/*********转向外环PID参数***********/
//			lcd_showstr(10,7,"podao:");
//			lcd_showint32(70,7,ring_distance[3],4);
//			
//			
//			
//			
//			lcd_showstr(10,8,"OB_L1:");
//			lcd_showfloat(70,8,SAIDAO_DATA[0],2,3);
//			
//
//			lcd_showstr(10,9,"OB_L2:");
//			lcd_showint32(70,9,SAIDAO_DATA[1],4);
						
			
		}
		
		
		if(key_check(KEY_U) ==  KEY_DOWN)
		{     
			//dsp_single_colour(BLACK);
			if(m<10)
				lcd_showstr(0,m," ");
			else if(m<20)
				lcd_showstr(0,m-10," ");
			
			else if(m<30)
				lcd_showstr(0,m-20," ");
			else if(m<40)
				lcd_showstr(0,m-30," ");
			m++;
			if(m==10)
				dsp_single_colour(BLACK);
			if(m==20)
				dsp_single_colour(BLACK);

			if(m==30)
				dsp_single_colour(BLACK);
			
			if(m>39)
			{
				m=0;
				dsp_single_colour(BLACK);
			}
			//	while(key_check(KEY_D) ==  KEY_DOWN);
		}
		
		
		if(key_check(KEY_D) ==  KEY_DOWN)
		{         
			//dsp_single_colour(BLACK);
			if(m<10)
				lcd_showstr(0,m," ");
			else if(m<20)
				lcd_showstr(0,m-10," ");	
			else if(m<30)
				lcd_showstr(0,m-20," ");
			else if(m<40)
				lcd_showstr(0,m-30," ");
			m--;
			if(m==9)
				dsp_single_colour(BLACK);
			if(m==19)
				dsp_single_colour(BLACK);
			if(m==29)
				dsp_single_colour(BLACK);
			if(m<0)
			{
				m=39;
				dsp_single_colour(BLACK);
			}
			
			//while(key_check(KEY_D) ==  KEY_DOWN);
		}
		
		if(key_check(KEY_R) ==  KEY_DOWN)
		{
			switch(m)
			{
				
				
				case 0:Int_Ring[0]+=1;
				if(Int_Ring[0]>2)
					Int_Ring[0]=1;
				break;
				case 1:Int_Ring[1]+=1;
				if(Int_Ring[1]>2)
					Int_Ring[1]=1;
				break;
				
				case 2:Int_Ring_Speed[0]+=1;
				if(Int_Ring_Speed[0]>2)
					Int_Ring_Speed[0]=1;
				break;
				
				case 3:Int_Ring_Speed[1]+=1;
				if(Int_Ring_Speed[1]>2)
					Int_Ring_Speed[1]=1;
				break;
				
				
				case 4:Int_ob_way[0]+=1;
				break;
				
				
				
				case 5:Int_ob_way[1]+=1;
				
				break;
				
				case 6:Int_ob_way[2]+=1;				
				break;
				
				
				case 7:Int_ob_way[3]+=1;				
				break;
				
				
				case 8:Int_Chang_Speed[0]+=1;
				
				break;
				
				case 9:Int_Chang_Speed[1]+=1;
				
				break;
				
				
				case 10:Int_Chang_Speed[2]+=1;
				
				break;
				
				case 11:Int_Motor_L[0]+=1;  break;
				case 12:Int_Motor_L[1]+=1;  break;  
				
				case 13:Int_Motor_R[0]+=1;  break;
				case 14:Int_Motor_R[1]+=1;  break;   
				
				case 15:Int_Direct_In[0]+=1;  break;
				case 16:Int_Direct_In[2]+=1;  break;
				
				case 17:Int_Direct_Out[0]+=1;  break;
				case 18:Int_Direct_Out[2]+=1;  break;
				
				case 19:Int_Speed_set+=1; break;
				
				
				
				case 20:Int_CAMERA_DATA[0]+=1;  break;
				
				case 21:Int_CAMERA_DATA[1]+=1;  break;
				case 22:Int_CAMERA_DATA[2]+=1;  break;
				
				case 23:Int_CAMERA_DATA[3]+=1; break;
				
				
				case 24:Int_LINE_DATA[0]+=1;  break;
				
				case 25:Int_LINE_DATA[1]+=1;  break;
				case 26:Int_LINE_DATA[2]+=1;  break;
				
				case 27:Int_LINE_DATA[3]+=1; break;
				
				
				
				case 28:Int_saidao_data[0]+=1;  break;
				
				case 29:Int_saidao_data[1]+=1;  break;
				case 30:Int_saidao_data[2]+=1;  break;
				
				case 31:Int_saidao_data[3]+=1; break;
				
				
				case 32:Int_CESHI_DATA[0]+=1;  break;
				
				case 33:Int_CESHI_DATA[1]+=1;  break;
				case 34:Int_CESHI_DATA[2]+=1;  break;
				
				//case 31:Int_saidao_data[3]+=1; break;
				
			}
			systick_delay_ms(150);
		}
		
		if(key_check(KEY_L) ==  KEY_DOWN)
		{
			switch(m)
			{
				
				
				
				
				
				case 0:Int_Ring[0]-=1;
				if(Int_Ring[0]<1)
					Int_Ring[0]=2;
				break;
				case 1:Int_Ring[1]-=1;
				if(Int_Ring[1]<1)
					Int_Ring[1]=2;				
				break;
				case 2:Int_Ring_Speed[0]-=1;
				if(Int_Ring_Speed[0]<1)
					Int_Ring_Speed[0]=2;
				break;
				
				
				case 3:Int_Ring_Speed[1]-=1;
				if(Int_Ring_Speed[1]<1)
					Int_Ring_Speed[1]=2;
				break;
				
				case 4:Int_ob_way[0]-=1;				
				break;
				
				case 5:Int_ob_way[1]-=1;				
				break;
				
				case 6:Int_ob_way[2]-=1;				
				break;
				
				
				case 7:Int_ob_way[3]-=1;				
				break;
				
				case 8:Int_Chang_Speed[0]-=1;
				
				break;
				
				case 9:Int_Chang_Speed[1]-=1;
				
				break;
				
				
				case 10:Int_Chang_Speed[2]-=1;
				
				break;
				case 11:Int_Motor_L[0]-=1;  break;
				case 12:Int_Motor_L[1]-=1;  break;  
				
				case 13:Int_Motor_R[0]-=1;  break;
				case 14:Int_Motor_R[1]-=1;  break;   
				
				case 15:Int_Direct_In[0]-=1;  break;
				case 16:Int_Direct_In[2]-=1;  break;
				
				case 17:Int_Direct_Out[0]-=1;  break;
				case 18:Int_Direct_Out[2]-=1;  break; 
				case 19:Int_Speed_set-=1; break;
				
				case 20:Int_CAMERA_DATA[0]-=1;  break;
				
				case 21:Int_CAMERA_DATA[1]-=1;  break;
				case 22:Int_CAMERA_DATA[2]-=1;  break;
				
				case 23:Int_CAMERA_DATA[3]-=1; break;
				
				
				
				case 24:Int_LINE_DATA[0]-=1;  break;
				
				case 25:Int_LINE_DATA[1]-=1;  break;
				case 26:Int_LINE_DATA[2]-=1;  break;
				
				case 27:Int_LINE_DATA[3]-=1; break;
				
								
				case 28:Int_saidao_data[0]-=1;  break;
				
				case 29:Int_saidao_data[1]-=1;  break;
				case 30:Int_saidao_data[2]-=1;  break;
				
				case 31:Int_saidao_data[3]-=1; break;
				
				case 32:Int_CESHI_DATA[0]-=1;  break;
				
				case 33:Int_CESHI_DATA[1]-=1;  break;
				case 34:Int_CESHI_DATA[2]-=1;  break;
				
			}
			systick_delay_ms(150);
		}
		if(m<10)
			lcd_showstr(0,m,"<");
		else if(m<20)
			lcd_showstr(0,m-10,"<");
		else if(m<30)
			lcd_showstr(0,m-20,"<");
		else if(m<40)
			lcd_showstr(0,m-30,"<");
		
		if(key_check(KEY_B) ==  KEY_DOWN)
		{
			set=0;
			
			FLASH_EraseSector(SectorNum);
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Direct_In, 8, 8);    //转向内环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Direct_Out, 8, 16);  // 转向外环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Motor_L, 8, 24);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Motor_R, 8, 32);       //电机环 
			FLASH_WriteSector(SectorNum, (const uint8*)&Int_Speed_set, 8, 40);       //电机环 
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Ring, 8, 48);       //电机环 
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Ring_Speed, 8, 56);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_ob_way, 8, 64);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_Chang_Speed, 8, 72);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_CAMERA_DATA, 8, 80);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_LINE_DATA, 8, 88);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_saidao_data, 8, 96);       //电机环
			FLASH_WriteSector(SectorNum, (const uint8*)Int_CESHI_DATA, 8, 104);       //电机环
			dsp_single_colour(BLACK);
			while(key_check(KEY_B) ==  KEY_DOWN);
		}
		
		
		
		Direct_In[0] = (float)Int_Direct_In[0]/1;
		Direct_In[1] = (float)Int_Direct_In[1]/1;
		Direct_In[2] = (float)Int_Direct_In[2]/1;
		Direct_In[3] = (float)Int_Direct_In[3]/1;
		
		Direct_Out[0] = (float)Int_Direct_Out[0]/10;
		Direct_Out[1] = (float)Int_Direct_Out[1]/1;
		Direct_Out[2] = (float)Int_Direct_Out[2]/10;
		Direct_Out[3] = (float)Int_Direct_Out[3]/1;
		
		PID_Motor_L[0] = (float)Int_Motor_L[0]/10;
		PID_Motor_L[1] = (float)Int_Motor_L[1]/10;
		PID_Motor_L[2] = (float)Int_Motor_L[2]/10;
		PID_Motor_L[3] = (float)Int_Motor_L[3]/10;
		
		PID_Motor_R[0] = (float)Int_Motor_R[0]/10;
		PID_Motor_R[1] = (float)Int_Motor_R[1]/10;
		PID_Motor_R[2] = (float)Int_Motor_R[2]/10;
		PID_Motor_R[3] = (float)Int_Motor_R[3]/10;
		Speed_Set=Int_Speed_set;
		Ring[0]=Int_Ring[0];  /****环岛***1右环   2左环***/
		Ring[1]=Int_Ring[1];  /****环岛***1右环   2左环***/
		Ring_Speed[0]=Int_Ring_Speed[0];  /****环岛***1右环   2左环***/
		Ring_Speed[1]=Int_Ring_Speed[1];  /****环岛***1右环   2左环***/
		Way_Flag=Int_ob_way[0];
		Obstacle_Angle[0]=Int_ob_way[1];
		Obstacle_Angle[1]=Int_ob_way[2];
		Obstacle_Line=Int_ob_way[3];
		//    Ring[1]=Int_Ring+1;
		//    if(Ring[1]>2)
		//    Ring[1]=1;
		
		
		Speed_Nor_High = (float)Int_Chang_Speed[0];
		Speed_Nor_Low = (float)Int_Chang_Speed[1];
		Speed_Up = (float)Int_Chang_Speed[2];
		
		Camera_data=Int_CAMERA_DATA[0];
		Side_Thres=Int_CAMERA_DATA[1];
		ob_Side_Thres=Int_CAMERA_DATA[2];
		yuanhuan_line=Int_CAMERA_DATA[3];
		MT9V032_CFG[7][1]=Camera_data;
		
		
		
		ring_distance[0]=Int_LINE_DATA[0];
		ring_distance[1]=Int_LINE_DATA[1];
		ring_distance[2]=Int_LINE_DATA[2];
		ring_distance[3]=Int_LINE_DATA[3];
		
		
		SAIDAO_DATA[0]=Int_saidao_data[0];
		SAIDAO_DATA[1]=Int_saidao_data[1];
		SAIDAO_DATA[2]=Int_saidao_data[2];
		SAIDAO_DATA[3]=Int_saidao_data[3];
		

		ceshi_data[0]=Int_CESHI_DATA[0];
		ceshi_data[1]=Int_CESHI_DATA[1];
		ceshi_data[2]=Int_CESHI_DATA[2];
		//SAIDAO_DATA[3]=Int_CESHI_DATA[3];
	}
}































void Mode_Set()
{
	int Set_Flag = 1;
	int Mode_Flag;
	
	while(Set_Flag)
	{
		switch(Mode_Flag)
		{
			case 0://直立模式
			showimage(xiaolian);
			break;
			case 1://直立跑动模式
			showimage(heilian);
			break;
			case 2://直立跑动模式
			showimage(GTR);
			break;
		}
		
		if(key_check(KEY_U) ==  KEY_DOWN)
		{
			Mode_Flag-=1;
			if(Mode_Flag<0)
				Mode_Flag = 2;
			
			while(key_check(KEY_U) ==  KEY_DOWN);  
		}
		
		if(key_check(KEY_D) == KEY_DOWN)
		{
			Mode_Flag+=1;
			if(Mode_Flag>2)
				Mode_Flag = 0;
			
			while(key_check(KEY_D) ==  KEY_DOWN);
		}
		
		if(key_check(KEY_B) ==  KEY_DOWN)
		{ 
			if(Mode_Flag==0)
				LCD_Flag=0;
			if(Mode_Flag==1)
				LCD_Flag=1;
			if(Mode_Flag==2)
			{
				LCD_Flag=2;
			}
			Set_Flag  = 0;
			dsp_single_colour(BLACK); 
			
			while(key_check(KEY_B) ==  KEY_DOWN);
		}
	}
}