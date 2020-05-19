#include "car.h"

char LCD_Flag=0;
int  stop_Flag=0;
int8 Target_Speed;
void Car_Run()
{
	
	
	if(key_check(KEY_B) ==  KEY_DOWN)
	{ 
		systick_delay_ms(10);
		dsp_single_colour(BLACK);
		while(key_check(KEY_B) ==  KEY_DOWN);	    
		Mode_Set();
	}
	
	
	if(key_check(KEY_D) ==  KEY_DOWN)
	{
		systick_delay_ms(10);
		dsp_single_colour(BLACK);
		while(key_check(KEY_D) ==  KEY_DOWN);
		Car_Debug();
	}
	
	
	if(LCD_Flag==1)
	{
		
		lcd_displayimage032(image[0], COL, ROW);
		
		/****划线****/
		
		int Point_Y,Point_X;
		int i;
		
		for(i=0;i<=128;i++)
		{
			lcd_drawpoint(i,25, GREEN);
			lcd_drawpoint(i,30, GREEN);
		}
		
		for(i=79;i>=0;i--)
		{
			Point_Y = (int)(i+0.5);
			Point_X = (int)(L_Side[i]*128/188)+0.5;
			if(Point_Y>=0&&Point_Y<=128)
			{
				lcd_drawpoint(Point_X,Point_Y, RED);
				//  lcd_drawpoint(Point_X+1,Point_Y,RED);
			}
			Point_X = (int)(R_Side[i]*128/188)+0.5;
			if(Point_Y>=1&&Point_Y<=128)
			{
				lcd_drawpoint(Point_X,Point_Y, BLUE);
				// lcd_drawpoint(Point_X-1,Point_Y, BLUE);
			}
			//	       Point_X = (int)(Mid_Line[i]*128/188+0.5);
			//	       if(Point_Y>=1&&Point_Y<=128)
			//	       {
			//		   lcd_drawpoint(Point_X,Point_Y, GREEN);
			//		   lcd_drawpoint(Point_X-1,Point_Y,GREEN);
			//	       }
			//	       
		}   
//		lcd_drawpoint(long_way[1],long_way[0], GREEN);
//		lcd_showint32(60, 8,  long_way[0],4);
//		lcd_showint32(0, 8,  long_way[1],4);
		//lcd_showint32(60, 8,  Direction_PWM,4);
		lcd_showstr(0,8,"start:");
		lcd_showint32(30, 8, jump_point,3); 
		lcd_showint32(0, 9,  Center_Deviation,4);
		lcd_showint32(30, 9,  DISS,4);
	}
	else if(LCD_Flag==2)
		LCD_Show();
//	else
//	{
//    lcd_showint32(0, 0, A_Start_Ok,3);  
//	lcd_showint32(0, 1, B_Start_Ok,3);  
//	}
//	
}


void LCD_Show()
{
	lcd_showstr(0,0,"S_L:");
	lcd_showint32(30, 0, Measure_Speed_L,3);  
	lcd_showstr(70,0,"S_R:");
	lcd_showint32(100,0, Measure_Speed_R,3);
	
	lcd_showstr(0,1,"ob:");
	lcd_showfloat(40,1, Obstacle_Flag,2,1);
	
	
	
	lcd_showint32(0, 2, A_Start_Ok,3);  
	lcd_showint32(40, 2, B_Start_Ok,3);  
//	lcd_showfloat(80,1, Ring[1],2,1);
//	
//	lcd_showstr(0,2,"start:");
////	lcd_showfloat(40,2, Ring_Speed[0],2,1);
////	lcd_showfloat(80,2, Ring_Speed[1],2,1);
//	
//	
//	lcd_showfloat(40,2, A_Reach_End,2,1);
	
	lcd_showstr(0,3,"flag:");
	lcd_showfloat(40,3,Ramp_Flag,2,1);//Broken_Flag=1;
	
	
	
	lcd_showstr(0,4,"speed:");
	lcd_showfloat(40,4,Target_Speed,2,1);//Target_Speed
	
	lcd_showstr(0,5,"data:");
	lcd_showint32(40,5,data,4);
	
	lcd_showstr(0,6,"DIR:");
	lcd_showfloat(40,6,Turn_offset*100,2,0);
//	
	lcd_showstr(0,7,"ag:");
	lcd_showfloat(40,7,Gyro_X,3,2);
//		lcd_showstr(0,5,"Stop:");
//	lcd_showfloat(40,5,Ramp_Distance,3,2);
//			lcd_showstr(0,6,"Way:");
//	lcd_showfloat(40,6,Way_Flag,3,2);
	lcd_showint32(0, 8,  ADC_End[1],4);
	lcd_showint32(40, 8,  ADC_End[5],4);
	lcd_showint32(80, 8,  ADC_End[4],4);
	
//	lcd_showint32(20, 8,  ADC_End[2],4);
//	lcd_showint32(60, 8,  ADC_End[3],4);
	lcd_showint32(0, 9,Sum_ADC[0],5);//Circle_Flag
	lcd_showint32(50, 9,Sum_ADC[1],5);
	
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      停车
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note
//-------------------------------------------------------------------------------------------------------------------
int Stop_Up_Flag=0, DelayStop, SlowStop;
void Car_Up(int set, int del_t, int slow_t)//延时时间,加速时间
{
	
	if(set&&!Stop_Up_Flag)
		
	{
		Stop_Up_Flag = slow_t;
		DelayStop = del_t;
		SlowStop = slow_t;
	}
	else
	{
		if(DelayStop)
			Target_Speed=0;
		else 
		{
			Target_Speed = (10-SlowStop) * 50 / Stop_Up_Flag;
			if(Target_Speed>=40)Speed_jiajia=0;
		}
	}
}