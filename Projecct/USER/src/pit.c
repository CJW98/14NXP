#include "pit.h"

//uint8 2ms_Flag=0;
char Flag_3ms=0,Flag_6ms=0,Flag_9ms=0,Flag_18ms=0;
extern float sys_time;
int Disable;//起跑线失能时间
void PIT0_IRQHandler(void)
{
      
	static uint8 Time_3ms = 0;
	static uint8 Time_6ms = 0;
	static uint8 Time_9ms = 0;
	static uint8 Time_18ms = 0;
	
	Time_6ms++;
	Time_9ms++;
	Time_18ms++;
	
	Flag_3ms=1;
	
	if(Time_6ms==2)
	{
	    Time_6ms=0;
	    Flag_6ms=1;
	
	}
	if(Time_9ms==3)
	{
	    Time_9ms=0;
	    Flag_9ms=1;
	}
	if(Time_18ms==6)
	{
	    Time_18ms=0;
	    Flag_18ms=1;	    
	}
//	Control();
	//Master_Control();
	Run_Control();
	PIT_FlAG_CLR(pit0);//清除标志位
}
 
extern int Stop_Flag, DelayStop, SlowStop;
int Beep=0;
void PIT1_IRQHandler(void)
{
//  if(Obstacle_Flag)
//    gpio_set (D10, 1);
//  else
//    gpio_set (D10, 0);

        if(Beep)
    {
        Beep--;
        gpio_set (D10, 1);
        if(!Beep)
        {
            gpio_set (D10, 0);
        }
    }

    /***延时停车***/
    if(DelayStop)
        DelayStop--;
    else if(SlowStop)
        SlowStop--;

  if(Disable)
   Disable--;
  
  PIT_FlAG_CLR(pit1);//清除定时器中断标志位    
}