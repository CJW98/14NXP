#include "Filter.h"
#include "math.h"

char Buff_Flag=0;

/***左右环判断****/
//void Data_Record(float adcval)
//{
//    uint8 i;
//    uint8 way=0,way2=0;
//    	for(i=29;i>0;i--)
//	{
//           turn_buff[i] = turn_buff[i-1];
//	}
//	turn_buff[0]=adcval;
//	for(i=29;i>=0;i--)
//	{    
//	    if(turn_buff[i] >0)
//		way++;
//	     else
//		way2++;
//
//	}
//	if(abs(way-way2)>10)
//	 Buff_Flag=1;
//}

//void Record_init(void)
//{
//    uint8 i;
//   for(i=29;i>=0;i--)
//    {
//	turn_buff[i] = 0;
//    }
//
//
//}


uint16 adc_buff[2][6]={0};
uint16_t Sum_ADC[2]=0;
void Filter(void)
{
	
	// uint16_t Sum_ADC[2]=0;
	int16  i,j,k,temp,count;
	int16  ad_valu[5][5],ad_valu1[5],ad_sum[5];
	
	for(i=0;i<5;i++)
	{
		ad_valu[0][i]=adc_once(AD1, ADC_12bit);  			// 左
		ad_valu[1][i]=adc_once(AD2, ADC_12bit);     		// 左2
		ad_valu[2][i]=adc_once(AD3, ADC_12bit);  			//右2 
		ad_valu[3][i]=adc_once(AD4, ADC_12bit);     		// 右	
		ad_valu[4][i]=adc_once(AD5, ADC_12bit);     		// 	
	}
	
	
	
	
	/*=========================冒泡排序升序==========================*///舍弃最大值和最小值
	for(i=0;i<5;i++)
	{
		for(j=0;j<4;j++)
		{
			for(k=0;k<4-j;k++)
			{
				if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
				{
					temp = ad_valu[i][k+1];
					ad_valu[i][k+1] = ad_valu[i][k];
					ad_valu[i][k] = temp;
				}
			}
		}
	}
	
	/*===========================中值滤波=================================*/
	for(i=0;i<5;i++)    //求中间三项的和
	{
		ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
		ad_valu1[i] = ad_sum[i] / 3;
	}
	
	
	for(i=0;i<5;i++)            //将数值中个位数除掉
	{
		ADC_End[i+1] = (int16)(ad_valu1[i]/10*10);
		
	}
	
	adc_buff[0][3] = adc_buff[0][2];
	adc_buff[0][2] = adc_buff[0][1];
	adc_buff[0][1] = adc_buff[0][0];
	adc_buff[0][0] = ADC_End[1];//输入值
	
	
	adc_buff[1][3] = adc_buff[1][2];
	adc_buff[1][2] = adc_buff[1][1];
	adc_buff[1][1] = adc_buff[1][0]; 
	adc_buff[1][0] = ADC_End[4];//输入值
	
	Sum_ADC[0]=adc_buff[0][0]+adc_buff[0][1]+adc_buff[0][2]+adc_buff[0][3];
	
	Sum_ADC[1]=adc_buff[1][0]+adc_buff[1][1]+adc_buff[1][2]+adc_buff[1][3];
	Ring_Stable();

	
	
	
}



