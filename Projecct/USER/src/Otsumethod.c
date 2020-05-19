#include "Otsumethod.h"

//*************大津法  输入col图像列   row图像行

uint8 img_threshold = 0; 
uint8 Pixle[60][188]=0;
uint8 img_1[40][188]=0;
uint8 img_2[40][188]=0;
uint8 img_3[40][188]=0;

uint8 Threshold_1=0;
uint8 Threshold_2=0;
uint8 Threshold_3=0;



//void Otsu_Method(void)
//{   
//  
//
//
//          uint8 num=0;
//          uint8 i=0;
//          uint8 j=0;
//          for(num=0;num<40;num++)
//            memcpy(&img_1[num][0],&image[num][0],188);
//          for(num=40;num<80;num++)
//            memcpy(&img_2[num-40][0],&image[num][0],188);
//          for(num=80;num<120;num++)
//            memcpy(&img_3[num-80][0],&image[num][0],188);
//          Threshold_1 = GetOSTU(img_1); 
//          Threshold_2 = GetOSTU(img_2); 
//          Threshold_3 = GetOSTU(img_3); 
//         for(i = 0; i < 20; i++)
//         {
//          for(j = 0; j < 94; j++)
//         {                                
//
//          if(img_1[2*i][2*j] >Threshold_1) //大津法阈值   数值越大，显示的内容越多，较浅的图像也能显示出来    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//        for(i = 20; i < 40; i++)
//         {
//          for(j = 0; j < 94; j++)
//         {                                
//      //if(Image_Use[i][j] >GaveValue)//平均值阈值
//          if(img_2[2*i-40][2*j] >Threshold_2) //大津法阈值   数值越大，显示的内容越多，较浅的图像也能显示出来    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//        for(i = 40; i < 60; i++)
//         {
//          for(j = 0; j < 94; j++)
//         {                                
//      //if(Image_Use[i][j] >GaveValue)//平均值阈值
//          if(img_3[2*i-80][2*j] >Threshold_3) //大津法阈值   数值越大，显示的内容越多，较浅的图像也能显示出来    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//            
//
//}










extern float sys_time;

 //uint8_t image[ROW][COL];//采集的图像
uint8_t img[IMAGE_H][IMAGE_W];//二值化后的图像
uint8_t Threshold;		//阈值
void Otsu(void)
{	
	uint8_t* src_img = *image;    //灰度图像
	uint8_t* img_binary = *img;
	uint16_t AllGrayVal[256]={0};//所有灰度值的个数矩阵
	float AllGrayScale[256];
	uint16_t i,gray_val;
	float w0, w1;	//前景和背景的像素点占比
	float u0, u1;	//前景、背景和总体的平均灰度
	float g,maxg=0;	//类间方差
	
	uint16_t MinTop[2]={0};
	uint16_t MaxTop[2]={0};	
	
	for(i=0;i<IMAGE_SIZE;i++)
	{
		AllGrayVal[*(src_img++)]++;
	}
	for(i=0;i<256;i++)
	{
		AllGrayScale[i] = (float)AllGrayVal[i] / IMAGE_SIZE;
	}
	
	//相同像素点占百分比
	
	
	
	for(gray_val=0;gray_val<256;gray_val+=2)//找最大类间方差时的阈值
	{
		w0 = w1 = u0 = u1 = 0;
		for(i=0;i<256;i+=2)
		{
			if(i<=gray_val)//小于阈值,前景部分
			{
				w0 += AllGrayScale[i];
				u0 += i * AllGrayScale[i];
			}
			else		//背景
			{
				//w1 += AllGrayScale[i];
				u1 += i * AllGrayScale[i];
			}
		}
		w1 = 1.0f - w0;
		u0 /= w0;	//前景均灰度
		u1 /= w1;	//背景均灰度
		g = w0 * w1 * (u0-u1) * (u0-u1);// g=ω0ω1(μ0-μ1)^2 
		if(g>maxg && gray_val<128)//有更大的类间方差
		{
			maxg = g;
			Threshold = gray_val;//找到最大类间方差对应的阈值
		}
	}
	
	
	
	
	//双峰
	for(i=Threshold;i>0;i--)
	{
		if(MinTop[1]<AllGrayVal[i])
		{
			MinTop[1]=AllGrayVal[i];
			MinTop[0] = i;
			break;
		}
	}
	for(i=Threshold;i<128;i++)
	{
		if(MaxTop[1]<AllGrayVal[i])
		{
			MaxTop[1]=AllGrayVal[i];
			MaxTop[0] = i;
			break;
		}
	}
	Threshold = (MinTop[0]+MaxTop[0])/2;
	
	if(Threshold < 40) Threshold = 255;
	src_img = *image;//开始使用阈值二值化
	pit_time_start(pit1);   
	for(i=0;i<IMAGE_SIZE;i++)
	{
		if(*(src_img++)<=Threshold)//小于固定阈值,前景部分
			*(img_binary++) = 0X00;
		else
			*(img_binary++) = 0X01;
	}
	sys_time=pit_time_get(pit1)/(bus_clk_mhz);
}