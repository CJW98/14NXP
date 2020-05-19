#include "Otsumethod.h"

//*************���  ����colͼ����   rowͼ����

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
//          if(img_1[2*i][2*j] >Threshold_1) //�����ֵ   ��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//        for(i = 20; i < 40; i++)
//         {
//          for(j = 0; j < 94; j++)
//         {                                
//      //if(Image_Use[i][j] >GaveValue)//ƽ��ֵ��ֵ
//          if(img_2[2*i-40][2*j] >Threshold_2) //�����ֵ   ��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//        for(i = 40; i < 60; i++)
//         {
//          for(j = 0; j < 94; j++)
//         {                                
//      //if(Image_Use[i][j] >GaveValue)//ƽ��ֵ��ֵ
//          if(img_3[2*i-80][2*j] >Threshold_3) //�����ֵ   ��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����    
//           Pixle[i][j] =1;        
//          else                                        
//           Pixle[i][j] =0;
//         }    
//        }
//            
//
//}










extern float sys_time;

 //uint8_t image[ROW][COL];//�ɼ���ͼ��
uint8_t img[IMAGE_H][IMAGE_W];//��ֵ�����ͼ��
uint8_t Threshold;		//��ֵ
void Otsu(void)
{	
	uint8_t* src_img = *image;    //�Ҷ�ͼ��
	uint8_t* img_binary = *img;
	uint16_t AllGrayVal[256]={0};//���лҶ�ֵ�ĸ�������
	float AllGrayScale[256];
	uint16_t i,gray_val;
	float w0, w1;	//ǰ���ͱ��������ص�ռ��
	float u0, u1;	//ǰ���������������ƽ���Ҷ�
	float g,maxg=0;	//��䷽��
	
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
	
	//��ͬ���ص�ռ�ٷֱ�
	
	
	
	for(gray_val=0;gray_val<256;gray_val+=2)//�������䷽��ʱ����ֵ
	{
		w0 = w1 = u0 = u1 = 0;
		for(i=0;i<256;i+=2)
		{
			if(i<=gray_val)//С����ֵ,ǰ������
			{
				w0 += AllGrayScale[i];
				u0 += i * AllGrayScale[i];
			}
			else		//����
			{
				//w1 += AllGrayScale[i];
				u1 += i * AllGrayScale[i];
			}
		}
		w1 = 1.0f - w0;
		u0 /= w0;	//ǰ�����Ҷ�
		u1 /= w1;	//�������Ҷ�
		g = w0 * w1 * (u0-u1) * (u0-u1);// g=��0��1(��0-��1)^2 
		if(g>maxg && gray_val<128)//�и������䷽��
		{
			maxg = g;
			Threshold = gray_val;//�ҵ������䷽���Ӧ����ֵ
		}
	}
	
	
	
	
	//˫��
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
	src_img = *image;//��ʼʹ����ֵ��ֵ��
	pit_time_start(pit1);   
	for(i=0;i<IMAGE_SIZE;i++)
	{
		if(*(src_img++)<=Threshold)//С�ڹ̶���ֵ,ǰ������
			*(img_binary++) = 0X00;
		else
			*(img_binary++) = 0X01;
	}
	sys_time=pit_time_get(pit1)/(bus_clk_mhz);
}