#include "handle.h"

char Starting_Flag=0;

/***flash参数*****/
uint8_t Side_Thres=25;      //边界阈值
uint8_t ob_Side_Thres=25;   //壁障边界阈值
uint8_t ob_camera=10;
//const int Road[IMAGE_H]=
//{					
//	16 , 18, 20, 21, 23, 24, 26, 27, 29, 31,
//	32 , 34, 35, 37, 38, 40, 42, 43, 45, 46,
//	48 , 49, 51, 53, 54, 56, 57, 59, 60, 62,
//	64 , 65, 67, 68, 70, 71, 73, 75, 76, 78,
//	79 , 81, 83, 84, 86, 87, 89, 90, 92, 94,
//	95 , 97, 98,100,101,103,105,106,108,109,
//	111,112,114,116,117,119,120,122,123,125,
//	127,128,130,131,133,134,136,138,139,141,
//};	
const int Road[IMAGE_H]=
{10,11,13,13,14,15,16,17,18,19,
20,21,22,23,25,27,29,31,33,35,
37,39,41,43,45,47,49,51,53,55,
57,57,60,63,65,67,69,71,73,75,
77,79,81,83,85,88,90,92,94,96,
98,100,102,104,106,107,109,111,113,115,
117,119,120,122,123,125,127,129,131,133,
133,135,137,139,140,141,143,145,147,147,
};
int Gearshift;  //档位
char dis_circle=0;   //环岛防止误判断道和避障

uint8_t  SAIDAO_DATA[4]={10,10,50,60};//第一个壁障线  第二个壁障线  起跑线间隔  断道路径
//const int Road[IMAGE_H]=
//{					
//	21,24,26,27,29,31,33,35,37,39,
//	41,43,45,47,49,51,53,55,57,59,
//	61,63,65,67,69,71,73,75,77,79,
//	81,83,85,87,89,91,93,95,97,99,
//	101,103,105,107,109,111,113,115,117,118,
//	119,120,121,122,124,126,128,129,130,131,
//	132,133,134,135,136,137,139,141,143,145,
//	147,149,151,153,155,157,159,161,163,165,
//};
int Control_Line=60;//60

int Center_Deviation;//中心偏差

/**********************避障变量参数****************/
uint8 Way_Flag ;
int Obstacle_Flag;
char Step_One;
uint16 Width[3];

int16 Difference;
/**********************避障变量参数****************/


/**********************坡道变量参数***************/
uint8 Ramp_Width[3],Ramp_Add;

/********************************************/

int L_Side[80],R_Side[80],Mid_Line[80];//左右边线,中线
uint8 L_Find_Flag = 0,R_Find_Flag = 0,L_Find = 0,R_Find = 0;//找线标志
int SearchMid = 80;//从上一图像基点中线开始搜索下一图像基点
uint8 Turn_Point_L,Turn_Point_R,Times_L,Times_R,Extend_L,Extend_R,Dir_L,Dir_R;//计算拐点变量
uint8 L_Miss,R_Miss;//丢线起始行
float Lsm_k=0,Lsm_b=0;//最小二乘法计算参数
int L_Miss_Point[2],R_Miss_Point[2];

int Forecast_Point;//二乘法预测边界点
int Scan_Start,Scan_End;//追踪寻线扫描起始点,结束点
float Cos_Angle;//计算拐点角度
float Compensate_K;
char Broken_Flag_Left = 0,
Broken_Flag_Right = 0,
Broken_Up_Flag = 0;
char Broken_Flag=0;
void Image_Handle()
{
	Searching_Nearby();//搜寻最近几行边界--------采集错误不处理保持上一次的边线
	
	Side_Line();//搜寻全部边界
	Boundary_Analysis();
	Midline_Extraction();
	
}
uint8 long_way[2];
void Line(void)
{
	uint8 i,j;
	int RoadPoint[2],LinePoint[2];
	for(j=80;j<=110;j++)
	{
		for(i=50;i>=30;i--)
		{
			if((abs(image[i][j-2]-image[i][j+2])*100/(image[i][j-2]+image[i][j+2]))>20)    //找到该行右边界
				break;
		}
		long_way[0] = long_way[0]>i ? long_way[0] : i;
		if(long_way[0]<=i) long_way[1] = j;  
	}
	
	RoadPoint[0] =long_way[0];
	RoadPoint[1] =long_way[1];
	
	LinePoint[0] = 79;
	LinePoint[1] = 0;
	
	
	Connection(L_Boundary,RoadPoint,LinePoint);//连接两端点
	
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      搜寻近处左右边界基点
//  @param      NULL
//  @return     uint8
//  @since      v1.0
//  Sample usage:
//  @note       L_Find,R_Find
//-------------------------------------------------------------------------------------------------------------------
uint8 Searching_Nearby(void)
{
	char temp1,temp2;
	int i,j;
	L_Find = 0;//左边基点
	R_Find = 0;//右边基点
	L_Find_Flag = 0;//左基点标志
	R_Find_Flag = 0;//右基点标志
	
	L_Miss = 0;//左丢线行
	R_Miss = 0;//右丢线行
	
	Turn_Point_L = 0;//左边拐点
	Turn_Point_R = 0;//右边拐点
	Times_L = 1;//找左拐点次数
	Times_R = 1;//找右拐点次数
	Extend_L = 1;//初始左边延伸方向
	Extend_R = 0;//初始右边延伸方向
	Dir_L = 1;//寻线左边延伸方向
	Dir_R = 0;//寻线右边延伸方向
	
	for(i=IMAGE_H-1;i>=0;i--)//所有左边界赋值-1
	{
		L_Side[i] = -1;
		R_Side[i] = IMAGE_W;
	}
	
	for(i=IMAGE_H-1; i>=60; i--)//搜索20行图像寻找两边线基点
	{
		for(j=SearchMid; j>=0; j=j--)   //从最左边向右3/4图像搜索左边界
		{
			if(Side_Thres_Count(image[i][j+3], image[i][j])>=Side_Thres)    //找到该行左边界
			{
				L_Side[i] = j;//记录该行左边界
				L_Find_Flag++;//找线标志+1
				if(i<=IMAGE_H-1-2)
				{
					if(L_Side[i] < L_Side[i+2]) Extend_L = 0;//记录向左延伸方向
					if(L_Side[i] > L_Side[i+2]) Extend_L = 1;//记录向右延伸方向
				}
				break;
			}
		}
		if(L_Find_Flag==Near_Lines)
		{
			L_Find = i;             //记录此边界
			for(i=L_Find+1; i<=IMAGE_H-1; i++)
			{
				if(L_Side[i] == -1)
				{
					L_Side[i] = L_Side[L_Find];
				}
			}
			break;
		}
	}
	
	for(i=IMAGE_H-1; i>=60; i--)//搜索20图像寻找两边线基点
	{
		for(j=SearchMid; j<=IMAGE_W-1; j=j++)   //从中心点靠左向右搜索
		{
			if(Side_Thres_Count(image[i][j-3], image[i][j])>=Side_Thres)    //找到该行右边界
			{
				R_Side[i] = j;//记录该行右边界
				R_Find_Flag++;//找线标志+1
				if(i<=IMAGE_H-1-2)
				{
					if(R_Side[i] < R_Side[i+2]) Extend_R = 0;//记录向左延伸方向
					if(R_Side[i] > R_Side[i+2]) Extend_R = 1;//记录向右延伸方向
				}
				break;
			}
		}
		if(R_Find_Flag==Near_Lines)                //找到足够边界
		{
			R_Find = i;             //记录此边界
			for(i=R_Find+1; i<=IMAGE_H-1; i++)
			{
				if(R_Side[i] == IMAGE_W)
				{
					R_Side[i] = R_Side[R_Find];
				}
			}
			break;
		}
	}
	SearchMid = 0;//计算最近10行中心为下一图像扫描起始点
	for(i=75;i>=66;i--)
	{
		SearchMid += (L_Side[i] + R_Side[i])/2;
	}
	SearchMid/=10;
	
	//   //   断道识别
	//  if(!L_Find&&!R_Find)
	//  {
	//    
	//      for(j=0; j<=IMAGE_W-1; j++)
	//      {
	//	  if(image[79][j]>50)break;
	//      }
	//      if(j>=IMAGE_W-2)
	//      {
	//	  temp1=1;
	//      }
	//      
	//      
	//      
	//       for(j=IMAGE_W-1; j>=0; j--)
	//      {
	//	  if(image[78][j]>50)break;
	//      }
	//      if(j<2)
	//      {
	//	  temp2=1;
	//      }
	//      if(temp1&&temp2)
	//	  stop_Flag=1;
	//      else
	//	   stop_Flag=0;
	//  }
	//   
	
	
	
	/***断道***/
	if(!Way_Flag && !Obstacle_Flag&&!Ramp_Flag&&!dis_circle)
	{
		Broken_Flag_Left = 0;
		Broken_Flag_Right = 0;
		Broken_Up_Flag = 0;
		// for(i=IMAGE_H-1;i>=75;i--)
		// {
		for(j=IMAGE_W/2;j>=0;j=j-2)
		{
			if(image[79][j]>=65 || abs(Side_Thres_Count(image[79][j+3],image[79][j]))>=Side_Thres)
				break;
			//          else
			//            return;
		}
		//   if(j<=0) Ob++;
		// }
		if(j<=0) Broken_Flag_Left=1;
		
		//   for(i=IMAGE_H-1;i>=75;i--)
		//  {
		for(j=IMAGE_W/2;j<=IMAGE_W-1;j=j+2)
		{
			if(image[79][j]>=65 || abs(Side_Thres_Count(image[79][j-3],image[79][j]))>=Side_Thres)
				break;
			//          else
			//            return;
		}
		// if(j>=187) Ob++;
		// }
		if(j>=187) Broken_Flag_Right=1;
		//  Ob=0;
		
		for(i=76;i>=67;i--)
		{
			if(image[i+3][94]>=65 || abs(Side_Thres_Count(image[i+3][94],image[i][94]))>=Side_Thres)
				break;
		}
		if(i<=67) Broken_Up_Flag = 1;
		
		if(Broken_Flag_Right && Broken_Flag_Left && Broken_Up_Flag)
		{
			Broken_Flag=1;
			//Way_Flag++;
		}
		//		else
		//			Broken_Flag=0;
	}
	
	
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      寻边线
//  @param      NULL0
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Lsm_k,Lsm_b
//-------------------------------------------------------------------------------------------------------------------
//uint8_t image[ROW][COL];//采集的图像长*宽  80*188
void Side_Line(void)
{
	int i,j,turn,Ob=0;
	
	if(L_Find)//左边寻到边线
	{
		for(i=L_Find-1;i>=0;i--)//从基点开始寻线
		{
			Forecast_Point = L_Side[i+1];//寻线点定为上一边界
			
			Scan_Start = Forecast_Point + Scan_Width;//寻线初始点
			Scan_End = Forecast_Point - Scan_Width;//寻线终止点
			if(Scan_Start > IMAGE_W-1-1-3)     //起始点限幅
				Scan_Start = IMAGE_W-1-1-3;
			if(Scan_End < 1)        //终止点限幅
				Scan_End = 1;
			
			for(j=Scan_Start;j>=Scan_End;j--)
			{
				if(Side_Thres_Count(image[i][j+3], image[i][j])>=Side_Thres)    //找到左边界右侧
				{
					L_Side[i] = j;
					if(L_Side[i]<L_Side[i+2])Dir_L = 0;//向左延伸
					if(L_Side[i]>L_Side[i+2])Dir_L = 1;//向右延伸
					break;
				}
			}
			if(Dir_L!=Extend_L && Times_L && L_Miss==0 && i<IMAGE_H-10)
			{
				if(Dir_L)//由左延伸变为向右延伸
				{
					if(L_Side[i+1]>L_Side[i+2])turn = i+2;  //取最左边的点为拐点
					else turn = i+1;
				}
				else //由右延伸变为左延伸
				{
					if(L_Side[i+1]>L_Side[i+2])turn = i+1;  //取最右边的点为拐点
					else turn = i+2;
				}
				Turn_Point_L = turn;
				Times_L = 0;
			}
			if(L_Side[i]==-1&&L_Miss==0)//如果之前没丢线
			{
				L_Miss = i;//记录丢线
			}
		}
	}
	else                    //左边界没有寻到线
	{
		L_Miss = IMAGE_H-1;
	}
	
	if(R_Find)
	{
		for(i=R_Find-1;i>=0;i--)//从基点开始寻线
		{
			Forecast_Point = R_Side[i+1];//寻线点定为上一边界
			
			Scan_Start = Forecast_Point - Scan_Width;//寻线初始点
			Scan_End = Forecast_Point + Scan_Width;//寻线终止点
			if(Scan_Start < 1+3)      //起始点限幅
				Scan_Start = 1+3;
			if(Scan_End > IMAGE_W-1-1)       //终止点限幅
				Scan_End = IMAGE_W-1-1;
			
			for (j=Scan_Start; j<=Scan_End; j++)
			{
				if(Side_Thres_Count(image[i][j-3], image[i][j])>=Side_Thres)      //找到右边界左侧
				{
					R_Side[i] = j;
					if(R_Side[i]<R_Side[i+2])Dir_R = 0;//向左延伸
					if(R_Side[i]>R_Side[i+2])Dir_R = 1;//向右延伸
					break;
				}
			}
			if(Dir_R!=Extend_R && Times_R && R_Miss==0 && i<IMAGE_H-10)
			{
				if(Dir_R)//由左延伸变为向右延伸
				{
					if(R_Side[i+1]>R_Side[i+2])turn = i+2;  //取最左边的点为拐点
					else turn = i+1;
				}
				else //由右延伸变为左延伸
				{
					if(R_Side[i+1]>R_Side[i+2])turn = i+1;  //取最右边的点为拐点
					else turn = i+2;
				}
				Turn_Point_R = turn;
				Times_R = 0;
			}
			if(R_Side[i]==IMAGE_W && R_Miss==0)
			{
				R_Miss = i;
			}
		}
	}
	else
	{
		R_Miss = IMAGE_H-1;
	}
	// cha8 = R_Miss - L_Miss;
	
	
	
	/*****壁障*****/
	char line_Flag=0;
	memset(Width,0,sizeof(Width));
	if(Way_Flag&&L_Find&&R_Find)//&&!Disable&&!dis_circle)
	{
		for(i=50;i>=SAIDAO_DATA[0];i--)
		{
			if(abs(Side_Thres_Count(image[i+3][IMAGE_W/2],image[i][IMAGE_W/2]))>=ob_Side_Thres)
			{
				Width[1] = i;
				break;
			}
			else
				Width[1] = 0;
		}
		if(Width[1])
		{
			for(i=50;i>=SAIDAO_DATA[0];i--)
			{
				if(abs(Side_Thres_Count(image[i+3][IMAGE_W/2+15],image[i][IMAGE_W/2+15]))>=ob_Side_Thres)
				{
					Width[2] = i;
					break;
				}
				else
					Width[2] = 0;
			}
			for(i=50;i>=SAIDAO_DATA[0];i--)
			{
				if(abs(Side_Thres_Count(image[i+3][IMAGE_W/2-15],image[i][IMAGE_W/2-15]))>=ob_Side_Thres)
				{
					Width[0] = i;
					break;
				}
				else
					Width[0] = 0;
			}
		}
		//    if(Width[1])
		//    {
		//	for(j=94;j>5;j--)
		//	{
		//     if(abs(Side_Thres_Count(image[Width[1]+5][j+3],image[Width[1]+5][j]))>=Side_Thres)
		//	 {
		//		 line_Flag=1;
		//		 break;
		//		 
		//	 }
		//	 else
		//		 line_Flag=0; 
		//	
		//	}
		//	}
		
		
		if(Width[1]>=SAIDAO_DATA[0] && Width[1]<=55 && abs(Width[0] - Width[1])<=2 && abs(Width[1] - Width[2])<=2)
			Step_One = 1;
		else
			Step_One = 0;
		if(abs(Width[0] - Width[2])<=3 && Step_One &&data<1200)
		{
			Obstacle_Flag = 1;
			Beep=2;
			Way_Flag--;
		}
//		
//		if(data<=600)
//		{
//			Obstacle_Flag = 1;
//			Way_Flag--;
//		}
		//    else
		//      Obstacle_Flag = 0;
	}
	
	//    else
	//      Obstacle_Flag = 0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      边线分析,判断左右急弯,环岛Roundabout
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Miss,Road
//-------------------------------------------------------------------------------------------------------------------
int img_type;//图像类型
void Boundary_Analysis(void)
{
	int i,j;
	float compensate_k;//赛道宽度补偿比例
	if(L_Find && R_Find)													  //两边都找到边线
	{
		if(L_Miss < R_Miss)	img_type = 0;									  //左边丢线少,二乘法,右边用左边补
		
		else				img_type = 1;  									  //右边丢线少,二乘法,左边用右边补
	}
	else if(L_Find)			img_type = 2;									  //只找到左边.左边二乘法,右边用左边补
	
	else if(R_Find)			img_type = 3;									  //只找到右边,右边二乘法,左边用右边补
	
	if(Camera_Flag&&Buff_Flag==1)                 img_type = 4;
	if(Camera_Flag&&Buff_Flag==2)                 img_type = 5;
	if(Ring_OutFlag&&Buff_Flag==1)             img_type = 6;
	if(Ring_OutFlag&&Buff_Flag==2)                 img_type = 7;
	switch(img_type)
	{
		case 0 :case 2 :														  //左边丢线少,二乘法,右边用左边补
		{
			if(R_Find) compensate_k = (float)(R_Side[R_Miss+1] - L_Side[R_Miss+1])/Road[R_Miss+1];
			
			else       compensate_k = 1.3;
			
			//if(Rounding_L || Rounding_R) compensate_k = 1;
			
			Least_Square_Method(L_Boundary,L_Miss+1);
			
			for(i=L_Miss+1;i>=0;i--)
			{
				L_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);  				  //由拟合直线计算横坐标
			}
			for(i=R_Miss+1;i>=0;i--)
			{
				R_Side[i] = (int)(L_Side[i] + compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			break;
		}
		case 1 :case 3 :													   	  //右边丢线少,二乘法,左边用右边补
		{
			if(L_Find) compensate_k = (float)(R_Side[L_Miss+1] - L_Side[L_Miss+1])/Road[L_Miss+1];
			
			else compensate_k = 1.3;
			
			//if(Rounding_L || Rounding_R) compensate_k = 1;
			
			Least_Square_Method(R_Boundary,R_Miss+1);
			
			for(i=R_Miss+1;i>=0;i--)
			{
				R_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);   				  //由拟合直线计算横坐标
			}
			for(i=L_Miss+1;i>=0;i--)
			{
				L_Side[i] = (int)(R_Side[i] - compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			break;
		}
		
		
		case 4 :													   	  //右边丢线少,二乘法,左边用右边补
		{
			
			compensate_k = 0.7;	    
			Least_Square_Method(R_Boundary,R_Miss+1);
			for(i=R_Miss+1;i>=0;i--)
			{
				R_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);   				  //由拟合直线计算横坐标
			}
			if(!R_Find)
			{
				for(i=IMAGE_H-1;i>=IMAGE_H-30;i--)
				{
					R_Side[i] = 187;
				}
				
			}
			for(i=IMAGE_H-1;i>=0;i--)
			{
				L_Side[i] = (int)(R_Side[i] - compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			
			break;
		}
		case 5 :														  //左边丢线少,二乘法,右边用左边补
		{
			compensate_k = 0.7;	 
			Least_Square_Method(L_Boundary,L_Miss+1);
			for(i=L_Miss+1;i>=0;i--)
			{
				L_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);  				  //由拟合直线计算横坐标
			}
			if(!L_Find)
			{
				for(i=IMAGE_H-1;i>=IMAGE_H-30;i--)
				{
					L_Side[i] = 1;
				}
				
			}
			for(i=IMAGE_H-1;i>=0;i--)
			{
				R_Side[i] = (int)(L_Side[i] + compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			break;
		}
		
		
		case 6 :														  //左边丢线少,二乘法,右边用左边补
		{
			compensate_k = 1.0;	 
			Least_Square_Method(L_Boundary,L_Miss+1);
			for(i=L_Miss+1;i>=0;i--)
			{
				L_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);  				  //由拟合直线计算横坐标
			}
			if(!L_Find)
			{
				for(i=IMAGE_H-1;i>=IMAGE_H-30;i--)
				{
					L_Side[i] = 1;
				}
				
			}
			for(i=IMAGE_H-1;i>=0;i--)
			{
				R_Side[i] = (int)(L_Side[i] + compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			break;
		}
		case 7 :													   	  //右边丢线少,二乘法,左边用右边补
		{
			
			compensate_k = 1.0;	    
			Least_Square_Method(R_Boundary,R_Miss+1);
			for(i=R_Miss+1;i>=0;i--)
			{
				R_Side[i] = (int)(Lsm_k * i + Lsm_b + 0.5f);   				  //由拟合直线计算横坐标
			}
			if(!R_Find)
			{
				for(i=IMAGE_H-1;i>=IMAGE_H-30;i--)
				{
					R_Side[i] = 187;
				}
				
			}
			for(i=IMAGE_H-1;i>=0;i--)
			{
				L_Side[i] = (int)(R_Side[i] - compensate_k * Road[i] + 0.5f); //由拟合直线计算横坐标
			}
			
			break;
		}
		
		default:break;
	}
	Rampway();
	if(!Disable)
		StartingLine();
	
	
	
	/****变速判断*****/
	int Distance = IMAGE_H/2;
	Gearshift = 0;//常速
	
	for(j=89; j<=98; j++)//判断是否全速(图像中心宽10,长80-2)
	{
		for(i=IMAGE_H/2; i>=0; i--)
		{
			if(image[i][j]<50)
				break;
		}
		if(i<Distance)
			Distance = i;
	}
	if(Distance<=2)
		Gearshift=3;//全速
	else//判断是否高速(图像中心宽10,长80-7)
	{
		Distance=IMAGE_H/2;
		for(j=84; j<=103; j++)
		{
			for(i=IMAGE_H/2; i>=0; i--)
			{
				if(image[i][j]<50)
					break;
			}
			if(i<Distance)
				Distance = i;
		}
		if(Distance<=7)
			Gearshift=2;//高速
		else//判断是否加速(图像中心宽10,长80-12)
		{
			Distance=IMAGE_H/2;
			for(j=79; j<=108; j++)
			{
				for(i=IMAGE_H/2; i>=0; i--)
				{
					if(image[i][j]<50)
						break;
				}
				if(i<Distance)
					Distance = i;
			}
			if(Distance<=12)
				Gearshift=1;//加速
		}
	}
	
	StraightRoad();
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      三角形三边求角度
//  @param      uint8 Boundary,uint8 Line
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Lsm_k,Lsm_b
//-------------------------------------------------------------------------------------------------------------------
float Side_Angle(uint8 Boundary,uint8 Point,uint8 Size)//求角度
{
	int x1,x2,x3,y1,y2,y3;
	float a,b,c,Cos_c;
	if(Boundary==L_Boundary)
	{
		y2 = Point;
		x2 = L_Side[y2];
		y1 = Point - Size;
		x1 = L_Side[y1];
		y3 = Point + Size;
		x3 = L_Side[y3];
	}
	if(Boundary==R_Boundary)
	{
		y2 = Point;
		x2 = R_Side[y2];
		y1 = Point - Size;
		x1 = R_Side[y1];
		y3 = Point + Size;
		x3 = R_Side[y3];
	}
	a=FastSqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	b=FastSqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3));
	c=FastSqrt((x1-x3)*(x1-x3)+(y1-y3)*(y1-y3));
	
	Cos_c = (a*a+b*b-c*c)/(2*a*b);
	return Cos_c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      由赛道两边界求得中线
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Fill_k,Fill_b,从(Miss+2)补到(Refind)
//-------------------------------------------------------------------------------------------------------------------
void Midline_Extraction(void)
{
	int i;
	float cen = 0;
	
	// Control_Line = (int)(Con_k * Measure_Speed + Con_b + 0.5);
	
	//    if(Control_Line>Nearline_Control)Control_Line = Nearline_Control;
	//
	//    if(Control_Line<Farline_Control)Control_Line = Farline_Control;
	
	for(i=Control_Line+15; i>=Control_Line; i--)    //中线加权平均
	{
		cen += (float)(L_Side[i] + R_Side[i])/2;   // * Line_Weight[i];
	}
	cen /= 16;
	
	Center_Deviation = (int)(cen-94+0.5);//赛道偏差
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      两个像素点之间的差比和
//  @param
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       像素A, 像素B
//-------------------------------------------------------------------------------------------------------------------
int16 Side_Thres_Count(int16 Pixel_A, int16 Pixel_B)
{
	
	Difference =(abs(Pixel_A-Pixel_B)*100/(Pixel_A+Pixel_B));
	
	return Difference;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      连接边线两点
//  @param      uint8 Boundary,uint8 Line
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Lsm_k,Lsm_b
//-------------------------------------------------------------------------------------------------------------------
void Connection(uint8 Boundary,int Point0[2],int Point1[2])
{
	int start,end,i;
	float connect_k,connect_b;
	if(Point0[0] == Point1[0])
	{
		if(Boundary==L_Boundary)
		{
			L_Side[Point0[0]] = Point0[1];
		}
		if(Boundary==R_Boundary)
		{
			R_Side[Point0[0]] = Point0[1];
		}
	}
	else
	{
		start = Point0[0]>Point1[0] ? Point0[0]:Point1[0];
		end = Point0[0]>Point1[0] ? Point1[0]:Point0[0];
		connect_k = (float)(Point0[1] - Point1[1]) / (Point0[0] - Point1[0]);
		connect_b = Point0[1] - connect_k * Point0[0];
		if(Boundary==L_Boundary)
		{
			for(i=start; i>=end; i--)
			{
				L_Side[i] = (int)(connect_k * i + connect_b + 0.5);
			}
		}
		if(Boundary==R_Boundary)
		{
			for(i=start; i>=end; i--)
			{
				R_Side[i] = (int)(connect_k * i + connect_b + 0.5);
			}
		}
	}
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      最小二乘法
//  @param      uint8 Boundary,uint8 Line
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Lsm_k,Lsm_b
//-------------------------------------------------------------------------------------------------------------------



void Least_Square_Method(uint8 Boundary,uint8 Line)//最小二乘法
{
	uint8 i;
	int Lsm_X[Lsm_Lines],Lsm_Y[Lsm_Lines];//最小二乘法横纵坐标
	float Lsm_ave_x = 0,Lsm_ave_y = 0,Numerator = 0,Denominator = 0;//最小二乘法X,Y均值,最小二乘法计算中的分子,分母
	
	if(Boundary == L_Boundary)
	{
		for(i=0; i<Lsm_Lines; i++)
		{
			Lsm_X[i] = Line+i+1;
			Lsm_Y[i] = L_Side[Line+i+1];
		}
	}
	if(Boundary == R_Boundary)
	{
		for(i=0; i<Lsm_Lines; i++)
		{
			Lsm_X[i] = Line+i+1;
			Lsm_Y[i] = R_Side[Line+i+1];
		}
	}
	for(i=0; i<Lsm_Lines; i++)
	{
		Lsm_ave_x += Lsm_X[i];
		Lsm_ave_y += Lsm_Y[i];
	}
	Lsm_ave_x /= Lsm_Lines;
	Lsm_ave_y /= Lsm_Lines;
	for(i=0; i<Lsm_Lines; i++)
	{
		Denominator += Lsm_X[i] * Lsm_X[i];
	}
	Denominator -= Lsm_Lines * Lsm_ave_x * Lsm_ave_x;
	
	for(i=0; i<Lsm_Lines; i++)
	{
		Numerator += Lsm_X[i] * Lsm_Y[i];
	}
	Numerator -= Lsm_Lines * Lsm_ave_x * Lsm_ave_y;
	
	Lsm_k = Numerator / Denominator;
	Lsm_b = Lsm_ave_y - Lsm_k * Lsm_ave_x;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      起跑线
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Miss,Road
//-------------------------------------------------------------------------------------------------------------------
//void StartingLine(void)
//{
//  uint8 jump_point = 0,j;
//  if( L_Miss<30 && R_Miss<30 && L_Side[30]>L_Side[79] && R_Side[30]<R_Side[79])
//  {
//    for(j=L_Side[59]; j<=R_Side[59]; j++)
//    {
//      if((abs(image[59][j-2]-image[59][j+2])*100/(image[59][j-2]+image[59][j+2]))>20)jump_point++;
//    }
//    for(j=L_Side[39]; j<=R_Side[39]; j++)
//    {
//      if((abs(image[39][j-2]-image[39][j+2])*100/(image[39][j-2]+image[39][j+2]))>20)jump_point++;
//    }
//    if(jump_point>10)
//    {	
//      Starting_Flag=1;
//      //Beep = 2;
//      // Car_Stop(1,2,6);
//    }
//  }
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      坡道
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Miss,Road
//-------------------------------------------------------------------------------------------------------------------
//
//void Rampway(void)
//{
//	uint8 i,j;
//	//	if(!Ramp_Flag)
//	//	{
//	//    if(L_Miss<20&&R_Miss<20)//进行坡道判断
//	//    {
//	//        if(R_Side[30]-L_Side[30]>Road[30]*13/10 && R_Side[50]-L_Side[50]>Road[50]*13/10 && L_Side[20]>L_Side[79] && R_Side[20]<R_Side[79])
//	//        {
//	//         Beep = 2;
//	//         Ramp_Flag=1;
//	//        }
//	//    }
//	//	}
//	if(L_Find && R_Find)
//	{
//		for(i=40;i>=2;i--)
//		{
//			if(image[i][IMAGE_W/2]<=70 || Side_Thres_Count(image[i+3][IMAGE_W/2],image[i][IMAGE_W/2])>=Side_Thres)
//				break;
//		}
//		Ramp_Width[1] = i;
//		if(Ramp_Width[1] == 1)
//		{
//			for(i=40;i>=2;i--)
//			{
//				if(image[i][IMAGE_W/2]<=70 || Side_Thres_Count(image[i+3][IMAGE_W/2-15],image[i][IMAGE_W/2-15])>=Side_Thres)
//					break;
//			}
//			Ramp_Width[0] = i;
//			for(i=40;i>=2;i--)
//			{
//				if(image[i][IMAGE_W/2]<=70 || Side_Thres_Count(image[i+3][IMAGE_W/2+15],image[i][IMAGE_W/2+15])>=Side_Thres)
//					break;
//			}
//			Ramp_Width[2] = i;
//		}
//		for(j=IMAGE_W/2-20;j<=IMAGE_W/2+20;j++)
//		{
//			for(i=40;i>=2;i--)
//			{
//				if(image[i][IMAGE_W/2]<=70 || Side_Thres_Count(image[i+3][j],image[i][j])>=Side_Thres)
//					break;
//			}
//			if(i==1) 
//				Ramp_Add++;
//		}
//		if(Ramp_Width[0]==1 && Ramp_Width[1]==1 &&Ramp_Width[2]==1&& Ramp_Add>=40)
//			Ramp_Flag = 1;
////		else
////			Ramp_Flag = 0;
//	}
////	else
////		Ramp_Flag = 0;
//}


void Rampway(void)
{
	if(!Ramp_Flag)
	{
		if(L_Miss<10&&R_Miss<10&&L_Find == 70 && R_Find ==70&&!Ring_state)//进行坡道判断
		{
			if(R_Side[30]-L_Side[30]>Road[30]*11/10 && R_Side[50]-L_Side[50]>Road[50]*11/10 && L_Side[20]>L_Side[79] && R_Side[20]<R_Side[79]&&ADC_End[5]>(ceshi_data[2]*100)&&ADC_End[1]<2000&&ADC_End[4]<2000)
			{
				Beep = 2;
				Ramp_Flag=1;
			}
		}
	}
}



uint8 jump_point=0;
void StartingLine(void)
{
	uint8  j;
	
	jump_point=0;
	if(L_Find == 70 && R_Find ==70&&!Ramp_Flag)
	{
		for(j=25; j<=135; j++)
		{
			if(Side_Thres_Count(image[30][j+3], image[30][j])>=ceshi_data[1])
				jump_point++;
		}
		for(j=30; j<=130; j++)
		{
			if(Side_Thres_Count(image[33][j+3], image[33][j])>=ceshi_data[1])jump_point++;
//			if(Side_Thres_Count(image[33][j+3], image[33][j])>=Side_Thres)
//				jump_point++;
		}
		if(jump_point>SAIDAO_DATA[2])
		{
		    A_Reach_End = 1;
			Beep=2;
		}
	}
}



//-------------------------------------------------------------------------------------------------------------------
//->赛道元素函数检测-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  @brief      直道
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       Miss,Road
//-------------------------------------------------------------------------------------------------------------------
void StraightRoad(void)
{
	//    if(Gearshift>0)
	//    {
	//        if(Rounding_L||Rounding_R||RampTime>40)
	//            Gearshift = 0;
	//    }
	Speed_Change(Gearshift);
}