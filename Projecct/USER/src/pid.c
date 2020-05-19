#include "pid.h"


float CONTROL[4]={1, 0, 30, 1000};
float CONTROL_N[4]={1, 0, 6, 1000};

float Direct_In[4] = {0.1, 0, 1, 1000};
float Direct_Out[4]={0, 0, 0, 1000};
float Direct_In_N[4] = {1, 0, 1, 1000};
float Obstacle_Direct[4] = {0.02,0,0.03,1000};
float Obstacle_Direct_N[4] = {20,0,1,1000};
float Angle_Speed_N[4] ={0.01, 0, 0.01, 1000};
PID Direct_In_PID,Direct_Out_PID,Direct_In_N_PID,OBSTACLE_DIRECT_PID,CONTROL_PID,CONTROL_N_PID,Obstacle_Direct_N_PID,Obstacle_Direct_PID,Angle_Speed_N_PID;	//�������͵����PID�����ṹ��


// PID������ʼ��
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}


void Motor_PID_Init(void)
{
    Motor_L.NowError  = 0;
    Motor_L.LastError = 0;	//Error[-1]
    Motor_L.PrevError = 0;	//Error[-2]
    
    
    Motor_R.NowError  = 0;
    Motor_R.LastError = 0;	//Error[-1]
    Motor_R.PrevError = 0;	//Error[-2]
    

    
}

/***λ��ʽ****/
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	// ��ǰ���
		 Realize;	// ���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���
	sptr->SumError += PID[KI] * iError;	// ������
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����

	return Realize;	// ����ʵ��ֵ
}

/***����ʽPID****/
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError,	//��ǰ���
		Increase;	//���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���

	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����
	
	return Increase;	// ��������
}


int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
{
	//����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	float iError;	//��ǰ���
	int  Actual;	//���ó���ʵ�����ֵ
	float Kp;		//��̬P
	
	iError = SetPoint - NowPiont;	//���㵱ǰ���
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <= PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
	//���κ�����Ϊ�˴ﵽ ���Խ��  ��ӦԽ�� �ظ���Խ�� ���� KIֵ�����Ϊ0ʱ��P Ҳ����ֱ���ϵ�Pֵ
	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//Pֵ���ֵ�ɶ��κ�����ϵ���˴�P��I����PID���������Ƕ�̬PID������Ҫע�⣡����
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//����ȫ΢��  
	sprt->LastError = iError;		//�����ϴ����

	Actual = range_protect(Actual, -260, 260);

	return Actual;
}


Motor Motor_L,Motor_R;
float fSpeedErrorInteg[2] = 0;
float g_fSpeedControlOut_R=0,g_fSpeedControlOut_L=0;
float g_fSpeedErrorTemp[2][5] = {0};
uint16 Speed_Set=0;
void Speed_Control()
{
	int8 index[2]={1,1};
	
	Motor_R.Kp=PID_Motor_R[0];
	Motor_L.Kp=PID_Motor_L[0];
	Motor_R.Ki=PID_Motor_R[1];
	Motor_L.Ki=PID_Motor_L[1];
	
	
	
	
	Motor_L.NowError = Speed[0] - Measure_Speed_L;
	Motor_R.NowError = Speed[1] - Measure_Speed_R;
	

    
    
//    if(Motor_R.NowError>=Motor_R.LastError)   
//	Motor_R.NowError = ((Motor_R.NowError-Motor_R.LastError)>2?(Motor_R.LastError+2):Motor_R.LastError); // ���ٶ�ƫ���޷�
//    else  
//	Motor_R.NowError = ((Motor_R.NowError-Motor_R.LastError)<-2?(Motor_R.LastError-2):Motor_R.LastError); 
//        
//    if(Motor_L.NowError>=Motor_L.LastError)   
//	Motor_L.NowError = ((Motor_L.NowError-Motor_L.LastError)>2?(Motor_L.LastError+2):Motor_L.LastError); // ���ٶ�ƫ���޷�
//    else  
//	Motor_L.NowError = ((Motor_L.NowError-Motor_L.LastError)<-2?(Motor_L.LastError-2):Motor_L.LastError); 
//    
//    
//  /**************����**********/
//     
// 
//    if((Motor_R.NowError<=15)&&(Motor_R.NowError>=-15))
//	index[0]=1;
//    else
//	index[0]=0;
//    fSpeedErrorInteg[0] = index[0]  * Motor_R.NowError * 1;
//    g_fSpeedControlOut_R +=10*(Motor_R.NowError-Motor_R.LastError) + fSpeedErrorInteg[0];
//    g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-800,800); 
//    Motor_R.LastError = Motor_R.NowError;//��¼��һ�ε�ƫ��e[n-1]
//// 
////	
//	if((Motor_L.NowError<=15)&&(Motor_L.NowError>=-15))
//		index[1]=1;
//	else
//		index[1]=0;
//	fSpeedErrorInteg[1] = index[0]  * Motor_L.NowError * 1;
//	g_fSpeedControlOut_L +=10*(Motor_L.NowError-Motor_L.LastError) + fSpeedErrorInteg[1];
//	g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-800,800); 
//	Motor_L.LastError = Motor_L.NowError;//��¼��һ�ε�ƫ��e[n-1]
	
    
//    if(A_Reach_End==1)
//	{
//
//    g_fSpeedControlOut_L += (5 + 5 ) * Motor_L.NowError	- (5  ) * Motor_L.LastError;//��������	
//    Motor_L.PrevError = Motor_L.LastError;//��¼����һ�ε�ƫ��e[n-2]
//    Motor_L.LastError = Motor_L.NowError;//��¼��һ�ε�ƫ��e[n-1]
//    g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-950,950); 
//    
//
// /**********����*******/   
//    
//
//    
//		
//	g_fSpeedControlOut_R += (5+ 5  ) * Motor_R.NowError	- (5 ) * Motor_R.LastError;//��������	
//	Motor_R.PrevError = Motor_R.LastError;//��¼����һ�ε�ƫ��e[n-2]
//	Motor_R.LastError = Motor_R.NowError;//��¼��һ�ε�ƫ��e[n-1]
//	g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-950,950);
//	}
//else
//{
    g_fSpeedControlOut_L += (Motor_L.Kp + Motor_L.Ki  + 0.1) * Motor_L.NowError	- (Motor_L.Kp  + 2 * 0.1) * Motor_L.LastError+ 0.1 * Motor_L.PrevError;//��������	
    Motor_L.PrevError = Motor_L.LastError;//��¼����һ�ε�ƫ��e[n-2]
    Motor_L.LastError = Motor_L.NowError;//��¼��һ�ε�ƫ��e[n-1]
    g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-980,980); 
    

 /**********����*******/   
    

    
		
	g_fSpeedControlOut_R += (Motor_R.Kp + Motor_R.Ki  + 0.1) * Motor_R.NowError	- (Motor_R.Kp  + 2 * 0.1) * Motor_R.LastError+ 0.1* Motor_R.PrevError;//��������	
	Motor_R.PrevError = Motor_R.LastError;//��¼����һ�ε�ƫ��e[n-2]
	Motor_R.LastError = Motor_R.NowError;//��¼��һ�ε�ƫ��e[n-1]
	g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-980,980);
    
//}   
//        

	

    
}





