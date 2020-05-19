#include "Kalman.h"
#include "matrix.h"

#define DELTA_T		0.003f
#define SELF_P		1.0f
#define EACH_P		0.00001f
#define INTERF_Q	0.005f	/*Ԥ��ģ�͵�������,
							Խ��Խ���Ŵ�������ֵ,��̬Խ��0.005
							ԽСԽ����Ԥ��ֵ,�˲�Ч��Խ��
							��������,��С�ͺ�������
							*/
//#define NOISE_R		0.3f

float F[4*4] = {
	1, 0, DELTA_T, 0,
	0, 1, 0, DELTA_T,
	0, 0, 1, 0,
	0, 0, 0, 1,
};
float P[4*4] = {
	SELF_P, 0, EACH_P, 0,
	0, SELF_P, 0, EACH_P,
	EACH_P, 0, SELF_P, 0,
	0, EACH_P, 0, SELF_P,
};
float Q[4*4] = {
	INTERF_Q, 0, 0, 0,
	0, INTERF_Q, 0, 0,
	0, 0, INTERF_Q, 0,
	0, 0, 0, INTERF_Q,
};
//	0.0460, 0, 0, 0,�ԽǱ�׼��
//	0, 0.0511, 0, 0,
//	0, 0, 0.0240, 0,
//	0, 0, 0, 0.0317,
float R[4*4] = {
	0.0020,    0.0000,   -0.0000,    0.0001,
    0.0000,    0.0026,    0.0000,   -0.0001,
   -0.0000,    0.0000,    0.0006,   -0.0000,
    0.0001,   -0.0001,   -0.0000,    0.0009,
};
//float H[4*4] = {//�۲����ʹ�����������ת��,��ʹ��H
//	1, 0, 0, 0,
//	0, 1, 0, 0,
//	0, 0, 1, 0,
//	0, 0, 0, 1,
//};
float K[4*4]={
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
};
float z[2]={0};
float x[4]={0};
float x_1[4];
float P_1[4*4];
//�м����
float z_x[4];
float Kz_x[4];

float KP[4*4];
float PaR[4*4];
float PaR_T[4*4];

float FP_1[4*4];
float F_T[4*4];
float FPF_T[4*4];

void KalmanFilter(float *accel,float *gyro,float *euler)
{
	z[0] = accel[0];//ת��Ϊ�ǶȺͽ��ٶ�
	//z[1] = accel[1];
	z[1] = gyro[0];
	//z[3] = gyro[1];

	/*---�������˲�---
	-----���ڴ�������������ֵ��Ŀ��۲�ֵ�������,ʡ��H����ļ���
	*/
	//����Ϊ�ϴ�״̬
	Matrix_Copy(x, 4, 1, x_1);// x_1 = x
	Matrix_Copy(P, 4, 4, P_1);// P_1 = P
	
	// x = F * x_1		Ԥ���״ֵ̬
	Matrix_Multiply(F, 4, 4, x_1, 1, x);
	// P = F * P_1 * F' + Q		Ԥ��Ĳ�ȷ����
	Matrix_Multiply(F, 4, 4, P_1, 4, FP_1);//F*P_1
	Maxtrix_Transpose(F, 4, 4, F_T);//F'
	Matrix_Multiply(FP_1, 4, 4, F_T, 4, FPF_T);//F*P_1*F'
	Maxtrix_Add(FPF_T, Q, 4, 4 , P);//+Q
	
	//K = P * (P + R)'		���¿�����ϵ��
	Maxtrix_Add(P, R, 4, 4 , PaR);// PaR = P + R
	Maxtrix_Transpose(PaR, 4, 4, PaR_T);// PaR_T = PaR'
	Matrix_Multiply(P, 4, 4, PaR_T, 4, K);// K = P * PaR_T
	
	//x = x + K*(z-x)	����״̬ΪԤ��״̬�Ͳ���״̬�ļ�Ȩ��
	Maxtrix_Sub(z, x, 4, 1, z_x);// z_x = z - x
	Matrix_Multiply(K, 4, 4, z_x, 1, Kz_x);// Kz_x = K * (z-x)
	Maxtrix_Add(x, Kz_x, 4, 1 , x);// x = x + Kz_x
	//P = P - KP	���ղ�ȷ���Եı仯
	Matrix_Multiply(K, 4, 4, P, 4, KP);// KP = K * P
	Maxtrix_Sub(P, KP, 4, 4, P);// P = P - KP

	//�洢����״̬
	euler[0] = x[0];
	//euler[1] = x[1];

}

