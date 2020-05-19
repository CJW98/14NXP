#include "Kalman.h"
#include "matrix.h"

#define DELTA_T		0.003f
#define SELF_P		1.0f
#define EACH_P		0.00001f
#define INTERF_Q	0.005f	/*预测模型的外界干扰,
							越大越相信传感器的值,动态越好0.005
							越小越相信预测值,滤波效果越好
							过大不收敛,过小滞后性严重
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
//	0.0460, 0, 0, 0,对角标准差
//	0, 0.0511, 0, 0,
//	0, 0, 0.0240, 0,
//	0, 0, 0, 0.0317,
float R[4*4] = {
	0.0020,    0.0000,   -0.0000,    0.0001,
    0.0000,    0.0026,    0.0000,   -0.0001,
   -0.0000,    0.0000,    0.0006,   -0.0000,
    0.0001,   -0.0001,   -0.0000,    0.0009,
};
//float H[4*4] = {//观测量和传感器量无需转换,不使用H
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
//中间变量
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
	z[0] = accel[0];//转换为角度和角速度
	//z[1] = accel[1];
	z[1] = gyro[0];
	//z[3] = gyro[1];

	/*---卡尔曼滤波---
	-----由于传感器函数传入值和目标观测值度量相等,省略H矩阵的计算
	*/
	//更新为上次状态
	Matrix_Copy(x, 4, 1, x_1);// x_1 = x
	Matrix_Copy(P, 4, 4, P_1);// P_1 = P
	
	// x = F * x_1		预测的状态值
	Matrix_Multiply(F, 4, 4, x_1, 1, x);
	// P = F * P_1 * F' + Q		预测的不确定性
	Matrix_Multiply(F, 4, 4, P_1, 4, FP_1);//F*P_1
	Maxtrix_Transpose(F, 4, 4, F_T);//F'
	Matrix_Multiply(FP_1, 4, 4, F_T, 4, FPF_T);//F*P_1*F'
	Maxtrix_Add(FPF_T, Q, 4, 4 , P);//+Q
	
	//K = P * (P + R)'		更新卡尔曼系数
	Maxtrix_Add(P, R, 4, 4 , PaR);// PaR = P + R
	Maxtrix_Transpose(PaR, 4, 4, PaR_T);// PaR_T = PaR'
	Matrix_Multiply(P, 4, 4, PaR_T, 4, K);// K = P * PaR_T
	
	//x = x + K*(z-x)	最终状态为预测状态和测量状态的加权和
	Maxtrix_Sub(z, x, 4, 1, z_x);// z_x = z - x
	Matrix_Multiply(K, 4, 4, z_x, 1, Kz_x);// Kz_x = K * (z-x)
	Maxtrix_Add(x, Kz_x, 4, 1 , x);// x = x + Kz_x
	//P = P - KP	最终不确定性的变化
	Matrix_Multiply(K, 4, 4, P, 4, KP);// KP = K * P
	Maxtrix_Sub(P, KP, 4, 4, P);// P = P - KP

	//存储本次状态
	euler[0] = x[0];
	//euler[1] = x[1];

}

