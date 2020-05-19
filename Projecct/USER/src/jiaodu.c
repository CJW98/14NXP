#include "headfile.h"
#include "jiaodu.h"


#define GRAVITY_ADJUST_TIME_CONSTANT  1   
#define DT     0.005
int16  Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z;

float m;

float Angle1_Y,Angle1_Z,Angle_Speed_Y,Angle_Speed_Z,Angle_Speed,Car_Angle=0;
//int16  Gyro_X_Offset=-40,Gyro_Y_Offset=1;


float angle_offset_vertical,Gyro_Now,g_fCarAngle,g_fGyroscopeAngleIntegral;
int16 Attitude_Angle[2];

float Angle[2],AngleSpeed[2];
void jiaodu(void)
{   
	float G_offset=0;
	float fDeltaValue;
	float EMAoffset=0.005;
	Acc_X = Get_X_Acc();
	Acc_Y = Get_Y_Acc();
	Acc_Z = Get_Z_Acc(); 
	Gyro_X= Get_X_Gyro();
	Gyro_Y= Get_Y_Gyro();   
	Gyro_Z= Get_Z_Gyro();
	
	float ratio=0.03;//0.048
	
	/*****ƫ����yaw*******/
	//    G_offset = EMAoffset *(Gyro_X+230)*GYRO_SCALE + (1-EMAoffset) * G_offset; 
	//    Gyro_X=(Gyro_X+230)*GYRO_SCALE-G_offset;  //X����ٶȹ�һ��
	Gyro_X=(Gyro_X+257)*GYRO_SCALE;
	Turn_Angle+=Gyro_X*0.003;
	
	
	/***�������Ƕ�PITCH*****/
	//    Angle1_Y = -RADTODEG(FastAtan2(Acc_Z*ACCEL_SCALE,FastSqrt(Acc_X*Acc_X+Acc_Y*Acc_Y)*ACCEL_SCALE));//�ȹ�һ�������ټ���ǰ��λ��
	//    Angle_Speed_Y=(Gyro_Y+257) *GYRO_SCALE;
	//     
	//    Kalman_Filter(Angle1_Y,Angle_Speed_Y);            //���ÿ������˲�����     
	
	
	
	
	AttitudeCalculation(Angle,AngleSpeed);
	Angle[0]=-Angle[0];
	
	/***�����˲�****/
	//    angle_offset_vertical=RADTODEG(FastAtan2(Acc_Z,FastSqrt(Acc_X*Acc_X+Acc_Y*Acc_Y)));//���ٶȽǶ�
	//    Gyro_Now=(float)(-(Get_Y_Gyro()+257)*GYRO_SCALE);//�����ǲɼ����Ľ��ٶȹ�һ��  
	//    if(angle_offset_vertical > 90)
	//	angle_offset_vertical = 90;               //��ֹ���ٶȽǶ����
	//    if(angle_offset_vertical < -90)
	//	angle_offset_vertical = -90;
	//    
	//    g_fCarAngle = g_fGyroscopeAngleIntegral;   //�����ںϽǶ�
	//    fDeltaValue = (angle_offset_vertical - g_fCarAngle)/(1);  //ʱ��ϵ������
	//    g_fGyroscopeAngleIntegral += (Gyro_Now + fDeltaValue) * 0.004;                //�ںϽǶ�
	
	
	
	
	
	
}



void AttitudeCalculation(float *euler,float *gyro)
{
	float accel[3];		
	accel[0] = -RADTODEG(FastAtan2(Acc_Z*ACCEL_SCALE2,FastSqrt(Acc_X*Acc_X+Acc_Y*Acc_Y)*ACCEL_SCALE2));//ǰ��
	//accel[0] = -RADTODEG(FastAtan2(Acc_Y,FastSqrt(Acc_X*Acc_X+Acc_Z*Acc_Z)));//����	//���ٶ���ֵ��������Ƕ�,ʸ����Ϊ1
	m=accel[0];
	gyro[0]  =(float)((Get_Y_Gyro()+257)*GYRO_SCALE);
	KalmanFilter(accel,gyro,euler);//��������ŷ����,a/gǰ��������
}





//******Kalman�˲�******//
////-------------------------------------------------------
static  float Q_angle=0.002, Q_gyro=0.001, R_angle=1, dt=0.004;   //0.001  0.001  5  0.004
//Q���󣬶�̬��Ӧ���� Q_angle�Ƕ�����  Q_gyroƯ������
static float Pk[2][2] = { {1, 0}, {0, 1 }};

static float Pdot[4] ={0,0,0,0};

static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
////-------------------------------------------------------


void Kalman_Filter(float angle_m,float gyro_m)	
{
	Car_Angle+=(gyro_m-q_bias) * dt; ///Ԥ��ֵ
	Pdot[0]=Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1]=- Pk[1][1];
	Pdot[2]=- Pk[1][1];
	Pdot[3]=Q_gyro;
	
	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;
	
	angle_err = angle_m -Car_Angle;///����ֵ-Ԥ��ֵ
	
	PCt_0 =  Pk[0][0];
	PCt_1 =  Pk[1][0];
	
	E = R_angle + PCt_0;
	
	K_0 = PCt_0 / E; ///����������
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = Pk[0][1];
	
	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;
	
	Car_Angle+= K_0 * angle_err; ///���ŽǶ�=Ԥ��ֵ+����������*(����ֵ-Ԥ��ֵ)
	//  Car_Angle= -Car_Angle;
	q_bias	+= K_1 * angle_err;
	Angle_Speed = gyro_m-q_bias;
}




#define Kp1 1.6f //10.0f             	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki1 0.001f//1.2f // //0.008f  	// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.002f                   	// half the sample period�������ڵ�һ��
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; 	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; 	// scaled integral error
/*
* ��������IMUupdate
* ����  ����Ԫ�ؽ���ŷ����
* ����  �������� ���ٶȼ�
* ���  ����
* ����  ���ڲ�����
*/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	// Site_t site;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
	// �Ȱ���Щ�õõ���ֵ���
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if (ax*ay*az == 0)
	{
		return;
	}
	
	norm = sqrt(ax*ax + ay*ay + az*az);	// acc���ݹ�һ��
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	
	// estimated direction of gravity and flux (v and w)	�����������������/��Ǩ
	vx = 2*(q1q3 - q0q2);									// ��Ԫ����xyz�ı�ʾ
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;		// �������������õ���־������
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	exInt = exInt + ex * Ki1;	// �������л���
	eyInt = eyInt + ey * Ki1;
	ezInt = ezInt + ez * Ki1;
	
	// adjusted gyroscope measurements
	gx = gx + Kp1*ex + exInt;	// �����PI�󲹳��������ǣ����������Ư��
	gy = gy + Kp1*ey + eyInt;
	gz = gz + Kp1*ez + ezInt;	// �����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
	
	// integrate quaternion rate and normalise	// ��Ԫ�ص�΢�ַ���
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	Attitude_Angle[0] = asin(-2*q1*q3 + 2*q0*q2) * 57.3; // pitch
	//	Attitude_Angle.X = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3; // roll
	Attitude_Angle[1] = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)*57.3; // yaw
	//	Attitude_Angle.Z = 0;
}

