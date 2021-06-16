#include "pid.h"
// 2.����ʽPID������
int Error_Last1,Error_Prev1;          // ��һ��ƫ��ֵ�����ϴ����
int Error_Last2,Error_Prev2;          // ��һ��ƫ��ֵ�����ϴ����
int Pwm_add1,Pwm,Pwm_last1;                    // PWM������PWM���ռ�ձ�
int Pwm_add2,Pwm,Pwm_last2;                    // PWM������PWM���ռ�ձ�
float Kp = 0.13	, Ki = 0.1, Kd = 0.1;       // PIDϵ��������ֻ�õ�PI������

int SpeedInnerControl1(int Target,int Speed) // �ٶ��ڻ�����
{
    int Error = Speed - Target;		  // ƫ�� = Ŀ���ٶ� - ʵ���ٶ�
//	if(-20.0 < Error < 20.0){
//		Pwm = Pwm_last;
//		return Pwm;
//	}
	if(Speed == 0){
		Error_Last1=0,Error_Prev1=0;
	}	
    Pwm_add1 = Kp * (Error - Error_Last1) + 					  // ����
			  Ki * Error +							    	  // ����
			  Kd * (Error - 2.0f * Error_Last1 + Error_Prev1)	  // ΢��
              ;  // ��һ��Ŀ�����������ź�Ϊ0ʱ��ϵͳ������ʧ��״̬
	
    Pwm = Pwm_add1+Pwm_last1;		              // ԭʼ��+���� = �����
	Pwm_last1 = Pwm;					  //�����ϴ�PWMֵ
	Error_Prev1 = Error_Last1;	  	  // �������ϴ����
    Error_Last1 = Error;	              // �����ϴ�ƫ��

    if(Pwm >= 1000) Pwm = 1000;	      // �޷�
    if(Pwm <=-1000) Pwm = -1000;
    return Pwm;	                      // �������ֵ
} 
int SpeedInnerControl2(int Target,int Speed) // �ٶ��ڻ�����
{
    int Error = Speed - Target;		  // ƫ�� = Ŀ���ٶ� - ʵ���ٶ�
//	if(-20.0 < Error < 20.0){
//		Pwm = Pwm_last;
//		return Pwm;
//	}
	if(Speed == 0){
		Error_Last2=0,Error_Prev2=0;
	}	
    Pwm_add2 = Kp * (Error - Error_Last2) + 					  // ����
			  Ki * Error +							    	  // ����
			  Kd * (Error - 2.0f * Error_Last2 + Error_Prev2)	  // ΢��
              ;  // ��һ��Ŀ�����������ź�Ϊ0ʱ��ϵͳ������ʧ��״̬
	
    Pwm = Pwm_add2+Pwm_last2;		              // ԭʼ��+���� = �����
	Pwm_last2 = Pwm;					  //�����ϴ�PWMֵ
	Error_Prev2 = Error_Last2;	  	  // �������ϴ����
    Error_Last2 = Error;	              // �����ϴ�ƫ��

    if(Pwm >= 1000) Pwm = 1000;	      // �޷�
    if(Pwm <=-1000) Pwm = -1000;
    return Pwm;	                      // �������ֵ
} 

//pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
typedef struct PID
{
float kp;
float ki;
float kd;
float ek;     //��ǰ���
float ek_1;   //��һ�����
float ek_sum; //����ܺ�
float limit;  //�޷�
}PID; 
static PID pid;
void PID_Init()
{
    pid.kp = 0.1;
    pid.ki = 0.2;
    pid.kd = 0.3;
    pid.limit = 800;
    pid.ek = 0;
    pid.ek_1 = 0;
    pid.ek_sum = 0;
}
// λ��ʽPID����
float PID_Postion(int Infrared,int Target)
{
	float pwm = 0;
		pid.ek = Target - Infrared; // ���㵱ǰ���
		pid.ek_sum += pid.ek;      //���ƫ��Ļ���
		pwm = pid.kp*pid.ek + pid.ki*pid.ek_sum + 
		pid.kd*(pid.ek - pid.ek_1);   //λ��ʽPID������
		pid.ek_1 = pid.ek;   //������һ��ƫ�� 
	if(pid.limit >= 1000) pid.limit = 1000;	      // �޷�
    else if(pid.limit <=-1000) pid.limit = -1000;
	if(pwm > pid.limit)
		{
		  pwm =  pid.limit;
		}
	else if(pwm < -pid.limit)
		{
		  pwm =  -pid.limit;
		}
	
return pwm;
}