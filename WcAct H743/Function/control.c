#include "control.h"
#include "tim.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"
int MotorSpeed1;  // 全局变量，LM电机当前速度数值，从编码器中获取
int MotorSpeed2;  // 全局变量，RM电机当前速度数值，从编码器中获取
int SpeedTarget1 = 200;	  // 全局变量，LM速度目标值
int SpeedTarget2 = 200;	  // 全局变量，RM速度目标值
int MotorOutput1;		  // 全局变量，LM电机输出
int MotorOutput2;		  // 全局变量，RM电机输出

// 1.通过TIM3读取电机脉冲并计算速度
void GetMotorPulse(void)
{
  // 计数器获得电机脉冲，该电机在10ms采样的脉冲/编码器转速则为实际转速的rpm
  MotorSpeed1 = (short)(__HAL_TIM_GET_COUNTER(&htim3)); 
  MotorSpeed2 = (short)(__HAL_TIM_GET_COUNTER(&htim4)); 

  if(MotorSpeed1 < 0 )
  {
	    MotorSpeed1 +=26549;
	    MotorSpeed1 = (abs(MotorSpeed1)*180)/39;
	    MotorSpeed1 = -MotorSpeed1;
  }
  else{MotorSpeed1 = (abs(MotorSpeed1)*180)/39;}
   MotorSpeed1 *= 5;
  //MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim3));
  __HAL_TIM_SET_COUNTER(&htim3,0);  // 计数器清零
  
   if(MotorSpeed2 < 0 )
  {
	    MotorSpeed2 +=26549;
	    MotorSpeed2= (abs(MotorSpeed2)*180)/39;
	    MotorSpeed2 = -MotorSpeed2;
  }
  else{MotorSpeed2 = (abs(MotorSpeed2)*180)/39;}
   MotorSpeed2 *= 5;
  //MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim3));
  __HAL_TIM_SET_COUNTER(&htim4,0);  // 计数器清零

}

