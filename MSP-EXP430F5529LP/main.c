#include <driverlib.h>
#include "msp430f5529.h"
#define COMPARE_VALUE 32768

#define TIMER_PERIOD 1045
#define DUTY_CYCLE  350
uint32_t clockValue = 0;
void TIM_Init(void){
    //Start timer in continuous mode sourced by ACLK
        Timer_A_initContinuousModeParam initContParam = {0};
        initContParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
        initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
        initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
        initContParam.timerClear = TIMER_A_DO_CLEAR;
        initContParam.startTimer = false;
        Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

        //Initiaze compare mode
        Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Timer_A_initCompareModeParam initCompParam = {0};
        initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
        initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
        initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
        initCompParam.compareValue = COMPARE_VALUE;
        Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

        Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);
}
void PWM_Init(void){

    Timer_A_outputPWMParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = TIMER_PERIOD;
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = DUTY_CYCLE;
    Timer_A_outputPWM(TIMER_A1_BASE, &param);


}
int main(void) {

    volatile uint32_t i;
      WDT_A_hold(WDT_A_BASE);
      TIM_Init();
      PWM_Init();
      GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
      GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
      GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN0);

      //Enter LPM0, enable interrupts
      __bis_SR_register(LPM0_bits + GIE);

      //For debugger
      __no_operation();

      //Verify if the Clock settings are as expected
      clockValue = UCS_getMCLK();
      clockValue = UCS_getACLK();
      clockValue = UCS_getSMCLK();

    while(1)
    {
        // Toggle P1.0 output
        //GPIO_toggleOutputOnPin(GPIO_PORT_P4 , GPIO_PIN7);
       // GPIO_toggleOutputOnPin(GPIO_PORT_P1 , GPIO_PIN0);
        // Delay
        //for(i=20000; i>0; i--);
    }
}

//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR (void)
{
    uint16_t compVal = Timer_A_getCaptureCompareCount(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)+COMPARE_VALUE;
    //Toggle P1.0
    GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
    //Add Offset to CCR0
    Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0,compVal);
}
