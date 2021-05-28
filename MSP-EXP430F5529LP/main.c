#include <driverlib.h>
#include "msp430f5529.h"
#define COMPARE_VALUE 32768

//Target frequency for MCLK in kHz
#define UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ   24000
#define UCS_MCLK_FLLREF_RATIO   6  //PLL

//XT2 Crystal Frequency being used
#define UCS_XT2_CRYSTAL_FREQUENCY   4000000

#define UCS_XT1_TIMEOUT 50000
#define TIMER_PERIOD 3999
#define DUTY_CYCLE1  3000
#define DUTY_CYCLE2  2000
#define UCS_XT2_TIMEOUT 50000
uint32_t clockValue = 0;

uint16_t status;
uint8_t returnValue = 0;

void TIM_Init(void){
    //Start timer
    Timer_A_initUpModeParam initUpParam = {0};
    initUpParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initUpParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpParam.timerPeriod = TIMER_PERIOD;
    initUpParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initUpParam.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    initUpParam.timerClear = TIMER_A_DO_CLEAR;
    initUpParam.startTimer = false;
    Timer_A_initUpMode(TIMER_A1_BASE, &initUpParam);
    //Timer_A_initUpMode(TIMER_A2_BASE, &initUpParam);
    Timer_A_startCounter(TIMER_A1_BASE,
            TIMER_A_UP_MODE
            );
    Timer_A_initUpModeParam initUpParam2 = {0};
    initUpParam2.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initUpParam2.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpParam2.timerPeriod = TIMER_PERIOD;
    initUpParam2.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initUpParam2.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    initUpParam.timerClear = TIMER_A_DO_CLEAR;
    initUpParam.startTimer = false;

    Timer_A_initUpMode(TIMER_A2_BASE, &initUpParam);
    Timer_A_startCounter(TIMER_A2_BASE,
            TIMER_A_UP_MODE
            );


}
void PWM_Init(void){
    PMM_setVCore(PMM_CORE_LEVEL_1);//主频提高后，VCore电压也需要随之配置
    PMM_setVCore(PMM_CORE_LEVEL_2);//主频提高后，VCore电压也需要随之配置
    PMM_setVCore(PMM_CORE_LEVEL_3);//主频提高后，VCore电压也需要随之配置

    //Startup HF XT2 crystal Port select XT2
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN2 + GPIO_PIN3);//XT2口不作为普通IO

    //Initialize XT2. Returns STATUS_SUCCESS if initializes successfully
    returnValue = UCS_turnOnXT2WithTimeout(UCS_XT2_DRIVE_4MHZ_8MHZ,UCS_XT2_TIMEOUT);//启动XT2

    //Set DCO FLL reference = REFO
    UCS_initClockSignal(UCS_FLLREF,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);//XT2作为FLL参考

    UCS_initFLLSettle(UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ,UCS_MCLK_FLLREF_RATIO);//MCLK设置为24MHz
    UCS_initClockSignal(UCS_SMCLK,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);//SMCLK设置为4MHz

    //Initialize compare mode to generate PWM1
    Timer_A_initCompareModeParam initComp1Param = {0};
    initComp1Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    initComp1Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp1Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    initComp1Param.compareValue = DUTY_CYCLE1;
    Timer_A_initCompareMode(TIMER_A1_BASE, &initComp1Param);

    //Initialize compare mode to generate PWM2
    Timer_A_initCompareModeParam initComp2Param = {0};
    initComp2Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    initComp2Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp2Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    initComp2Param.compareValue = DUTY_CYCLE2;
    Timer_A_initCompareMode(TIMER_A2_BASE, &initComp2Param);

}
int main(void) {

    volatile uint32_t i;
      WDT_A_hold(WDT_A_BASE);
      TIM_Init();
      PWM_Init();
      GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
      GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
      GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN0);
      GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4);

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
