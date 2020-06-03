#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

#define    PWM_VALUE           800
#define    TMR_T               2099
#define    SYSCLOCK            168000000
#include "stm32f4xx.h"

void DWT_Init(void)
{
        //разрешаем использовать счётчик
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         //обнуляем значение счётного регистра
	DWT_CYCCNT  = 0;
         //запускаем счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;
}

static __inline uint32_t delta(uint32_t t0, uint32_t t1)
{
    return (t1 - t0);
}

void delay_us(uint32_t us)
{
      uint32_t t0 =  DWT->CYCCNT;
      uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
      while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

void delay_ms(uint32_t us)
{
      uint32_t t0 =  DWT->CYCCNT;
      uint32_t us_count_tic =  us * (SystemCoreClock/1000);
      while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

int FRQ_CALCULATOR(int frequency)
{
	//int SysClockFreq = SystemCoreClockUpdate();
	int calFreq = SYSCLOCK*0.5/((TIM1->PSC+1)*frequency);

	return calFreq;

}

void TIM_EN()
{
	TIM1->CCMR1=TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
	TIM1->CCER=TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM1->BDTR=TIM_BDTR_MOE;
	//TIM1->CCR1=TMR_T - PWM_VALUE;
	//TIM1->CCR2=PWM_VALUE;
	TIM1->ARR=TMR_T;
	TIM1->CR1=TIM_CR1_ARPE | TIM_CR1_CMS_1 | TIM_CR1_CMS_0;
	TIM1->CR1|=TIM_CR1_CEN;
	TIM1->EGR=TIM_EGR_UG;
}

void SET_PWM(int pwm)
{
	TIM1->CCR1=TMR_T - pwm;
	TIM1->CCR2=pwm;
}

void SET_FRQ(int frq, int pwm)
{
	//CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
	int frqVal = FRQ_CALCULATOR(frq);
	int pwmVal = frqVal*pwm*0.01;
	TIM1->CCR1=frqVal - pwmVal;
	TIM1->CCR2=pwmVal;
	TIM1->ARR=frqVal;
	//TIM1->CCER=TIM_CCER_CC1E | TIM_CCER_CC2E;
}
