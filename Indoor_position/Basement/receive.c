#include "receive.h"
/**
  * @brief  initialise the reseave
  * @param  None
  * @retval None
  */
void TimerCapture_Init(void)
{
	/* Configure paraments */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/*Open clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /*Configure GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	
  /*T2*/
	TIM_TimeBaseStructure.TIM_Period = 0xffff;	
	TIM_TimeBaseStructure.TIM_Prescaler=71;//
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	/*T2CH1*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/*T2CH2*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/*T2CH3*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/*T2CH4*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/*T3*/
	TIM_TimeBaseStructure.TIM_Period = 0xffff;	
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
//	/*T3C1*/
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
//	TIM_ICInit(TIM3, &TIM_ICInitStructure);
//	
//	/*T3C2*/
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
//	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	/*T3C3*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	
//	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
//	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	
	/*Distribute the prior*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**
  * @brief  The control data reseave IRQ in TIM_ICPolarity_BothEdge
  * @param  None
  * @retval None
  */
void Reseave_IrqTIM2(Chx_Width*p)
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	if(TIM2->SR & TIM_IT_CC1)//1515 2015 1015
	{
		u16 ccr1 = TIM2->CCR1;
		TIM2->SR &= ~TIM_FLAG_CC1OF;
		if(GPIOA->IDR & GPIO_Pin_0)//up
		{
			temp_cnt1 = ccr1;
			TIM2->CCER |= (1<<1);
		}
		else
		{
			temp_cnt1_2 = ccr1;
			TIM2->CCER &= ~(1<<1);
			if(temp_cnt1_2>=temp_cnt1)
				p->Ch1 = temp_cnt1_2-temp_cnt1;
			else
				p->Ch1  = 0xffff-temp_cnt1+temp_cnt1_2;
		}
	}
	
	if(TIM2->SR & TIM_IT_CC2)//
	{
		u16 ccr2 = TIM2->CCR2;
		if(GPIOA->IDR & GPIO_Pin_1)
		{
			temp_cnt2 = ccr2;
			TIM2->CCER |= (1<<5);
		}
		else
		{
			temp_cnt2_2 = ccr2;
			TIM2->CCER &= ~(1<<5);
			if(temp_cnt2_2>=temp_cnt2)
				p->Ch2 = temp_cnt2_2-temp_cnt2;
			else
				p->Ch2 = 0xffff-temp_cnt2+temp_cnt2_2;
		}
	}
	
	if(TIM2->SR & TIM_IT_CC3)
	{
		u16 ccr3 = TIM2->CCR3;
		if(GPIOA->IDR & GPIO_Pin_2)
		{
			temp_cnt3 = ccr3;
			TIM2->CCER |= (1<<9);
		}
		else
		{
			temp_cnt3_2 = ccr3;
			TIM2->CCER &= ~(1<<9);
			if(temp_cnt3_2>=temp_cnt3)
				p->Ch3=temp_cnt3_2-temp_cnt3;
			else
				p->Ch3=0xffff-temp_cnt3+temp_cnt3_2;
		}
	}
	
	if(TIM2->SR & TIM_IT_CC4)
	{
		u16 ccr4 = TIM2->CCR4;
		if(GPIOA->IDR & GPIO_Pin_3)
		{
			temp_cnt4 = ccr4;
			TIM2->CCER |= (1<<13);
		}
		else
		{
			temp_cnt4_2 = ccr4;
			TIM2->CCER &= ~(1<<13);
			if(temp_cnt4_2>=temp_cnt4)
				p->Ch4 = temp_cnt4_2-temp_cnt4;
			else
				p->Ch4 = 0xffff-temp_cnt4+temp_cnt4_2;
		}
	}
}

void Reseave_IrqTIM3(Chx_Width*p)
{
	static u16 temp_cnt3,temp_cnt3_2;
//	if(TIM3->SR & TIM_IT_CC1)//1515 2015 1015
//	{
//		u16 ccr1 = TIM3->CCR1;
//		TIM3->SR &= ~TIM_FLAG_CC1OF;
//		if(GPIOA->IDR & GPIO_Pin_6)//up
//		{
//			temp_cnt1 = ccr1;
//			TIM3->CCER |= (1<<1);
//		}
//		else
//		{
//			temp_cnt1_2 = ccr1;
//			TIM3->CCER &= ~(1<<1);
//			if(temp_cnt1_2>=temp_cnt1)
//				p->Ch5 = temp_cnt1_2-temp_cnt1;
//			else
//				p->Ch5  = 0xffff-temp_cnt1+temp_cnt1_2;
//		}
//	}
//	
//	if(TIM3->SR & TIM_IT_CC2)//
//	{
//		u16 ccr2 = TIM3->CCR2;
//		if(GPIOA->IDR & GPIO_Pin_7)
//		{
//			temp_cnt2 = ccr2;
//			TIM3->CCER |= (1<<5);
//		}
//		else
//		{
//			temp_cnt2_2 = ccr2;
//			TIM3->CCER &= ~(1<<5);
//			if(temp_cnt2_2>=temp_cnt2)
//				p->Ch6 = temp_cnt2_2-temp_cnt2;
//			else
//				p->Ch6 = 0xffff-temp_cnt2+temp_cnt2_2;
//		}
//	}
//	
	if(TIM3->SR & TIM_IT_CC3)
	{
		u16 ccr3 = TIM3->CCR3;
		if(GPIOB->IDR & GPIO_Pin_0)
		{
			temp_cnt3 = ccr3;
			TIM3->CCER |= (1<<9);
		}
		else
		{
			temp_cnt3_2 = ccr3;
			TIM3->CCER &= ~(1<<9);
			if(temp_cnt3_2>=temp_cnt3)
				p->Ch7=temp_cnt3_2-temp_cnt3;
			else
				p->Ch7=0xffff - temp_cnt3 + temp_cnt3_2;
		}
	}
}
