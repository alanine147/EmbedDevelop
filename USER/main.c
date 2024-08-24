#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"

/************************************************
 ALIENTEK ̽����STM32F407������ʵ��0-1
 Template����ģ��-�½������½�ʹ��-HAL��汾
 ����֧�֣�www.openedv.com
 �Ա����̣� http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 �������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


/***ע�⣺�����̺ͽ̳��е��½�����3.3С�ڶ�Ӧ***/


void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int main(void)
{

	GPIO_InitTypeDef GPIO_Initure;
     
    HAL_Init();                    	 			//��ʼ��HAL��    
    Stm32_Clock_Init(336,8,2,7);   				//����ʱ��,168Mhz

    __HAL_RCC_GPIOF_CLK_ENABLE();           	//����GPIOFʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10; 	//PF9,10
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������
    GPIO_Initure.Pull=GPIO_PULLUP;          	//����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//����
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);

	while(1)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_SET);		//PF9��1 
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);		//PF10��1  			
		Delay(0x7FFFFF);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);		//PF9��0
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);	//PF10��0  
		Delay(0x7FFFFF);
	}
}
