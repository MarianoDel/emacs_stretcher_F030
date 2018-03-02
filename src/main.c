/**
  ******************************************************************************
  * @file    Template_2/main.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Use this template for new projects with stm32f0xx family.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "tim.h"
#include "uart.h"
#include "hard.h"

#include "signals.h"

#include "core_cm0.h"
#include "adc.h"
#include "flash_program.h"

#include "stm32f0xx_it.h"



//--- VARIABLES EXTERNAS ---//


// ------- Externals del Puerto serie  -------
volatile unsigned char tx1buff[SIZEOF_DATA];
volatile unsigned char rx1buff[SIZEOF_DATA];
volatile unsigned char usart1_have_data = 0;

// ------- Externals del o para el ADC -------
#ifdef ADC_WITH_INT
volatile unsigned short adc_ch[3];
volatile unsigned char seq_ready = 0;
#endif

// ------- Externals para filtros -------
unsigned short mains_voltage_filtered;
//
//
// volatile unsigned short scroll1_timer = 0;
// volatile unsigned short scroll2_timer = 0;
//
// volatile unsigned short standalone_timer;
// volatile unsigned short standalone_enable_menu_timer;
// //volatile unsigned short standalone_menu_timer;
// volatile unsigned char grouped_master_timeout_timer;
volatile unsigned short take_temp_sample = 0;


// parameters_typedef param_struct;

//--- VARIABLES GLOBALES ---//
volatile unsigned char current_excess = 0;






// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
//volatile unsigned char display_timer;
volatile unsigned char timer_meas;

volatile unsigned char door_filter;
volatile unsigned char take_sample;
volatile unsigned char move_relay;


volatile unsigned short secs = 0;
volatile unsigned char hours = 0;
volatile unsigned char minutes = 0;




//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);

// ------- para el LM311 -------
extern void EXTI0_1_IRQHandler(void);




//--- Private Definitions ---//


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
	unsigned char i, ii;

	main_state_t main_state = MAIN_INIT;

	char s_lcd [100];		//lo agrando porque lo uso tambien para enviar SMS

	//GPIO Configuration.
	GPIO_Config();

	//ACTIVAR SYSTICK TIMER
	if (SysTick_Config(48000))
	{
		while (1)	/* Capture error */
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			for (i = 0; i < 255; i++)
			{
				asm (	"nop \n\t"
						"nop \n\t"
						"nop \n\t" );
			}
		}
	}

	//--- Prueba LED y JUMPER ---//
	// while (1)
	// {
	// 	if (STOP_JUMPER)
	// 	{
	// 		LED_OFF;
	// 	}
	// 	else
	// 	{
	// 		if (LED)
	// 			LED_OFF;
	// 		else
	// 		  	LED_ON;
	//
	// 		Wait_ms (250);
	// 	}
	// }
	//--- Fin Prueba LED y JUMPER ---//



	//--- Welcome code ---//
	LED_OFF;

	USART1Config();

	//--- Prueba LED y USART TX ---//
	// while (1)
	// {
	// 	Usart1Send((char *) (const char *) "ADC Sync getted!\r\n");
	// 	if (LED)
	// 		LED_OFF;
	// 	else
	// 		LED_ON;
	//
	// 	Wait_ms (1000);
	// }
	//--- Fin Prueba LED y USART TX---//

	//--- Prueba LED y USART RX ---//
	// while (1)
	// {
	// 	if (usart1_have_data)
	// 	{
	// 		LED_ON;
	// 		usart1_have_data = 0;
	// 		ReadUsart1Buffer (s_lcd, SIZEOF_DATA);
	// 		Usart1Send(s_lcd);
	// 		LED_OFF;
	// 	}
	// }
	//--- Fin Prueba LED y USART RX ---//

//---------- Pruebas de Hardware --------//
	AdcConfig();		//recordar habilitar sensor en adc.h

	TIM_1_Init ();					//lo utilizo para synchro ADC muestras 1500Hz
	TIM_3_Init ();					//lo utilizo para mosfets LOW_LEFT, HIGH_LEFT, LOW_RIGHT, HIGH_RIGHT

	//Update_TIM3_CH2 (10);
	// TIM3->CCR3 = 1000;
	// TIM3->ARR = 6858;

	// while (1);

	//--- Prueba Pines PWM ---//
	// EXTIOff ();
	// while (1)
	// {
	// 	for (d = 0; d < DUTY_100_PERCENT; d++)
	// 	{
	// 		Update_TIM3_CH1 (d);
	// 		Update_TIM3_CH2 (d);
	// 		Update_TIM3_CH3 (d);
	// 		Update_TIM3_CH4 (d);
	//
	// 		Wait_ms(2);
	// 	}
	// }
	//--- Fin Prueba Pines PWM ---//

	//--- Prueba de seniales PWM ---//
	//CUADRADA baja
	// LOW_RIGHT_PWM(DUTY_100_PERCENT+1);
	// HIGH_RIGHT_PWM(0);
	//
	// while (1)
	// {
	// 	HIGH_LEFT_PWM(0);
	// 	LOW_LEFT_PWM(DUTY_50_PERCENT);
	//
	// 	Wait_ms(20);
	//
	// 	LOW_LEFT_PWM(0);
	//
	// 	Wait_ms(20);
	// }

	// //CUADRADA alta derecha
	// while (1)
	// {
	// 	HIGH_LEFT_PWM(0);
	// 	LOW_LEFT_PWM(DUTY_100_PERCENT+1);
	//
	// 	LOW_RIGHT_PWM(0);
	// 	HIGH_RIGHT_PWM(DUTY_50_PERCENT);
	//
	// 	Wait_ms(20);
	//
	// 	HIGH_RIGHT_PWM(0);
	// 	Wait_ms(20);
	// }

	//CUADRADA alta izquierda
	//--- Prueba ADC y synchro ---//
	ADC1->CR |= ADC_CR_ADSTART;	//1500Hz con TIM1
	seq_ready = 0;




	//--- Pruebas lazo PID
	//-- primero preparo el puente H segun la funcion que busque
	LOW_LEFT_PWM(0);
	HIGH_RIGHT_PWM(0);
	LOW_RIGHT_PWM(DUTY_100_PERCENT+1);

	HIGH_LEFT_PWM(0);	//este es el que actua


	//prueba de nuevas rutinas
	SetSignalType (SQUARE_SIGNAL);
	SetFrequency (THIRTY_HZ);
	SetPower (20);
	// void GenerateSignal (void);


	while (1)
	{
		GenerateSignal();
	}


	//--- Prueba ADC y synchro ---//
	// ADC1->CR |= ADC_CR_ADSTART;
	// seq_ready = 0;
	// seq_index = 0;
	//
	// while (1)
	// {
	// 	if (seq_ready)
	// 	{
	// 		seq_ready = 0;
	// 		seq_index++;
	// 	}
	//
	// 	if (seq_index > 10000)
	// 	{
	// 		seq_index = 0;
	// 		sprintf(s_lcd, "ch0: %4d ch1: %4d\r\n", Input_Signal, I_Sense);
	// 		Usart1Send(s_lcd);
	// 		if (LED)
	// 			LED_OFF;
	// 		else
	// 		  	LED_ON;
	// 	}
	// }
	//--- Fin Prueba ADC y synchro ---//




//---------- Inicio Programa de Produccion Redonda Basic --------//
//--- FIN Programa de pruebas synchro de Relay -----
//--- Programa de Redonda Basic - Produccion - -----

//---------- Fin Programa de Produccion Redonda Basic--------//


	return 0;
}

//--- End of Main ---//





void TimingDelay_Decrement(void)
{
	if (wait_ms_var)
		wait_ms_var--;

	if (timer_standby)
		timer_standby--;

	if (take_temp_sample)
		take_temp_sample--;

	if (timer_meas)
		timer_meas--;

	// //cuenta de a 1 minuto
	// if (secs > 59999)	//pasaron 1 min
	// {
	// 	minutes++;
	// 	secs = 0;
	// }
	// else
	// 	secs++;
	//
	// if (minutes > 60)
	// {
	// 	hours++;
	// 	minutes = 0;
	// }


}

void EXTI0_1_IRQHandler(void)
{

	if(EXTI->PR & 0x00000001)	//Line0
	{
		LED_ON;

		// if (CURRENT_LOOP)		//intenta corregir flux inbalance como lazo de corriente
		// {
		// 	//hubo interrupcion, me fijo que pwm estaba generando
		// 	if (TIM1->CNT < TIM1->CCR1)
		// 	{
		// 		//generaba TIM1
		//
		// 	}
		//
		// }
		UpdateTIMSync(0);


		current_excess = 1;


		EXTI->PR |= 0x00000001;
	}
}


//------ EOF -------//
