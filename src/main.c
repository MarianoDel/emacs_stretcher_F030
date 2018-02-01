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

// #define Iout_Sense	adc_ch[0]
// #define Vin_Sense		adc_ch[1]
// #define I_Sense		adc_ch[2]
// #define Vout_Sense	adc_ch[3]

volatile unsigned char seq_ready = 0;
unsigned short zero_current;

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


//--- FILTROS DE SENSORES ---//
#define LARGO_FILTRO 16
#define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
unsigned short vtemp [LARGO_FILTRO + 1];
unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//


//--- Private Definitions ---//


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
	unsigned char i, ii;

	unsigned char need_to_save = 0;
	unsigned short d = 0;
	unsigned int zero_current_loc = 0;

	main_state_t main_state = MAIN_INIT;


	unsigned short hyst;

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

	//--- Leo los parametros de memoria ---//

  // hile (1)
  // {
  //  if (STOP_JUMPER)
  //  {
  //  	LED_OFF;
  //  }
  //  else
  //  {
  // 	  if (LED)
  // 	  	LED_OFF;
  // 	  else
  // 	  	LED_ON;
  //
  // 	  Wait_ms (250);
  //  }
  // }



	//--- Welcome code ---//
	LED_OFF;

	USART1Config();


//---------- Pruebas de Hardware --------//
	AdcConfig();		//recordar habilitar sensor en adc.h

	TIM_1_Init ();					//lo utilizo para mosfet Ctrol_M_B,
	TIM_3_Init ();					//lo utilizo para mosfet Ctrol_M_A y para synchro ADC
	TIM_14_Init();					//Set current overflow

	// UpdateTIMSync (12);
	// Update_TIM3_CH1 (100);		//lo uso para ver diff entre synchro adc con led
	// Update_TIM14_CH1 (512);		//lo uso para ver diff entre synchro adc con led
	// Update_TIM1_CH1 (100);		//lo uso para ver diff entre synchro adc con led

	// while (1)
	// {
	// 	// ISENSE_OFF;
	// 	// Wait_ms(100);
	// 	// ISENSE_ON;
	// 	// Wait_ms(2);
	// }

	// while (1)
	// {
	// 	if (usart1_have_data)
	// 	{
	// 		usart1_have_data = 0;
	// 		ReadUsart1Buffer (s_lcd, SIZEOF_DATA);
	// 		Usart1Send(s_lcd);
	// 	}
	// 	// Wait_ms(1000);
	//
	// 	// Usart1Send((char *) (const char *) "  Features:\r\n");
	// 	// Wait_ms(5000);
	// }

// Usart1Send((char *) (const char *) "\r\nKirno Placa Redonda - Basic V1.0\r\n");
// Usart1Send((char *) (const char *) "  Features:\r\n");
// #ifdef WITH_1_TO_10_VOLTS
// Usart1Send((char *) (const char *) "  Dimmer 1 to 10V\r\n");
// #endif
// #ifdef WITH_HYST
// Usart1Send((char *) (const char *) "  Night Hysteresis\r\n");
// #endif
// #ifdef WITH_TEMP_CONTROL
// Usart1Send((char *) (const char *) "  Temp Control\r\n");
// #endif
// #ifdef USE_WITH_SYNC
// Usart1Send((char *) (const char *) "  Sync by Edges\r\n");
// #else
// Usart1Send((char *) (const char *) "  Sync by ADC\r\n");
// #endif
// #ifdef USE_GSM
// Usart1Send((char *) (const char *) "  Uses GSM for SMS data\r\n");
// #endif

	while (1)
	{
		switch (main_state)
		{
			case MAIN_INIT:
				main_state = SYNCHRO_ADC;
				ADC1->CR |= ADC_CR_ADSTART;
				seq_ready = 0;

				Update_TIM14_CH1 (512);		//permito 1.75V en LM311
				break;

			case SYNCHRO_ADC:
				if (seq_ready)
				{
					Usart1Send((char *) (const char *) "ADC Sync getted!\r\n");
					main_state = SET_ZERO_CURRENT;
					seq_ready = 0;
					timer_standby = 2000;

					//TODO: para debug pruebo sin INT!!!!
					EXTIOff ();
				}
				break;

			case SET_ZERO_CURRENT:
				if (!STOP_JUMPER)
				{
					// if (!timer_meas)
					// {
					// 	timer_meas = 5;
					//
					// 	// if (d < 425)
					// 	if (d < 25)		//empiezo suabe
					// 		d++;
					//
					// 	UpdateTIMSync (d);
					// 	LED_OFF;
					// }
					if (d != 25)
					{
						d = 25;
						UpdateTIMSync (d);
						timer_meas = 100;		//dejo 100ms como minimo
					}
				}
				else
				{
					if (!timer_meas)
					{
						LED_ON;
						d = 0;
						UpdateTIMSync (0);
					}
				}
				break;

			case MAIN_OVERCURRENT:
				if (!timer_standby)
				{
					timer_standby = 100;
					if (LED)
						LED_OFF;
					else
						LED_ON;
				}

				if (STOP_JUMPER)
					main_state = SET_ZERO_CURRENT;
				break;

			default:
				main_state = SYNCHRO_ADC;
				break;
		}	//fin switch main_state

		if (!timer_standby)
		{
			timer_standby = 2000;
			sprintf (s_lcd, "VIN: %d, VOUT: %d, d: %d\r\n", Vin_Sense, Vout_Sense, d);
			Usart1Send(s_lcd);
		}

		if (current_excess)
		{
			current_excess = 0;
			d = 0;
			Usart1Send("\r\n Overcurrent!");
			main_state = MAIN_OVERCURRENT;
		}
	}	//fin while 1




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
