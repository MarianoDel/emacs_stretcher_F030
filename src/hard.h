/*
 * hard.h
 *
 *  Created on: 28/11/2013
 *      Author: Mariano
 */

#ifndef HARD_H_
#define HARD_H_


//----------- Defines For Configuration -------------
//----------- Harcode Direction -------------
// #define OWN_CHANNEL 1
#define OWN_CHANNEL 2
// #define OWN_CHANNEL 3

//----------- Hardware Board Version -------------
// #define VER_1_0
#define VER_1_1		//mismo pinout que VER_1_0

//-------- Type of Program ----------------
#define POWER_WITH_MANAGEMENT
// #define ONLY_POWER_WITHOUT_MANAGEMENT
// #define INT_SPEED_RESPONSE    //genera seniales y ve int, 30ms y reactiva solo
// #define INT_WITH_LED        //no genero nada, solo espero int y muevo el LED


//-------- Type of Program and Features ----------------
//Si utiliza la proteccion con la int para cortar la corriente
#define USE_PROTECTION_WITH_INT

//Si utiliza la proteccion de soft overcurrent o no
// #define USE_SOFT_OVERCURRENT

//Si utiliza la proteccion de no current
#define USE_SOFT_NO_CURRENT

//Modo de uso de la USART (placa individual single - placa enganchada bus)
#define USART_IN_BUS
// #define USART_SINGLE

//-------- Kind of Reports Sended ----------------

//-------- Others Configurations depending on the formers ------------
#ifdef USART_IN_BUS
#define USART_TX_OUTPUT_OPEN_DRAIN
#endif

//-------- Hysteresis Conf ------------------------

//-------- PWM Conf ------------------------

//-------- End Of Defines For Configuration ------

#if (defined VER_1_0 || defined VER_1_1)
//GPIOA pin0	Input_Signal
//GPIOA pin1	I_Sense

//GPIOA pin2	PROT Input
#define PROTECT	((GPIOA->IDR & 0x0004) != 0)

//GPIOA pin3	I_Sense_negado

//GPIOA pin4	NC
//GPIOA pin5	NC

//GPIOA pin6	para TIM3_CH1	LOW_LEFT
//GPIOA pin7	para TIM3_CH2	HIGH_LEFT
//GPIOB pin0	para TIM3_CH3	LOW_RIGHT
//GPIOB pin1	para TIM3_CH4	HIGH_RIGHT

//GPIOA pin8	NC

//GPIOA pin9	usart1 tx
//GPIOA pin10	usart1 rx

//GPIOA pin11	NC

//GPIOA pin12
#define LED ((GPIOA->ODR & 0x1000) != 0)
#define LED_ON	GPIOA->BSRR = 0x00001000
#define LED_OFF GPIOA->BSRR = 0x10000000

//GPIOA pin13	NC
//GPIOA pin14	NC
//GPIOA pin15	NC

//GPIOB pin3	NC
//GPIOB pin4	NC
//GPIOB pin5	NC

//GPIOB pin6
#define STOP_JUMPER ((GPIOB->IDR & 0x0040) == 0)

//GPIOB pin7	NC
#endif	//


//ESTADOS DEL PROGRAMA PRINCIPAL
typedef enum
{
    MAIN_INIT = 0,
    SYNCHRO_ADC,
    SET_ZERO_CURRENT,
    MAIN_OVERCURRENT,
    SET_COUNTERS_AND_PHONE,
    LAMP_OFF,
    START_GSM,
    CONFIG_GSM,
    WELCOME_GSM,
    LAMP_ON,
    GO_TO_MAINS_FAILURE,
    MAINS_FAILURE

} main_state_t;

//ESTADOS DEL LED
typedef enum
{    
    START_BLINKING = 0,
    WAIT_TO_OFF,
    WAIT_TO_ON,
    WAIT_NEW_CYCLE
} led_state_t;

//Estados Externos de LED BLINKING
#define LED_NO_BLINKING    0
#define LED_TREATMENT_STANDBY    1
#define LED_TREATMENT_GENERATING    2


/* Module Functions ------------------------------------------------------------*/
void ChangeLed (unsigned char);
void UpdateLed (void);


#endif /* HARD_H_ */
