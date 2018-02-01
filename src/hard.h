/*
 * hard.h
 *
 *  Created on: 28/11/2013
 *      Author: Mariano
 */

#ifndef HARD_H_
#define HARD_H_


//----------- Defines For Configuration -------------
//----------- Hardware Board Version -------------
#define VER_1_0
// #define VER_1_1		//mismo pinout que VER_1_0

//-------- Type of Program ----------------
// #define DEBUG_ON


//-------- Type of Program and Features ----------------

//-------- Kind of Reports Sended ----------------

//-------- Others Configurations depending on the formers ------------
//-------- Hysteresis Conf ------------------------

//-------- PWM Conf ------------------------

//-------- End Of Defines For Configuration ------

#if (defined VER_1_0 || defined VER_1_1)
//GPIOA pin0	Vin_Sense
//GPIOA pin1	Vout_Sense
//GPIOA pin2	I_Sense
//GPIOA pin2	para pruebas
// #define ISENSE_ON	GPIOA->BSRR = 0x00000004
// #define ISENSE_OFF GPIOA->BSRR = 0x00040000


//GPIOA pin3	NC
//GPIOA pin4	NC
//GPIOA pin5	NC

//GPIOA pin6	para TIM3_CH1
//GPIOA pin7	NC

//GPIOB pin0
#define OVERCURRENT	((GPIOB->IDR & 0x0001) == 0)

//GPIOB pin1	TIM14_CH1 o TIM3_CH4

//GPIOA pin8	para TIM1_CH1
// #define CTRL_M_B_ON	GPIOA->BSRR = 0x00000100
// #define CTRL_M_B_OFF GPIOA->BSRR = 0x01000000


//GPIOA pin9
//GPIOA pin10	usart1 tx rx

//GPIOA pin11	NC
//GPIOA pin12	NC
//GPIOA pin13	NC
//GPIOA pin14	NC

//GPIOA pin15
#define LED ((GPIOA->ODR & 0x8000) != 0)
#define LED_ON	GPIOA->BSRR = 0x00008000
#define LED_OFF GPIOA->BSRR = 0x80000000

//GPIOB pin3	NC
//GPIOB pin4	NC
//GPIOB pin5	NC

//GPIOB pin6
#define STOP_JUMPER ((GPIOB->IDR & 0x0040) == 0)

//GPIOB pin7	NC
#endif	//


//ESTADOS DEL PROGRAMA PRINCIPAL
// #if (defined USE_GSM_GATEWAY) || (defined USE_GSM) || (defined USE_GPS)
#if (defined USE_GSM_GATEWAY) || (defined USE_GPS)
#define MAIN_INIT				0
#define MAIN_INIT_1				1
#define MAIN_SENDING_CONF		2
#define MAIN_WAIT_CONNECT_0		3
#define MAIN_WAIT_CONNECT_1		4
#define MAIN_WAIT_CONNECT_2		5
#define MAIN_READING_TCP		6
#define MAIN_TRANSPARENT		7
#define MAIN_AT_CONFIG_2B		8
#define MAIN_ERROR				9

#define MAIN_STAND_ALONE		10
#define MAIN_GROUPED			11
#define MAIN_NETWORKED			12
#define MAIN_NETWORKED_1		13
#define MAIN_IN_MAIN_MENU		14
#endif

//ESTADOS DEL PROGRAMA PRINCIPAL EN MODO MQTT
#ifdef WIFI_TO_MQTT_BROKER
typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_sending_conf,
  wifi_state_wait_ip,
  wifi_state_wait_ip1,
  wifi_state_idle,
  wifi_state_connecting,
  wifi_state_connected,
  wifi_state_disconnected,
  wifi_state_error,
  wifi_state_socket_close,
  mqtt_socket_create,
  client_conn,
  mqtt_connect,
  mqtt_sub,
  mqtt_pub,
  mqtt_device_control,
  wifi_undefine_state       = 0xFF,
} wifi_state_t;
#endif

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

//Estados cuando la lampara esta prendida
typedef enum
{
	init_airplane0 = 0,
	init_airplane1,
	meas_init,
	meas_meas,
	meas_reporting0,
	meas_reporting1,
	meas_reporting2,
  	meas_go_airplane

} lamp_on_state_t;


#define SIZEOF_DATA1	512
#define SIZEOF_DATA		256
#define SIZEOF_DATA512	SIZEOF_DATA1
#define SIZEOF_DATA256	SIZEOF_DATA
#define SIZEOF_BUFFTCP	SIZEOF_DATA


//--- Temas con el sync de relay
//#define TT_DELAYED_OFF		5600		//para relay metab
//#define TT_DELAYED_ON		6560		//para relay metab
#ifdef USE_WITH_SYNC
#define TT_DELAYED_OFF		5260		//para relay placa redonda
#define TT_DELAYED_ON		5400		//para relay placa redonda
#else
#define TT_DELAYED_OFF		6660		//para relay placa redonda
#define TT_DELAYED_ON		7000		//para relay placa redonda

#define ADC_THRESHOLD		1024		//threshold para deteccion de flanco
#define ADC_NOISE				50		//threshold para ruido de ADC
#endif
#define TT_RELAY			60		//timeout de espera antes de pegar o despegar el relay por default

enum Relay_State {

	ST_OFF = 0,
	ST_WAIT_ON,
	ST_DELAYED_ON,
	ST_ON,
	ST_WAIT_OFF,
	ST_DELAYED_OFF

};

//--- Temas de la medicion de potencia
// #define KW			0.01013		//R originales en OPAMP
// #define KW			0.01992		//con los cambos en las R y ajustado en 300W MUESTRA A
// #define KW			0.02131		//midiendo desde 50 a 300W en MUESTRA A ajustado con python "ajuste_potencia.py" 19-12-17
#define KW			0.02119		//midiendo a 300W en MUESTRA A con programa USE_ONLY_POWER_SENSE 19-12-17
// #define MIN_SENSE_POWER		753		//15W con KW
#define MIN_SENSE_POWER		1506		//30W con KW

//--- Temas con la medicion de tension
//MUESTRA A (BV)
// #define GLITCH_VOLTAGE			764		//equivale a 100V 0.616V V_Sense
// #define DISCONNECT_VOLTAGE		1786		//equivale 160V 1.44V V_Sense
// #define CONNECT_VOLTAGE			2233		//equivale 180V 1.8V V_Sense

//MUESTRA B KIRNO	15-12-2017
#define CONNECT_VOLTAGE			2035		//equivale 180V 1.64V V_Sense
#define DISCONNECT_VOLTAGE		1662		//equivale 160V 1.34V V_Sense
#define GLITCH_VOLTAGE			636		//equivale a 100V 0.512V V_Sense



/* Module Functions ------------------------------------------------------------*/
unsigned short GetHysteresis (unsigned char);
unsigned char GetNew1to10 (unsigned short);
void UpdateVGrid (void);
void UpdateIGrid (void);
unsigned short GetVGrid (void);
unsigned short GetIGrid (void);
unsigned short PowerCalc (unsigned short, unsigned short);
unsigned short PowerCalcMean8 (unsigned short * p);
void ShowPower (char *, unsigned short, unsigned int, unsigned int);

#endif /* HARD_H_ */
