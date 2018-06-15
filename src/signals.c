
/* Includes ------------------------------------------------------------------*/
#include "signals.h"
#include "hard.h"
#include "stm32f0xx.h"
#include "tim.h"
#include "dsp.h"
#include "adc.h"

#include "uart.h"
#include "gpio.h"
#include <stdio.h>


//--- VARIABLES EXTERNAS ---//
//del ADC
extern volatile unsigned char seq_ready;
extern volatile unsigned short adc_ch[];

//del Main
extern volatile unsigned short timer_signals;

//--- VARIABLES GLOBALES ---//
treatment_t treatment_state = TREATMENT_INIT_FIRST_TIME;
signals_struct_t signal_to_gen;
discharge_state_t discharge_state = INIT_DISCHARGE;
unsigned char global_error = 0;

unsigned short * p_signal;
unsigned short * p_signal_running;

short d = 0;

unsigned char protected = 0;

//-- para determinacion de soft overcurrent ------------
#ifdef USE_SOFT_OVERCURRENT
unsigned short soft_overcurrent_max_current_in_cycles [SIZEOF_OVERCURRENT_BUFF];
unsigned short soft_overcurrent_treshold = 0;
unsigned short soft_overcurrent_index = 0;
#endif

//-- para determinar no current
//SIZEOF_SIGNALS * max_ADC = 150 * 1023 = 153450
//ojo, depende del salto de indice en la tabla segun la freq elegida
#ifdef USE_SOFT_NO_CURRENT
unsigned int current_integral = 0;
unsigned int current_integral_running = 0;
unsigned char current_integral_ended = 0;
unsigned char current_integral_errors = 0;
unsigned short current_integral_threshold = 0;
#endif

//Signals Templates
#define I_MAX 465
const unsigned short s_senoidal_1_5A [SIZEOF_SIGNALS] ={0,19,38,58,77,96,115,134,152,171,
														206,224,240,257,273,288,303,318,332,
														358,370,381,392,402,412,420,428,435,
														447,452,456,460,462,464,464,464,464,
														460,456,452,447,442,435,428,420,412,
														392,381,370,358,345,332,318,303,288,
														257,240,224,206,189,171,152,134,115,
														77,58,38,19,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0};

const unsigned short s_cuadrada_1_5A [SIZEOF_SIGNALS] = {465,465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,465,465,465,465,465,
														465,465,465,465,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0};

const unsigned short s_triangular_1_5A [SIZEOF_SIGNALS] = {0,6,12,18,24,31,37,43,49,55,
														68,74,80,86,93,99,105,111,117,
														130,136,142,148,155,161,167,173,179,
														192,198,204,210,217,223,229,235,241,
														254,260,266,272,279,285,291,297,303,
														316,322,328,334,341,347,353,359,365,
														378,384,390,396,403,409,415,421,427,
														440,446,452,458,465,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0};

const unsigned short s_triangular_6A [SIZEOF_SIGNALS] = {0,11,23,35,47,59,71,83,95,107,
                                                           131,143,155,167,179,191,203,215,227,
                                                           251,263,275,287,299,311,323,335,347,
                                                           371,383,395,407,419,431,443,455,467,
                                                           491,503,515,527,539,551,563,575,587,
                                                           611,623,635,647,659,671,683,695,707,
                                                           731,743,755,767,779,791,803,815,827,
                                                           851,863,875,887,899,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0,
                                                           0,0,0,0,0,0,0,0,0};


//--- FUNCIONES DEL MODULO ---//
void TreatmentManager (void)
{
    switch (treatment_state)
    {
    case TREATMENT_INIT_FIRST_TIME:
        HIGH_LEFT_PWM(0);
        LOW_LEFT_PWM(0);
        HIGH_RIGHT_PWM(0);
        LOW_RIGHT_PWM(DUTY_ALWAYS);

        if (AssertTreatmentParams() == resp_ok)
        {
            treatment_state = TREATMENT_STANDBY;
            ChangeLed(LED_TREATMENT_STANDBY);
        }
        break;

    case TREATMENT_STANDBY:
        break;

    case TREATMENT_START_TO_GENERATE:		//reviso una vez mas los parametros y no tener ningun error
        if ((AssertTreatmentParams() == resp_ok) && (GetErrorStatus() == ERROR_OK))
        {
            GenerateSignalReset();

#ifdef USE_SOFT_OVERCURRENT
            //cargo valor maximo de corriente para el soft_overcurrent
            soft_overcurrent_treshold = 1.2 * I_MAX * signal_to_gen.power / 100;
            soft_overcurrent_index = 0;

            for (unsigned char i = 0; i < SIZEOF_OVERCURRENT_BUFF; i++)
                soft_overcurrent_max_current_in_cycles[i] = 0;
#endif
#ifdef USE_SOFT_NO_CURRENT
            current_integral_errors = 0;
            if (signal_to_gen.frequency == TEN_HZ)
                current_integral_threshold = CURRENT_INTEGRAL_THRESHOLD_10HZ;
            else if (signal_to_gen.frequency == THIRTY_HZ)
                current_integral_threshold = CURRENT_INTEGRAL_THRESHOLD_30HZ;
            else
                current_integral_threshold = CURRENT_INTEGRAL_THRESHOLD_60HZ;
#endif
            EXTIOn();
            if (signal_to_gen.synchro_needed)
                treatment_state = TREATMENT_GENERATING_WITH_SYNC;
            else
                treatment_state = TREATMENT_GENERATING;

            ChangeLed(LED_TREATMENT_GENERATING);
        }
        else
        {
            //error de parametros
            treatment_state = TREATMENT_INIT_FIRST_TIME;
        }
        break;

    case TREATMENT_GENERATING:
        //Cosas que dependen de las muestras
        //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
        //la respuesta
        GenerateSignal();

#ifdef USE_SOFT_OVERCURRENT
        //TODO: poner algun synchro con muestras para que no ejecute el filtro todo el tiempo
        //soft current overload check
        if (MAFilter8 (soft_overcurrent_max_current_in_cycles) > soft_overcurrent_treshold)
        {
            treatment_state = TREATMENT_STOPPING;
            SetErrorStatus(ERROR_SOFT_OVERCURRENT);
        }
#endif

#ifdef USE_SOFT_NO_CURRENT
        if (current_integral_ended)
        {
            current_integral_ended = 0;
            if (current_integral_errors < CURRENT_INTEGRAL_MAX_ERRORS)
            {
                if (current_integral < current_integral_threshold)
                {
                    current_integral_errors++;
                    Usart1Send("e\n");
                }
                else if (current_integral_errors)
                    current_integral_errors--;
            }
            else
            {
                treatment_state = TREATMENT_STOPPING;
                SetErrorStatus(ERROR_NO_CURRENT);
            }
        }                
#endif
        break;

    case TREATMENT_GENERATING_WITH_SYNC:
        break;

    case TREATMENT_STOPPING:
        //10ms descarga rapida y a idle
        HIGH_LEFT_PWM(0);
        LOW_RIGHT_PWM(DUTY_ALWAYS);
        timer_signals = 10;
        treatment_state = TREATMENT_STOPPING2;
        break;

    case TREATMENT_STOPPING2:		//aca lo manda directamente la int
        if (!timer_signals)
        {
            treatment_state = TREATMENT_INIT_FIRST_TIME;
            EXTIOff();
            ENABLE_TIM3;
            LED_OFF;
        }
        break;

    default:
        treatment_state = TREATMENT_INIT_FIRST_TIME;
        break;
    }
}

void TreatmentManager_IntSpeed (void)
{
    switch (treatment_state)
    {
        case TREATMENT_INIT_FIRST_TIME:
            if (!timer_signals)
            {
                HIGH_LEFT_PWM(0);
                LOW_LEFT_PWM(0);
                HIGH_RIGHT_PWM(0);
                LOW_RIGHT_PWM(DUTY_ALWAYS);

                if (GetErrorStatus() == ERROR_OK)
                {
                    GenerateSignalReset();
                    treatment_state = TREATMENT_GENERATING;
                    LED_OFF;
                    EXTIOn();
                }
            }
            break;

        case TREATMENT_GENERATING:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignal();

            break;

        case TREATMENT_STOPPING2:		//aca lo manda directamente la int
            if (!timer_signals)
            {
                treatment_state = TREATMENT_INIT_FIRST_TIME;
                EXTIOff();
                ENABLE_TIM3;
                LED_OFF;
                SetErrorStatus(ERROR_FLUSH_MASK);
                timer_signals = 30;    //30ms mas de demora despues de int
            }
            break;

        default:
            treatment_state = TREATMENT_INIT_FIRST_TIME;
            break;
    }
}

treatment_t GetTreatmentState (void)
{
    return treatment_state;
}

resp_t StartTreatment (void)
{
    if (treatment_state == TREATMENT_STANDBY)
    {
        if ((AssertTreatmentParams() == resp_ok) && (GetErrorStatus() == ERROR_OK))
        {
            treatment_state = TREATMENT_START_TO_GENERATE;
            return resp_ok;
        }
    }
    return resp_error;
}

void StopTreatment (void)
{
    if (treatment_state != TREATMENT_STANDBY)
        treatment_state = TREATMENT_STOPPING;
}

error_t GetErrorStatus (void)
{
	error_t error = ERROR_OK;

	if (global_error & ERROR_OVERTEMP_MASK)
		error = ERROR_OVERTEMP;
	else if (global_error & ERROR_OVERCURRENT_MASK)
		error = ERROR_OVERCURRENT;
	else if (global_error & ERROR_NO_CURRENT_MASK)
		error = ERROR_NO_CURRENT;
	else if (global_error & ERROR_SOFT_OVERCURRENT_MASK)
		error = ERROR_SOFT_OVERCURRENT;

	return error;
}

void SetErrorStatus (error_t e)
{
    if (e == ERROR_FLUSH_MASK)
        global_error = 0;
    else
    {
        if (e == ERROR_OVERTEMP)
            global_error |= ERROR_OVERTEMP_MASK;
        if (e == ERROR_OVERCURRENT)
            global_error |= ERROR_OVERCURRENT_MASK;
        if (e == ERROR_SOFT_OVERCURRENT)
            global_error |= ERROR_SOFT_OVERCURRENT_MASK;
        if (e == ERROR_NO_CURRENT)
            global_error |= ERROR_NO_CURRENT_MASK;
    }
}

//TODO: PONER UNA TRABA DE SETEOS PARA NO CAMBIAR NADA CORRIENDO
//recibe tipo de senial
resp_t SetSignalType (signal_type_t a)
{
    //TODO: despues cargar directamente los k
    if ((treatment_state != TREATMENT_INIT_FIRST_TIME) && (treatment_state != TREATMENT_STANDBY))
        return resp_error;

    if (a == SQUARE_SIGNAL)
        p_signal = (unsigned short *) s_cuadrada_1_5A;

#if (defined USE_PROTECTION_WITH_INT) && (defined INT_SPEED_RESPONSE)
    if (a == TRIANGULAR_SIGNAL)
        p_signal = (unsigned short *) s_triangular_6A;
#else
    if (a == TRIANGULAR_SIGNAL)
        p_signal = (unsigned short *) s_triangular_1_5A;    
#endif

    if (a == SINUSOIDAL_SIGNAL)
        p_signal = (unsigned short *) s_senoidal_1_5A;

    signal_to_gen.signal = a;

    return resp_ok;
}

//recibe referecnia a la estructura de senial
//recibe tipo de senial
// resp_t SetSignalType (signals_struct_t * s, signal_type_t a)
// {
//     //TODO: despues cargar directamente los k
//     if ((treatment_state != TREATMENT_INIT_FIRST_TIME) && (treatment_state != TREATMENT_STANDBY))
//         return resp_error;

//     if (a == SQUARE_SIGNAL)
//         p_signal = (unsigned short *) s_cuadrada_1_5A;

// #if (defined USE_PROTECTION_WITH_INT) && (defined INT_SPEED_RESPONSE)
//     if (a == TRIANGULAR_SIGNAL)
//         p_signal = (unsigned short *) s_triangular_6A;
// #else
//     if (a == TRIANGULAR_SIGNAL)
//         p_signal = (unsigned short *) s_triangular_1_5A;    
// #endif

//     if (a == SINUSOIDAL_SIGNAL)
//         p_signal = (unsigned short *) s_senoidal_1_5A;

//     // signal_to_gen.signal = a;
//     s->signal = a;

//     return resp_ok;
// }

resp_t SetFrequency (frequency_t a)
{
    if ((treatment_state != TREATMENT_INIT_FIRST_TIME) && (treatment_state != TREATMENT_STANDBY))
        return resp_error;

    if (a == TEN_HZ)
        signal_to_gen.freq_table_inc = 1;

    if (a == THIRTY_HZ)
        signal_to_gen.freq_table_inc = 3;

    if (a == SIXTY_HZ)
        signal_to_gen.freq_table_inc = 6;

    signal_to_gen.frequency = a;

    return resp_ok;
}

resp_t SetPower (unsigned char a)
{
    if ((treatment_state != TREATMENT_INIT_FIRST_TIME) && (treatment_state != TREATMENT_STANDBY))
        return resp_error;

    if (a > 100)
        signal_to_gen.power = 100;
    else if (a < 10)
        signal_to_gen.power = 10;
    else
        signal_to_gen.power = a;

    return resp_ok;
}

//verifica que se cumplan con todos los parametros para poder enviar una senial coherente
resp_t AssertTreatmentParams (void)
{
    resp_t resp = resp_error;

    if ((signal_to_gen.power > 100) || (signal_to_gen.power < 10))
        return resp;

    if ((signal_to_gen.freq_table_inc != 1) &&
        (signal_to_gen.freq_table_inc != 3) &&
        (signal_to_gen.freq_table_inc != 6))
        return resp;

    if ((signal_to_gen.frequency != TEN_HZ) &&
        (signal_to_gen.frequency != THIRTY_HZ) &&
        (signal_to_gen.frequency != SIXTY_HZ))
        return resp;

    if ((signal_to_gen.signal != SQUARE_SIGNAL) &&
        (signal_to_gen.signal != TRIANGULAR_SIGNAL) &&
        (signal_to_gen.signal != SINUSOIDAL_SIGNAL))
        return resp;

    //TODO: revisar tambien puntero!!!!
    return resp_ok;
}

void SendAllConf (void)
{
    char b [64];
    sprintf(b, "channel: %s\n", GetOwnChannel());
    Usart1Send(b);
    sprintf(b, "signal: %d\n", signal_to_gen.signal);
    Usart1Send(b);
    sprintf(b, "freq: %d, inc: %d\n", signal_to_gen.frequency, signal_to_gen.freq_table_inc);
    Usart1Send(b);
    sprintf(b, "power: %d\n\n", signal_to_gen.power);
    Usart1Send(b);
}

//reset a antes de la generacion de seniales
void GenerateSignalReset (void)
{
    discharge_state = INIT_DISCHARGE;
}

//la llama el manager para generar las seniales, si no esta el jumper de proteccion genera
//sino espera a que sea quitado
void GenerateSignal (void)
{
    if (!protected)
    {
        if (!STOP_JUMPER)
        {              
            if (seq_ready)
            {
                seq_ready = 0;

                switch (discharge_state)
                {
                case INIT_DISCHARGE:			//arranco siempre con descarga por TAU
                    HIGH_LEFT_PWM(0);
                    LOW_RIGHT_PWM(DUTY_ALWAYS);
                    discharge_state = NORMAL_DISCHARGE;
                    p_signal_running = p_signal;

                    //no current
                    current_integral_running = 0;
                    current_integral_ended = 0;
                    break;

                case NORMAL_DISCHARGE:

                    d = PID_roof ((*p_signal_running * signal_to_gen.power / 100),
                                  I_Sense, d);

                    //reviso si necesito cambiar a descarga por tau
                    if (d < 0)
                    {
                        HIGH_LEFT_PWM(0);
                        discharge_state = TAU_DISCHARGE;
                        d = 0;	//limpio para pid descarga
                    }
                    else
                    {
                        if (d > DUTY_95_PERCENT)		//no pasar del 95% para dar tiempo a los mosfets
                            d = DUTY_95_PERCENT;

                        HIGH_LEFT_PWM(d);
                    }
                    break;

                case TAU_DISCHARGE:		//la medicion de corriente sigue siendo I_Sense

                    d = PID_roof ((*p_signal_running * signal_to_gen.power / 100),
                                  I_Sense, d);	//OJO cambiar este pid

                    //reviso si necesito cambiar a descarga rapida
                    if (d < 0)
                    {
                        if (-d < DUTY_100_PERCENT)
                            LOW_RIGHT_PWM(DUTY_100_PERCENT + d);
                        else
                            LOW_RIGHT_PWM(0);

                        discharge_state = FAST_DISCHARGE;
                    }
                    else
                    {
                        //vuelvo a NORMAL_DISCHARGE
                        if (d > DUTY_95_PERCENT)		//no pasar del 95% para dar tiempo a los mosfets
                            d = DUTY_95_PERCENT;

                        HIGH_LEFT_PWM(d);
                        discharge_state = NORMAL_DISCHARGE;
                    }
                    break;

                case FAST_DISCHARGE:		//la medicion de corriente ahora esta en I_Sense_negado

                    d = PID_roof ((*p_signal_running * signal_to_gen.power / 100),
                                  I_Sense_negado, d);	//OJO cambiar este pid

                    //reviso si necesito cambiar a descarga rapida
                    if (d < 0)
                    {
                        if (-d < DUTY_100_PERCENT)
                            LOW_RIGHT_PWM(DUTY_100_PERCENT + d);
                        else
                            LOW_RIGHT_PWM(0);
                    }
                    else
                    {
                        //vuelvo a TAU_DISCHARGE
                        LOW_RIGHT_PWM(DUTY_ALWAYS);
                        discharge_state = TAU_DISCHARGE;
                    }
                    break;

                case STOPPED_BY_INT:		//lo freno la interrupcion
                    break;

                default:
                    discharge_state = INIT_DISCHARGE;
                    break;
                }

                //-- Soft Overcurrent --//
#ifdef USE_SOFT_OVERCURRENT
                soft_overcurrent_max_current_in_cycles[soft_overcurrent_index] = I_Sense;
                if (soft_overcurrent_index < (SIZEOF_OVERCURRENT_BUFF - 1))
                    soft_overcurrent_index++;
                else
                    soft_overcurrent_index = 0;
#endif

                //-- Signal Update --//
                if ((p_signal_running + signal_to_gen.freq_table_inc) < (p_signal + SIZEOF_SIGNALS))
                {
                    p_signal_running += signal_to_gen.freq_table_inc;
#ifdef USE_SOFT_NO_CURRENT
                    current_integral_running += I_Sense;
#endif
                }
                else
                {
                    p_signal_running = p_signal;
#ifdef USE_SOFT_NO_CURRENT
                    current_integral = current_integral_running;
                    current_integral_running = 0;
                    current_integral_ended = 1;
#endif
                }
            }    //cierra sequence
        }    //cierra jumper protected
        else
        {
            //me piden que no envie senial y proteja
            HIGH_LEFT_PWM(0);
            LOW_RIGHT_PWM(0);
            protected = 1;
        }
    }    //cierra variable protect
    else
    {
        //estoy protegido reviso si tengo que salir
        if (!STOP_JUMPER)
        {
            //tengo que salir del modo
            protected = 0;
            LOW_RIGHT_PWM(DUTY_ALWAYS);
        }
    }
}

//hubo sobrecorriente, me llaman desde la interrupcion
void Overcurrent_Shutdown (void)
{
#ifdef INT_WITH_LED
    if (LED)
        LED_OFF;
    else
        LED_ON;
#endif
	//primero freno todos los PWM
	HIGH_LEFT_PWM(0);
	LOW_RIGHT_PWM(0);

	DISABLE_TIM3;

	//freno la generacionde la senial
	discharge_state = STOPPED_BY_INT;

	//ahora aviso del error
	SetErrorStatus(ERROR_OVERCURRENT);

	//meto la generacion en Overcurrent
	timer_signals = 10;
	treatment_state = TREATMENT_STOPPING2;
	// EXTIOff();
}

//--- end of file ---//
