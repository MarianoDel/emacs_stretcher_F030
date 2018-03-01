
/* Includes ------------------------------------------------------------------*/
#include "signals.h"
#include "tim.h"
#include "dsp.h"
#include "adc.h"


//--- VARIABLES EXTERNAS ---//
//del ADC
extern volatile unsigned char seq_ready;
extern volatile unsigned short adc_ch[];

//--- VARIABLES GLOBALES ---//
signals_struct_t signal_to_gen;
discharge_state_t discharge_state = INIT_DISCHARGE;

unsigned short * p_signal;
unsigned short * p_signal_running;

short d = 0;

//Signals Templates
const unsigned short s_senoidal_0_5A [150] = {0,6,12,19,25,32,38,44,50,57,
														68,74,80,85,91,96,101,106,110,
														119,123,127,130,134,137,140,142,145,
														149,150,152,153,154,154,154,154,154,
														153,152,150,149,147,145,142,140,137,
														130,127,123,119,115,110,106,101,96,
														85,80,74,68,63,57,50,44,38,
														25,19,12,6,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0,
														0,0,0,0,0,0,0,0,0};

const unsigned short s_senoidal_1_5A [150] ={0,19,38,58,77,96,115,134,152,171,
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

const unsigned short s_cuadrada_1_5A [150] = {465,465,465,465,465,465,465,465,465,465,
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

const unsigned short s_triangular_1_5A [150] = {0,6,12,18,24,31,37,43,49,55,
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


//TODO: PONER UNA TRABA DE SETEOS PARANO CAMBIAR NADA CORRIENDO

//--- FUNCIONES DEL MODULO ---//
void SetSignalType (signal_type_t a)
{
	//TODO: despues cargar directamente los k

	if (a == SQUARE_SIGNAL)
		p_signal = (unsigned short *) s_cuadrada_1_5A;

	if (a == TRIANGULAR_SIGNAL)
		p_signal = (unsigned short *) s_triangular_1_5A;

	if (a == SINUSOIDAL_SIGNAL)
		p_signal = (unsigned short *) s_senoidal_1_5A;

	signal_to_gen.signal = a;

}

void SetFrequency (frequency_t a)
{
	if (a == TEN_HZ)
		signal_to_gen.freq_table_inc = 1;

	if (a == THIRTY_HZ)
		signal_to_gen.freq_table_inc = 3;

	if (a == SIXTY_HZ)
		signal_to_gen.freq_table_inc = 6;

	signal_to_gen.frequency = a;
}

void SetPower (unsigned char a)
{
	signal_to_gen.power = a;
}

void GenerateSignal (void)
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
				break;

			case NORMAL_DISCHARGE:

				d = PID_roof (*p_signal_running, I_Sense, d);

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

				d = PID_roof (*p_signal_running, I_Sense, d);	//OJO cambiar este pid

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

				d = PID_roof (*p_signal_running, I_Sense_negado, d);	//OJO cambiar este pid

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

			default:
				discharge_state = INIT_DISCHARGE;
				break;
		}

		//-- TRIANGULARES --//
		if ((p_signal_running + signal_to_gen.freq_table_inc) < &s_triangular_1_5A[150])
			p_signal_running += signal_to_gen.freq_table_inc;
		else
			p_signal_running = (unsigned short *) s_triangular_1_5A;

	}
}

//--- end of file ---//
