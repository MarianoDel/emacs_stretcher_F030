
/* Includes ------------------------------------------------------------------*/
#include "comm.h"
#include "signals.h"
#include "uart.h"

#include <string.h>



//--- VARIABLES EXTERNAS ---//
//del main
extern const char s_ok [];
// ------- Externals del Puerto serie  -------
extern unsigned char usart1_have_data;

//--- VARIABLES GLOBALES ---//
//globales de este modulo

//strings de comienzo o lineas intermedias
const char s_ch1 [] = {"ch1 "};
const char s_ch2 [] = {"ch2 "};
const char s_ch3 [] = {"ch3 "};
const char s_chf [] = {"chf "};
const char s_set_signal [] = {"signal "};
const char s_frequency [] = {"freq "};
const char s_power [] = {"power "};

//strings de fines de linea
const char s_square [] = {"square"};
const char s_triangular [] = {"triangular"};
const char s_sinusoidal [] = {"sinusoidal"};
const char s_ten_hz [] = {"10Hz"};
const char s_thirty_hz [] = {"30Hz"};
const char s_sixty_hz [] = {"60Hz"};
const char s_start_treatment [] = {"start treatment"};
const char s_stop_treatment [] = {"stop treatment"};
const char s_status [] = {"status"};
const char s_flush_errors [] = {"flush erros"};


char buffMessages [100];
const char * p_own_channel;


//--- FUNCIONES DEL MODULO ---//
void SetOwnChannel (unsigned char ch)
{
	if (ch == 1)
		p_own_channel = s_ch1;
	else if (ch == 2)
		p_own_channel = s_ch2;
	else
		p_own_channel = s_ch3;

}
void UpdateCommunications (void)
{
	if (SerialProcess() > 2)	//si tiene algun dato significativo
	{
		InterpretarMsg();
	}
}

//Procesa consultas desde el micro principal
//carga el buffer buffMessages y avisa con el flag msg_ready
unsigned char SerialProcess (void)
{
	unsigned char bytes_readed = 0;

	if (usart1_have_data)
	{
		usart1_have_data = 0;
		bytes_readed = ReadUsart1Buffer((unsigned char *) buffMessages, sizeof(buffMessages));
	}
	return bytes_readed;
}

resp_t InterpretarMsg (void)
{
	resp_t resp = resp_not_own;
	unsigned char broadcast = 0;
	char * pStr = buffMessages;

	//reviso canal propio o canal broadcast
	if ((strncmp(pStr, p_own_channel, sizeof(s_chf) - 1) == 0) ||
	    (strncmp(pStr, s_chf, sizeof(s_chf) - 1) == 0))
	{
		resp = resp_ok;

		//es broadcast?
		if (*(pStr + 3) == 'f')
			broadcast = 1;

		pStr += sizeof(s_chf);	//normalizo al mensaje, hay un espacio

		//-- Signal Setting
		if (strncmp(pStr, s_set_signal, sizeof(s_set_signal) - 1) == 0)
		{
			pStr += sizeof(s_set_signal);		//normalizo al payload, hay un espacio

			if (strncmp(pStr, s_square, sizeof(s_square) - 1) == 0)
				SetSignalType (SQUARE_SIGNAL);
			else if (strncmp(pStr, s_triangular, sizeof(s_triangular) - 1) == 0)
				SetSignalType (TRIANGULAR_SIGNAL);
			else if (strncmp(pStr, s_sinusoidal, sizeof(s_sinusoidal) - 1) == 0)
				SetSignalType (SINUSOIDAL_SIGNAL);
			else
				resp = resp_error;
		}

		//-- Frequency Setting
		if (strncmp(pStr, s_frequency, sizeof(s_frequency) - 1) == 0)
		{
			pStr += sizeof(s_frequency);		//normalizo al payload, hay un espacio

			if (strncmp(pStr, s_ten_hz, sizeof(s_ten_hz) - 1) == 0)
				SetFrequency (TEN_HZ);
			else if (strncmp(pStr, s_thirty_hz, sizeof(s_thirty_hz) - 1) == 0)
				SetFrequency (THIRTY_HZ);
			else if (strncmp(pStr, s_sixty_hz, sizeof(s_sixty_hz) - 1) == 0)
				SetFrequency (SIXTY_HZ);
			else
				resp = resp_error;
		}

		//-- Power Setting
		if (strncmp(pStr, s_power, sizeof(s_power) - 1) == 0)
		{
			pStr += sizeof(s_power);		//normalizo al payload, hay un espacio



		}

		//-- Start Treatment
		if (strncmp(pStr, s_start_treatment, sizeof(s_start_treatment) - 1) == 0)
		{
			//se puede empezar
			if (GetTreatmentState() == TREATMENT_STANDBY)
			{
				StartTreatment();
				resp = resp_ok;
			}
			else
				resp = resp_error;
		}

		//-- Status
		if (strncmp(pStr, s_status, sizeof(s_status) - 1) == 0)
		{
			//reviso errores y envio
			switch (GetErrorStatus())
			{
				case ERROR_OK:
					break;

				case ERROR_OVERCURRENT:
					break;

				case ERROR_NO_CURRENT:
					break;

				case ERROR_OVERTEMP:
					break;

			}
		}
	}	//fin if chx
	return resp;
}
