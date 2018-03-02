
/* Includes ------------------------------------------------------------------*/
#include "comm.h"
#include "signals.h"
#include <string.h>



//--- VARIABLES EXTERNAS ---//
//del main
extern const char s_ok [];

//--- VARIABLES GLOBALES ---//
//globales de este modulo
const char s_get_gauss [] = {"get_gauss"};
const char s_get_temp [] = {"get_temp"};
const char s_get_params [] = {"get_params"};
const char s_channel [] = {"channel"};
const char s_set_display [] = {"set_display"};
const char s_cmd_display [] = {"cmd_display"};
const char s_keepalive [] = {"keepalive"};


char pCmd [20];

//--- FUNCIONES DEL MODULO ---//
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

unsigned char InterpretarMsg (void)
{
	unsigned char resp = resp_not_own;
	char * pStr = buffMessages;

	//reviso canal propio o canal broadcast
	if ((strncmp(pStr, p_own_channel, sizeof(s_chf) - 1) == 0) ||
	    (strncmp(pStr, s_chf, sizeof(s_chf) - 1) == 0))
	{
		resp = resp_ok;
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


	}
}
