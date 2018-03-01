/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGNALS_H
#define __SIGNALS_H

//--- Exported types ---//
typedef enum {
	SQUARE_SIGNAL = 0,
	TRIANGULAR_SIGNAL,
	SINUSOIDAL_SIGNAL

} signal_type_t;

typedef enum {
	TEN_HZ = 0,
	THIRTY_HZ,
	SIXTY_HZ

} frequency_t;

typedef struct {
	signal_type_t signal;
	frequency_t frequency;
	unsigned char freq_table_inc;
	unsigned char power;

	//internals
	unsigned short kprop;
	unsigned short kinteg;
	unsigned short kderv;

} signals_struct_t;

//TIPO de descarga y estado de signal
typedef enum
{
	INIT_DISCHARGE = 0,
	NORMAL_DISCHARGE,
	TAU_DISCHARGE,
	FAST_DISCHARGE,
	STOP_ALL,
	STOPPED

} discharge_state_t;


//--- Exported constants ---//

//--- Exported macro ---//

//--- Exported functions ---//
void SetSignalType (signal_type_t);
void SetFrequency (frequency_t);
void SetPower (unsigned char);
void GenerateSignal (void);

#endif
//--- End ---//
//--- END OF FILE ---//
