/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGNALS_H
#define __SIGNALS_H
#include "comm.h"		//para respuestas

//--- Exported types ---//
typedef enum {
	TREATMENT_INIT_FIRST_TIME = 0,
	TREATMENT_STANDBY,
	TREATMENT_START_TO_GENERATE,
	TREATMENT_GENERATING,
	TREATMENT_GENERATING_WITH_SYNC,
	TREATMENT_STOPPING,
	TREATMENT_STOPPING2

} treatment_t;

typedef enum {
	ERROR_OK = 0,
	ERROR_OVERCURRENT,
	ERROR_NO_CURRENT,
	ERROR_SOFT_OVERCURRENT,
	ERROR_OVERTEMP

} error_t;

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
	unsigned char synchro_needed;

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
	STOPPED_BY_INT

} discharge_state_t;


//--- Exported constants ---//

//--- Exported macro ---//
#define SIZEOF_SIGNALS		150

#define ERROR_OVERCURRENT_MASK			0x01
#define ERROR_NO_CURRENT_MASK				0x02
#define ERROR_OVERTEMP_MASK				0x04
#define ERROR_SOFT_OVERCURRENT_MASK		0x08
#define ERROR_FLUSH_MASK					0xff

#define SIZEOF_OVERCURRENT_BUFF			8

#define FlushErrorStatus() SetErrorStatus(ERROR_FLUSH_MASK)

//--- Exported functions ---//
resp_t SetSignalType (signal_type_t);
resp_t SetFrequency (frequency_t);
resp_t SetPower (unsigned char);
void GenerateSignal (void);
resp_t AssertTreatmentParams (void);
treatment_t GetTreatmentState (void);
resp_t StartTreatment (void);
void StopTreatment (void);
error_t GetErrorStatus (void);
void SetErrorStatus (error_t);
void SendAllConf (void);
void TreatmentManager (void);
void Overcurrent_Shutdown (void);
void TreatmentManager_IntSpeed (void);

#endif
//--- End ---//
//--- END OF FILE ---//
