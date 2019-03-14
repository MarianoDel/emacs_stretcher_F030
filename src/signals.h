//------------------------------------------------
// #### PROYECTO STRETCHER F030 - Power Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### SIGNALS.H ################################
//------------------------------------------------
#ifndef _SIGNALS_H_
#define _SIGNALS_H_
#include "comm.h"		//para respuestas
#include "hard.h"


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

} signal_frequency_t;

typedef enum {
	ZERO_DEG_OFFSET = 0,
        NINTY_DEG_OFFSET,
	HUNDRED_TWENTY_DEG_OFFSET,
        HUNDRED_EIGHTY_DEG_OFFSET,
        TWO_HUNDRED_FORTY_DEG_OFFSET

} signal_offset_t;

typedef struct {
    signal_type_t signal;
    signal_frequency_t frequency;
    signal_offset_t offset;
    unsigned char power;
    unsigned char synchro_needed;    //por ahora salen siempre sincronizadas

    //internals
    unsigned short t1;
    unsigned short t2;

    unsigned short kprop;
    unsigned short kinteg;
    unsigned short kderv;

} signals_struct_t;

typedef enum {
	TREATMENT_INIT_FIRST_TIME = 0,
	TREATMENT_STANDBY,
	TREATMENT_START_TO_GENERATE,
	TREATMENT_GENERATING,
	TREATMENT_STOPPING,
	TREATMENT_STOPPING2,
        TREATMENT_JUMPER_PROTECTED,
        TREATMENT_JUMPER_PROTECT_OFF

} treatment_t;

typedef enum {
	ERROR_OK = 0,
	ERROR_OVERCURRENT,
	ERROR_NO_CURRENT,
	ERROR_SOFT_OVERCURRENT,
	ERROR_OVERTEMP

} error_t;


//TIPO de descarga y estado de signal
typedef enum
{
	GEN_SIGNAL_INIT = 0,
        GEN_SIGNAL_WAIT_FOR_SYNC,
        GEN_SIGNAL_WAIT_FOR_FIRST_SYNC,        
        GEN_SIGNAL_WAIT_T1,
        GEN_SIGNAL_DRAWING,
        GEN_SIGNAL_WAIT_T2,
	GEN_SIGNAL_STOPPED_BY_INT,
        GEN_SIGNAL_DRAWING_ENDED

} gen_signal_state_t;


//--- Exported constants ---//
#define TAU_LR_MILLIS    9

//--- Exported macro ---//
#define SIZEOF_SIGNALS		100

#define ERROR_OVERCURRENT_MASK			0x01
#define ERROR_NO_CURRENT_MASK				0x02
#define ERROR_OVERTEMP_MASK				0x04
#define ERROR_SOFT_OVERCURRENT_MASK		0x08
#define ERROR_FLUSH_MASK					0xff

#define SIZEOF_OVERCURRENT_BUFF			8

#define CURRENT_INTEGRAL_MAX_ERRORS        SIGNAL_ADMITED_WITH_NO_CURRENT
//TODO: cambiar esto si la senial generada tiene la misma cantidad de puntos para todas las freq
#define CURRENT_INTEGRAL_THRESHOLD_10HZ         270
#define CURRENT_INTEGRAL_THRESHOLD_30HZ         90
#define CURRENT_INTEGRAL_THRESHOLD_60HZ         55

#define SAMPLE_TIME_10HZ    500
#define SAMPLE_TIME_30HZ    166
#define SAMPLE_TIME_60HZ    83

#define FlushErrorStatus() SetErrorStatus(ERROR_FLUSH_MASK)

#define SIGNAL_PWM_FAST_DISCHARGE do {\
        HIGH_LEFT_PWM (0);\
        LOW_RIGHT_PWM (0);\
    } while (0)

#ifdef VER_2_0
#define SIGNAL_PWM_NORMAL_DISCHARGE do {\
        HIGH_LEFT_PWM (0);\
        LOW_RIGHT_PWM (DUTY_ALWAYS);\
    } while (0)
#endif

#if (defined VER_1_0) || (defined VER_1_1)
#define SIGNAL_PWM_NORMAL_DISCHARGE do {\
        HIGH_LEFT_PWM(0);
        LOW_LEFT_PWM(0);
        HIGH_RIGHT_PWM(0);
        LOW_RIGHT_PWM(DUTY_ALWAYS);
    } while (0)
#endif

//--- Exported functions ---//
// resp_t SetSignalType (signals_struct_t *, signal_type_t);
resp_t SetSignalTypeAndOffset (signal_type_t, signal_offset_t);
resp_t SetFrequency (signal_frequency_t);
resp_t SetPower (unsigned char);
void GenerateSignalReset (void);
void GenerateSignal (void);
resp_t AssertTreatmentParams (void);
resp_t StartTreatment (void);
void StopTreatment (void);
error_t GetErrorStatus (void);
void SetErrorStatus (error_t);
void SendAllConf (void);
void TreatmentManager (void);
void Overcurrent_Shutdown (void);
void TreatmentManager_IntSpeed (void);

treatment_t GetTreatmentState (void);
gen_signal_state_t GetGenSignalState (void);

#endif    /* _SIGNALS_H_ */

//--- end of file ---//

