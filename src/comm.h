/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMM_H_
#define COMM_H_


//--- Exported types ---//
//--- Exported constants ---//

//--- Exported macro ---//
#define GET_PARAMS	10
#define GET_TEMP 11
#define GET_GAUSS 12
#define CHANNEL	13
#define SET_DISPLAY	14
#define CMD_DISPLAY	15
#define KEEP_ALIVE	16

#define ERROR	50

//--- Exported functions ---//
unsigned char InterpretarMsg (char *);
void AnalizarMsg (char *);


#endif
//--- End ---//
//--- END OF FILE ---//
