/*
 * hard.c
 *
 *  Created on: 28/03/2017
 *      Author: Mariano
 */

#include "hard.h"
#include "tim.h"
#include "stm32f0xx.h"
#include "adc.h"
#include "dsp.h"

#include <stdio.h>

/* Externals variables ---------------------------------------------------------*/
extern unsigned short timer_relay;
extern volatile unsigned short adc_ch[];
extern unsigned short zero_current;
extern unsigned short mains_voltage_filtered;


/* Global variables ------------------------------------------------------------*/
//unsigned char relay_state = 0;
enum Relay_State relay_state = ST_OFF;
unsigned char last_edge;
unsigned char mains_with_glitch = 0;
unsigned char mains_voltage_index = 0;
unsigned short mains_vector[8];

unsigned short max_igrid_last, max_igrid, min_igrid_last, min_igrid;
unsigned short max_vgrid_last, max_vgrid, min_vgrid_last, min_vgrid;
unsigned short igrid_update_samples;
unsigned short vgrid_update_samples;



/* Module Functions ------------------------------------------------------------*/


#ifdef WITH_HYST
unsigned short GetHysteresis (unsigned char hours_past)
{
	if (hours_past > 8)
		return HYST_MIN;
	else if (hours_past > 6)
		return HYST_6;
	else if (hours_past > 4)
		return HYST_4;
	else if (hours_past > 2)
		return HYST_2;
	else
		return HYST_MAX;
}
#endif

#ifdef WITH_1_TO_10_VOLTS
unsigned char GetNew1to10 (unsigned short light)	//prendo en 3722 a 4095 tengo 373 puntos
{
	unsigned short new_light = 0;

	if (light > VOLTAGE_PHOTO_ON)
	{
		new_light = light - VOLTAGE_PHOTO_ON;
	}
	new_light += PWM_MIN;

	if (new_light > 255)
		new_light = 255;

	// if (light < VOLTAGE_PHOTO_ON)
	// 	new_light = PWM_MIN;
	// else
	// {
	// 	new_light = light - VOLTAGE_PHOTO_ON;
	// 	new_light += PWM_MIN;
	// }

	return (unsigned char) new_light;
}
#endif

//Hay que llamarla sincronizado con las muestras, entran 312 en un ciclo
void UpdateVGrid (void)
{
	//miro la ultima medicion
	if (vgrid_update_samples < 350)    //312 un ciclo de 20ms
	{
		//reviso si es un maximo
	   if (V_Sense > max_vgrid)
			max_vgrid = V_Sense;

	   //   //reviso si es un minimo
	   //   if (V_Sense < min_vgrid)
	   //       min_vgrid = V_Sense;

	     vgrid_update_samples++;
	 }
	 else
	 {
		 //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
		 max_vgrid_last = max_vgrid;
		min_vgrid_last = min_vgrid;
		max_vgrid = 0;
		min_vgrid = 0;
		//   max_vgrid = 2048;
		//   min_vgrid = 2048;
		vgrid_update_samples = 0;

		//reviso si es un glitch
		if (max_vgrid_last < GLITCH_VOLTAGE)
			mains_with_glitch = 1;
		else
			mains_with_glitch = 0;

		//filtro de alimentacion
		if (mains_voltage_index < 8)
		{
			mains_vector[mains_voltage_index] = max_vgrid_last;
			mains_voltage_index++;
		}
		else
		{
			mains_voltage_filtered = MAFilter8(mains_vector);
			mains_voltage_index = 0;
			mains_vector[0] = max_vgrid_last;
		}
    }
}

unsigned char Mains_Glitch (void)
{
	return mains_with_glitch;
}

//Hay que llamarla sincronizado con las muestras, entran 312 en un ciclo
void UpdateIGrid (void)
{
    //miro la ultima medicion
    if (igrid_update_samples < 350)    //20 es toda la senoidal 23 es un ciclo y un octavo
    {
        //reviso si es un maximo
        if (I_Sense > max_igrid)
            max_igrid = I_Sense;

        //reviso si es un minimo
        if (I_Sense < min_igrid)
            min_igrid = I_Sense;

        igrid_update_samples++;
    }
    else
    {
        //if (LED2)
        //    LED2_OFF;
        //else
        //    LED2_ON;

        //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
        max_igrid_last = max_igrid;
        min_igrid_last = min_igrid;
        max_igrid = zero_current;
        min_igrid = zero_current;
        igrid_update_samples = 0;
    }
}

unsigned short GetVGrid (void)
{
    if (max_vgrid_last > min_vgrid_last)
        return max_vgrid_last - min_vgrid_last;    //valor de tension pico a pico
    else
        return 0;
}

unsigned short GetIGrid (void)
{
    if (max_igrid_last > min_igrid_last)
        return max_igrid_last - min_igrid_last;    //valor de corriente pico a pico
    else
        return 0;
}

unsigned short PowerCalc (unsigned short a, unsigned short b)
{
	unsigned int temp;
	temp = a * b;
	temp >>= 8;
	return (unsigned short) temp;
}

unsigned short PowerCalcMean8 (unsigned short * p)
{
	unsigned char i, j, ii, max_index, min_index;
	unsigned short power = 0;

	//busco el maximo
	for (i = 0; i < 10; i++)
	{
		if (power < *(p+i))
		{
			power = *(p+i);
			max_index = i;
		}
	}

	//busco el minimo
	power = 4095;
	for (i = 0; i < 10; i++)
	{
		if (power > *(p+i))
		{
			power = *(p+i);
			min_index = i;
		}
	}

	//descarto extremos
	j = 0;
	if (min_index == max_index)	//revisar min_index != max_index
		ii = 9;
	else
		ii = 10;

	for (i = 0; i < ii; i++)
	{
		if ((j != min_index) && (j != max_index))
		{
			*(p+j) = *(p+i);
			j++;
		}
	}

	power = MAFilter8 (p);
	return power;
}

void ShowPower (char * pstr, unsigned short pi, unsigned int e_acum_hours, unsigned int e_acum_secs)
{
	unsigned short power_int, power_dec;
	unsigned short wh_int, wh_dec;
	float fcalc = 1.0;

	fcalc = pi * KW;
	power_int = (unsigned short) fcalc;
	fcalc = fcalc - power_int;
	fcalc = fcalc * 100;
	power_dec = (unsigned short) fcalc;

	fcalc = (e_acum_hours + e_acum_secs / 1800) * KW;
	wh_int = (unsigned short) fcalc;
	fcalc = fcalc - wh_int;
	fcalc = fcalc * 10;
	wh_dec = (unsigned short) fcalc;

	sprintf(pstr, "pi: %3d.%02d wh: %d.%01d\r\n", power_int, power_dec, wh_int, wh_dec);

}
