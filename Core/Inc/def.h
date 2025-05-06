/**
  ******************************************************************************
  * @file    def.h
  * @brief   This file contains system parameters defnition      
  ******************************************************************************
  */

#ifndef DEF_H
#define DEF_H

// FRAC type
#define FRAC32(x) ((int32_t)((x) < 1 ? ((x) >= -1 ? ((int32_t)((x)*0x80000000)) : ((int32_t)0x80000000)) : ((int32_t)0x7FFFFFFF)))
#define FRAC16(x) ((int16_t)((x) < 1 ? ((x) >= -1 ? ((int32_t)((x)*0x8000)) : ((int32_t)0x8000)) : ((int32_t)0x7FFF)))

// General constants
#define PI 						  3.14159265359
#define SQRT3           1.7320508
#define ONE_DIV_SQRT3   FRAC16(1/SQRT3)
#define HALF_SQRT3      FRAC16(SQRT3/2) 
#define ONE_DIV_POWER 	FRAC16(1/power)
#define POLE_PAIRS 			15
#define DIVIDE_2_F16 	  16384	
#define SIN_INDEX_SHIFT 6		
#define SIN_LENGTH_TABLE 256		

// Power
#define SCALE_POWER			135.3
#define POWER_FAULT			FRAC16(70.0/SCALE_POWER)
#define POWER_OVER			FRAC16(60.0/SCALE_POWER)
#define POWER_MAX				FRAC16(55.0/SCALE_POWER)
#define POWER_NOMINAL		FRAC16(48.0/SCALE_POWER)
#define POWER_MIN				FRAC16(18.0/SCALE_POWER)

// Current
#define SCALE_CURRENT		165.0
#define I_SDQ_MAX				FRAC16(10.0/SCALE_CURRENT) // maximal current vector 
#define I_SDQ_MAX_CONT	FRAC16(10.0/SCALE_CURRENT) // maximal current vector 
#define I_SD_ALIGNMENT	FRAC16(5.0/SCALE_CURRENT)
#define I_SD_FAULT			FRAC16(40.0*SCALE_CURRENT) // Fault current
#define PI_P_GAIN_IS    12000	  							 	   // stator current both axes PI regulator proportional gain 
#define PI_I_GAIN_IS    2000									 		 // stator current both axes PI regulator integral gain 

// Speed
#define SCALE_OMEGA			500.0
#define OMEGA_MAX	 			FRAC16(200.0/SCALE_OMEGA)  // maximal required speed
#define OMEGA_RAMP 			(OMEGA_MAX / 1000.0 + 0.5) // maximal ramp for speed 
#define PI_P_GAIN_OMEGA 8000 								       // omega both axes PI regulator proportional gain
#define PI_I_GAIN_OMEGA 62 								         // omega both axes PI regulator integral gain

#endif

