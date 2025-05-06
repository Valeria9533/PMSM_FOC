/**
  ******************************************************************************
  * @file    foc_lib.h
  * @brief   This file contains data structures and 
  * 				 function prototypes of the FOC library         
  ******************************************************************************
  */

#ifndef FOC_LIB_H
#define FOC_LIB_H

#include "main.h"
#include "math.h"
#include "def.h"

typedef struct {
    int16_t phaseA;
    int16_t phaseB;
    int16_t phaseC;
} ABC_System;

typedef struct {
    int16_t a_axis;
    int16_t b_axis;
} ab_System;

typedef struct {
    int16_t d_axis;
    int16_t q_axis;
} dq_System;

typedef struct {
		int16_t K_p; 
    int16_t K_i;			
    int16_t pi_out_max;	
    int16_t pi_out_min;	
		int32_t err_int;
} PI_params;

typedef struct {
    int16_t sin;
    int16_t cos;
} trig_angle;

void Park(ab_System *p_alphaBeta, dq_System *p_dq, trig_angle *pSinCos);
void invPark(ab_System *p_alphaBeta, dq_System *p_dq, trig_angle *pSinCos);
void Clarke(ab_System *p_alphaBeta, ABC_System *p_abc);
uint8_t SVPWM(ab_System *p_alphaBeta, ABC_System *p_abc);
int16_t PI_reg(int16_t act_val, int16_t des_val, PI_params* pi_params);
void motor_control(void);
void motor_control_add(void);
void PWM_Enable(void);
void PWM_Disable(void);
void PWM_Load(ABC_System *p_abc);
void check_faults(void);
int SQRT(int);
int16_t mult_r(int16_t x, int16_t y);
int16_t	form_SinCos(int16_t value, int16_t *psinTable);

#endif
