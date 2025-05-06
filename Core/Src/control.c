/**
  ******************************************************************************
  * @file    control.c
  * @brief   This file provides FOC algorithm         
  ******************************************************************************
  */

#include "main.h"
#include "foc_lib.h"

/* Variables */
// ADC
extern uint8_t 		adc_init;

// Timer
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;

// Flags
extern int8_t     Command, overcurrent, overvolt, undervolt, status_flag, overload, sensor_fault;

// Power
extern int16_t    scale_power;
extern int16_t    power, power_min, power_over, power_max, power_nominal, power_min, power_fault;

// Voltage
extern ABC_System u_Sabc; 			 // stator voltage in a,b,c	
extern ab_System  u_SAlphaBeta;  // stator voltage in alfa,beta
extern dq_System  u_Sdq;	 			 // stator voltage in d,q
extern dq_System  u_Sdq_filt;
extern int16_t    U_desired;

// Current
extern ABC_System i_Sabc_adc;    // ADC current measurements in a,b,c
extern ABC_System i_Sabc; 			 // stator current in a,b,c	
extern ab_System  i_SAlphaBeta;  // stator current in alfa,beta
extern dq_System  i_Sdq;	 			 // stator current in d,q
extern dq_System  i_Sdq_desired; // desired stator current d,q coordinates
extern int16_t    i_sdq_max, i_sdq_max_cont, i_sd_alignment, i_sd_fault;
extern int16_t    scale_current;
extern PI_params  current_PI_params; 

// Speed
extern int16_t    omega_max, omega_ramp, scale_omega;
extern PI_params  omega_PI_params; 
extern int16_t    Enc_in, Enc_in_old, delta_theta;
int16_t						delta_theta_filt;
extern int16_t    omega_desired, omega_actual;

// PI controller
extern int32_t 		err_int, err_int_q_is, err_int_d_is, err_curr, out;
extern int16_t 		pi_p_gain_is, pi_i_gain_is, pi_p_gain_omega, pi_i_gain_omega;

// FOC
extern uint8_t 		sector;
extern int16_t 		dir_step, theta_el_deg, step;
extern trig_angle sinCos_theta_el;
extern const int16_t sin_table[257];

int16_t TIM1_ADC_1_S, TIM1_ADC_1_E, TIM1_ADC_2_S, TIM1_ADC_2_E, TIM1_ADC_1, TIM1_ADC_2;

/* Functions */
void motor_control(void) {

	static int16_t t_i;
	
	TIM1_ADC_1_S = TIM1->CNT;
	
	// Update PI parameters 
	current_PI_params.K_p = pi_p_gain_is;
	current_PI_params.K_i = pi_i_gain_is;
	current_PI_params.pi_out_max = FRAC16(0.9);
	current_PI_params.pi_out_min = FRAC16(-0.9);	
	
	omega_PI_params.K_p = pi_p_gain_omega;
	omega_PI_params.K_i = pi_i_gain_omega;
	omega_PI_params.pi_out_max = i_sdq_max;
	omega_PI_params.pi_out_min = -i_sdq_max;
	
	// Stop 
	if (Command == 'h') { PWM_Disable(); theta_el_deg = 0; }
	
	// Run if no errors
	if ((Command != 'h') && (status_flag == 0)) { 
		PWM_Enable();
	}
	
	// Setting current loop
	if (Command == 'i') {	
		theta_el_deg = 0;
		if (((t_i++) & 0x40) != 0) i_Sdq_desired.q_axis = i_sd_alignment;
		else i_Sdq_desired.q_axis = 0;
	}	
	
	// Torque control 
	if ((Command == 'n') && (dir_step != 0)) {	

		step += dir_step;
		theta_el_deg = step;
		
		GPIOD->ODR &= ~(1<<3); // run led
	}
	
	// Speed control
	if (Command == 'o') {		
		
		theta_el_deg = Enc_in*2 *15;	
				
		if (omega_desired > omega_max) omega_desired = omega_max; 
		if (omega_desired < -omega_max) omega_desired = -omega_max;
		
		//U_required = PI_reg(omega_actual, omega_desired, &omega_PI_params);			

		i_Sdq_desired.q_axis = PI_reg(omega_actual, omega_desired, &omega_PI_params);	
		
		GPIOD->ODR &= ~(1<<3); // run led
	}
	
	// Sin/cos
	sinCos_theta_el.sin = form_SinCos(theta_el_deg, (int16_t*)&sin_table);
	sinCos_theta_el.cos = form_SinCos(theta_el_deg + DIVIDE_2_F16, (int16_t*)&sin_table);
	
	Clarke(&i_SAlphaBeta, &i_Sabc); 
	Park(&i_SAlphaBeta, &i_Sdq, &sinCos_theta_el); 

	// Set current integral error for q and d axis
	current_PI_params.err_int = err_int_q_is; 
	u_Sdq.q_axis = PI_reg(i_Sdq.q_axis, i_Sdq_desired.q_axis, &current_PI_params);
	err_int_q_is = current_PI_params.err_int;
	
	current_PI_params.err_int = err_int_d_is; 
	u_Sdq.d_axis = PI_reg(i_Sdq.d_axis, 0, &current_PI_params);
	err_int_d_is = current_PI_params.err_int;
	
	// Eliminate voltage ripple in q and d axis 	
	u_Sdq.q_axis = (((int32_t)u_Sdq.q_axis)<<15)/power;
	u_Sdq.d_axis = (((int32_t)u_Sdq.d_axis)<<15)/power;

	invPark(&u_SAlphaBeta, &u_Sdq, &sinCos_theta_el);
	sector = SVPWM(&u_SAlphaBeta, &u_Sabc);		
	
	// Load values to TIM1 CC registers 
	PWM_Load(&u_Sabc);
	
	TIM1_ADC_1_E = TIM1->CNT;
	
	TIM1_ADC_1 = abs(TIM1_ADC_1_E - TIM1_ADC_1_S);
}

void motor_control_add(void) {

	TIM1_ADC_2_S = TIM1->CNT;
	
	// Setting current loop
	if (Command == 'i') theta_el_deg = 0;
	
	// Torque control 
	if ((Command == 'n') && (dir_step != 0)) theta_el_deg = step; 

	// Sin/cos
	sinCos_theta_el.sin = form_SinCos(theta_el_deg, (int16_t*)&sin_table);
	sinCos_theta_el.cos = form_SinCos(theta_el_deg + DIVIDE_2_F16, (int16_t*)&sin_table);
	
	invPark(&u_SAlphaBeta, &u_Sdq, &sinCos_theta_el);
	sector = SVPWM(&u_SAlphaBeta, &u_Sabc);		
	PWM_Load(&u_Sabc);
	
	FMSTR_Recorder();
	
	TIM1_ADC_2_E = TIM1->CNT;
	
	TIM1_ADC_2 = abs(TIM1_ADC_2_E - TIM1_ADC_2_S);
}

void get_speed_encoder(void) {
	
	// Send request to absolute encoder
	SPI2_NSS_SET;
	SPI2->DR = 0xAAAA;
	
	// Read answer from absolute encoder
	Enc_in = SPI2->DR - INT16_MAX - INT16_MAX/2;
	
	delta_theta = Enc_in - Enc_in_old;	
	Enc_in_old = Enc_in;
	
	if(abs(delta_theta) > 32000) delta_theta = 0;
	
	delta_theta_filt = delta_theta_filt*0.9f + delta_theta*0.1f;
	
	omega_actual = delta_theta_filt*100;
}

void check_faults(void) {

	if(adc_init == 1) { 
		// Check current	
		if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == SET) overcurrent = 1;
		else overcurrent = 0;
		
		// Check power
		if (power > POWER_FAULT) overvolt = 1;
		else overvolt = 0;
		if (power < POWER_MIN) undervolt = 1; 
		else undervolt = 0; 
		
		status_flag = overcurrent + overvolt + undervolt + overload + sensor_fault;	

		if (status_flag == 0) GPIOD->ODR |= (1<<4); // error led off
		else { 
			PWM_Disable(); 
			Command = 'h';		
			GPIOD->ODR &= ~(1<<4); // error led on
		}		
		
		// Limit current
		if (i_Sdq_desired.q_axis > i_sdq_max) i_Sdq_desired.q_axis = i_sdq_max;
		if (i_Sdq_desired.q_axis < -i_sdq_max) i_Sdq_desired.q_axis = -i_sdq_max;	
	}
	else { 
		// Reset fault while power up	
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);	
		adc_init = 1;
	}
}

void PWM_Load(ABC_System *p_abc) {
	TIM1->CCR1 = mult_r(u_Sabc.phaseA,TIM1->ARR); 
	TIM1->CCR2 = mult_r(u_Sabc.phaseB,TIM1->ARR);
	TIM1->CCR3 = mult_r(u_Sabc.phaseC,TIM1->ARR);
}

void PWM_Enable(void) {
	
	TIM1->CCER |= 0x1555;	
	TIM1->BDTR |= TIM_BDTR_MOE;
	
	TIM8->CCER |= (1<<0)|(1<<4)|(1<<5);	
	TIM8->BDTR |= TIM_BDTR_MOE;
}	

void PWM_Disable(void) {	
	
	TIM1->CCER &= ~0x1555;
	TIM1->BDTR |= TIM_BDTR_MOE;	

	current_PI_params.err_int = 0;
	err_int_d_is = err_int_q_is = 0;
	U_desired = 0;
	i_Sdq_desired.d_axis = i_Sdq_desired.q_axis = 0;
	u_Sdq.d_axis = u_Sdq.q_axis = 0;
	u_Sabc.phaseA = u_Sabc.phaseB = u_Sabc.phaseC = TIM1->ARR>>1;
	sector = 1;
	dir_step = 0;
	omega_desired = 0;
	
	GPIOD->ODR |= (1<<3); // run led off
}	
