/**
  ******************************************************************************
  * @file    foc_lib.c
  * @brief   This file provides FOC library         
  ******************************************************************************
  */

#include "foc_lib.h"

/* Variables */
// Timer
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;

// Flags
int8_t     Command = 'h', overcurrent, overvolt, undervolt, status_flag, overload, sensor_fault;

// Power
int16_t    scale_power;
int16_t    power, power_min, power_over, power_max, power_nominal, power_min, power_fault;

// Voltage
ABC_System u_Sabc; 			 // stator voltage in a,b,c	
ab_System  u_SAlphaBeta; // stator voltage in alfa,beta
dq_System  u_Sdq;	 			 // stator voltage in d,q
dq_System  u_Sdq_filt;
int16_t    U_desired, U_ref_angle;

// Current
ABC_System i_Sabc_adc;   // ADC current measurements in a,b,c
ABC_System i_Sabc; 			 // stator current in a,b,c	
ab_System  i_SAlphaBeta; // stator current in alfa,beta
dq_System  i_Sdq;	 			 // stator current in d,q
dq_System  i_Sdq_desired = {0, 0}; // desired stator current d,q coordinates
int16_t    i_sdq_max, i_sdq_max_cont, i_sd_alignment, i_sd_fault;
int16_t    scale_current;
PI_params  current_PI_params; 

// Speed
int16_t    omega_max, omega_ramp, scale_omega;
PI_params  omega_PI_params; 
int16_t    Enc_in, Enc_in_old, delta_theta;
int16_t    omega_desired, omega_actual;

// PI controller
int32_t 	 err_int, err_int_q_is, err_int_d_is, err_curr;
int32_t 	 out = 0, high_lim, low_lim;
int16_t 	 pi_p_gain_is, pi_i_gain_is, pi_p_gain_omega, pi_i_gain_omega;

// FOC
uint8_t 	 sector;
int16_t 	 dir_step = 0, step, theta_el_deg = 0;
int32_t 	 t0, t1, t2, switch_time_A, switch_time_B, switch_time_C;
trig_angle sinCos_theta_el;

// Sine table 0 to -PI/2 scaled to 0 to -32768; (0 to -1 fractional) - 256+1 samples
const int16_t sin_table[257] = { 
	
				 0,-201,-402,-603,-804,-1005,-1206,-1407,-1607,-1808,
        -2009,-2210,-2410,-2611,-2811,-3011,-3211,-3411,-3611,-3811,
        -4011,-4210,-4409,-4609,-4808,-5006,-5205,-5403,-5602,-5800,
        -5997,-6195,-6392,-6589,-6786,-6983,-7179,-7375,-7571,-7766,
        -7961,-8156,-8351,-8545,-8739,-8933,-9126,-9319,-9512,-9704,
        -9896,-10087,-10278,-10469,-10659,-10849,-11039,-11228,-11416,-11605,
        -11793,-11980,-12167,-12353,-12539,-12725,-12910,-13094,-13278,-13462,
        -13645,-13828,-14010,-14191,-14372,-14552,-14732,-14912,-15090,-15269,
        -15446,-15623,-15800,-15976,-16151,-16325,-16499,-16673,-16846,-17018,
        -17189,-17360,-17530,-17700,-17869,-18037,-18204,-18371,-18537,-18703,
        -18868,-19032,-19195,-19358,-19519,-19681,-19841,-20001,-20159,-20318,
        -20475,-20631,-20787,-20942,-21097,-21250,-21403,-21555,-21706,-21856,
        -22005,-22154,-22301,-22448,-22594,-22740,-22884,-23027,-23170,-23312,
        -23453,-23593,-23732,-23870,-24007,-24144,-24279,-24414,-24547,-24680,
        -24812,-24943,-25073,-25201,-25330,-25457,-25583,-25708,-25832,-25955,
        -26077,-26199,-26319,-26438,-26557,-26674,-26790,-26905,-27020,-27133,
				-27245,-27356,-27466,-27576,-27684,-27791,-27897,-28002,-28106,-28208,
        -28310,-28411,-28511,-28609,-28707,-28803,-28898,-28993,-29086,-29178,
        -29269,-29359,-29447,-29535,-29621,-29707,-29791,-29874,-29956,-30037,
        -30117,-30196,-30273,-30350,-30425,-30499,-30572,-30644,-30714,-30784,
        -30852,-30919,-30985,-31050,-31114,-31176,-31237,-31298,-31357,-31414,
        -31471,-31526,-31581,-31634,-31685,-31736,-31785,-31834,-31881,-31927,
        -31971,-32015,-32057,-32098,-32138,-32176,-32214,-32250,-32285,-32319,
        -32351,-32383,-32413,-32442,-32469,-32496,-32521,-32545,-32568,-32589,
        -32610,-32629,-32647,-32663,-32679,-32693,-32706,-32718,-32728,-32737,
        -32745,-32752,-32758,-32762,-32765,-32767,-32767 
};        

/* Functions */
void Park(ab_System *p_alphaBeta, dq_System *p_dq, trig_angle *pSinCos) { // ab->dq

	int32_t tmp, tmp1, tmp2;
	
	tmp1 = p_alphaBeta->a_axis * (int32_t)pSinCos->cos;
  tmp2 = p_alphaBeta->b_axis * (int32_t)pSinCos->sin;
  tmp = (tmp1 + tmp2) >> 15;
	
	if (tmp > INT16_MAX) p_dq->d_axis = INT16_MAX;
  else if (tmp < -INT16_MAX) p_dq->d_axis = -INT16_MAX;
  else p_dq->d_axis = tmp;

	tmp1 = p_alphaBeta->b_axis * (int32_t)pSinCos->cos;
  tmp2 = p_alphaBeta->a_axis * (int32_t)pSinCos->sin;
  tmp = (tmp1 - tmp2) >> 15;

  if (tmp > INT16_MAX) p_dq->q_axis = INT16_MAX;
  else if (tmp < -INT16_MAX) p_dq->q_axis = -INT16_MAX;
  else p_dq->q_axis = tmp;
}

void invPark(ab_System *p_alphaBeta, dq_System *p_dq, trig_angle *pSinCos) { // dq->ab

	int32_t tmp, tmp1, tmp2;
	
	tmp1 = p_dq->d_axis * (int32_t)pSinCos->cos;
  tmp2 = p_dq->q_axis * (int32_t)pSinCos->sin;
  tmp = (int16_t)((tmp1 - tmp2) >> 15);
	
  if (tmp > INT16_MAX) p_alphaBeta->a_axis = INT16_MAX;
  else if (tmp < -INT16_MAX) p_alphaBeta->a_axis = -INT16_MAX;
	else p_alphaBeta->a_axis = tmp;	

  tmp1 = p_dq->d_axis * (int32_t)pSinCos->sin;
  tmp2 = p_dq->q_axis * (int32_t)pSinCos->cos;
  tmp = (int16_t)((tmp1 + tmp2) >> 15);
	
  if (tmp > INT16_MAX) p_alphaBeta->b_axis = INT16_MAX;
  else if (tmp < -INT16_MAX) p_alphaBeta->b_axis = -INT16_MAX;
  else p_alphaBeta->b_axis = tmp;	
}

void Clarke(ab_System *p_alphaBeta, ABC_System *p_abc) { // abc->ab

	int32_t tmp_a, tmp_b, tmp;	

	p_alphaBeta->a_axis = p_abc->phaseA;

  tmp_a = ONE_DIV_SQRT3 * (int32_t)p_abc->phaseA;
  tmp_b = ONE_DIV_SQRT3 * (int32_t)p_abc->phaseB;
  tmp = (tmp_a + 2*tmp_b) >> 15;
  if (tmp > INT16_MAX) p_alphaBeta->b_axis = INT16_MAX;
  else if (tmp < -INT16_MAX) p_alphaBeta->b_axis = -INT16_MAX;
  else p_alphaBeta->b_axis = (int16_t)tmp;
}
	
int16_t PI_reg(int16_t act_val, int16_t des_val, PI_params* pi_params) {
	
	int32_t Err, Integr32, Prop32, High, Low, out;  
	int64_t r, g;
  
	Err = (int32_t)des_val - (int32_t)act_val;	
	
	High = (int32_t)pi_params->pi_out_max << 16;
	Low = (int32_t)pi_params->pi_out_min << 16;
	
	Prop32 = (Err * pi_params->K_p);
	Integr32 = (Err * pi_params->K_i);		   

	r = (int64_t)pi_params->err_int + (int64_t)Integr32; // integral
  
	// limit integral
	if(r > High) pi_params->err_int = High; 
	else pi_params->err_int = (r < Low) ? Low : (int32_t)r;
	
	g = r + (int64_t)Prop32; // pi out
  
	// limit output
	if(g > High) out = High;
	else out = (g < Low) ? Low : (int32_t)g; 
    
	return (int16_t)(out>>16);
}

uint8_t SVPWM(ab_System *p_alphaBeta, ABC_System *p_abc) {

	int16_t x, y, z, yz;    
	
	x = p_alphaBeta->b_axis;
	yz = mult_r(p_alphaBeta->a_axis, HALF_SQRT3);
	y = (p_alphaBeta->b_axis >> 1) + yz;
	z = (p_alphaBeta->b_axis >> 1) - yz;
	
	if(y < 0) {
		if (z < 0) {
			p_abc->phaseA = FRAC16(0.5) + y/2 - z/2;
			p_abc->phaseB = p_abc->phaseA + z;
			p_abc->phaseC = p_abc->phaseA - y;                
			return 5;
		} 
		else {
			if (x > 0) {
				p_abc->phaseA = FRAC16(0.5) - x/2 + y/2;
				p_abc->phaseC = p_abc->phaseA - y;                   
				p_abc->phaseB = p_abc->phaseC + x;      
				return 3;
			} 
			else {
				p_abc->phaseA = FRAC16(0.5) + x/2 - z/2;
				p_abc->phaseB = p_abc->phaseA + z;
				p_abc->phaseC = p_abc->phaseB - x;                
				return 4;    
			}
		}
	} 
	else {
		if (z < 0) {
			if (x > 0) {
				p_abc->phaseA = FRAC16(0.5) + x/2 - z/2;
				p_abc->phaseB = p_abc->phaseA + z;
				p_abc->phaseC = p_abc->phaseB - x;
				return 1;
			} 
			else {
				p_abc->phaseA = FRAC16(0.5) - x/2 + y/2;
				p_abc->phaseC = p_abc -> phaseA - y;                   
				p_abc->phaseB = p_abc -> phaseC + x;      
				return 6;
			}
		} 
		else {
			p_abc->phaseA = FRAC16(0.5) + y/2 - z/2;
			p_abc->phaseB = p_abc->phaseA + z;
			p_abc->phaseC = p_abc->phaseA - y;                
			return 2;
		}
	}
}

int16_t	form_SinCos(int16_t value, int16_t *psinTable) {

	if (value >= 0) 
		return (value > DIVIDE_2_F16 ? -psinTable[(2*SIN_LENGTH_TABLE - (value >> SIN_INDEX_SHIFT))] : -psinTable[(value >> SIN_INDEX_SHIFT)]);
	else 
		return (value < -DIVIDE_2_F16 ? psinTable[(2*SIN_LENGTH_TABLE + (value >> SIN_INDEX_SHIFT))] : psinTable[(-(value >> SIN_INDEX_SHIFT))]);
}		

int16_t mult_r(int16_t x, int16_t y) { 
	return ((int32_t)x * y) >> 15; 
}

int SQRT(int input) {
	
	if ((int)input == 0 || (int)input == 1) return input;

	int start = 1, end = input, result;
	while (start <= end) {
		int mid = start + (end - start) / 2;

		// Check if mid is sqrt
		if (mid == input / mid) return mid;
		
		// If mid*mid < input
		if (mid <= input / mid) {
				start = mid + 1;
				result = mid;
		} else {
				end = mid - 1;
		}
	}
	return result;
}
