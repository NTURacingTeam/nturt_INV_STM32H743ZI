/*
 * mortor_control.c
 *
 *  Created on: Jun 17, 2023
 *      Author: hht
 */

#include "motor_control.h"
#include "config.h"
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#define _HIGH_IMPEDANCE 0

float nturt_inv_mc_normalize_angle(float angle){
  float a = fmod(angle, 2*M_PI);  
  return a >= 0 ? a : (a + 2*M_PI);
}

float nturt_inv_mc_electricalAngle(MotorControlConfig_TypeDef* handle, float shaft_angle, int pole_pairs) {
  return nturt_inv_mc_normalize_angle(((float)(handle->enc_dir * pole_pairs)*shaft_angle)-handle->zero_electric_angle);
}

void nturt_inv_mc_setPwm(MotorControlConfig_TypeDef* handle, float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE) {

  float dc_a = _constrain(Ua , 0.0f , 1.0f );
	float dc_b = _constrain(Ub , 0.0f , 1.0f );
	float dc_c = _constrain(Uc , 0.0f , 1.0f );

	//写入PWM到PWM 0 1 2 通道
	TIM_BASE->CCR1 = (uint32_t) roundf(dc_a*handle->period);
	TIM_BASE->CCR2 = (uint32_t) roundf(dc_b*handle->period);
	TIM_BASE->CCR3 = (uint32_t) roundf(dc_c*handle->period);

}

// should update voltage_power_supply every time before calling this function
void nturt_inv_mc_set_phase_volt(MotorControlConfig_TypeDef* handle, float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE,float Va,float Vb,float Vc) {
  angle_el = nturt_inv_mc_normalize_angle(angle_el);
  
  #ifdef MIDDLE_CLAMP
  float sa;
  float ca;
  _sincos(angle_el,&sa,&ca);
  float Ualpha =  ca * Ud - sa * Uq;  
  float Ubeta =  sa * Ud + ca * Uq;
  float Ua = Ualpha;
  float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
  float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;
  Ua -= Va;
  Ub -= Vb;
  Uc -= Vc;
  float Da = _constrain((Ua / handle->voltage_power_supply+1)/2,0.0f,1.0f);
  float Db = _constrain((Ub / handle->voltage_power_supply+1)/2,0.0f,1.0f);
  float Dc = _constrain((Uc / handle->voltage_power_supply+1)/2,0.0f,1.0f);
  #ifdef SVPWM
  float center = 0.5f;
  // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
  // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
  // Midpoint Clamp
  float Dmin = fminf(Da, fminf(Db, Dc));
  float Dmax = fmaxf(Da, fmaxf(Db, Dc));
  center -= (Dmax+Dmin) / 2;
  Da += center;
  Db += center;
  Dc += center;
  #endif
  #else
  angle_el =  nturt_inv_mc_normalize_angle (angle_el+M_PI/2);
  int sector = floor(angle_el / M_PI*3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * M_PI/3 - angle_el) * Uq / handle->voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * M_PI/3) * Uq / handle->voltage_power_supply;
  float T0 = 1 - T1 - T2;


  float Ta, Tb, Tc;
  switch (sector)
  {
    case 1:
      Ta = T1 + T2 + T0 / 2;
      Tb = T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 2:
      Ta = T1 + T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 3:
      Ta = T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T2 + T0 / 2;
      break;
    case 4:
      Ta = T0 / 2;
      Tb = T1 + T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 5:
      Ta = T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 6:
      Ta = T1 + T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T0 / 2;
      break;
    default:
      Ta = 0;
      Tb = 0;
      Tc = 0;
  }
  // 克拉克逆变换
  float Ua = Ta * handle->voltage_power_supply;
  float Ub = Tb * handle->voltage_power_supply;
  float Uc = Tc * handle->voltage_power_supply;
  #endif
  // Ua = Da * handle->voltage_power_supply;
  // Ub = Db * handle->voltage_power_supply;
  // Uc = Dc * handle->voltage_power_supply;
  nturt_inv_mc_setPwm(handle, Da,Db,Dc,TIM_BASE);
}

void nturt_inv_mc_cal_Idq(float* current_phase, float angle_el, float* Id, float* Iq)
{
	angle_el = nturt_inv_mc_normalize_angle(angle_el);
  float mid_current = (current_phase[0] + current_phase[1] + current_phase[2]) / 3.0f;
  // 将三相电流转换为两相电流
  float a = current_phase[0] - mid_current;
  float b = current_phase[1] - mid_current;
	float I_alpha=a;
  float I_beta=_1_SQRT3*(2*b+a);
//	 float Iq=-sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
//	 float Id=cos(angle_el)*I_alpha+sin(angle_el)*I_beta;
	*Iq = -sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
  *Id = cos(angle_el)*I_alpha+sin(angle_el)*I_beta;
}
