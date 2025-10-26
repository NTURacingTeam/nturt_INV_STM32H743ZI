/*
 * motor_control.h
 *
 *  Created on: Jun 17, 2023
 *      Author: hht
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include "stm32h7xx_hal.h"
#include <math.h>
#include "foc_utils.h"


#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

typedef struct {
  float zero_electric_angle; 
  int pole_pairs; // motor pole pairs / encoder pole pairs
  int enc_dir; // 1 for normal encoder mounting direction, -1 for reverse
  float voltage_power_supply; // should update every time
  int period; // period for the PWM 
} MotorControlConfig_TypeDef;

float nturt_inv_mc_normalize_angle(float angle);
float nturt_inv_mc_electricalAngle(MotorControlConfig_TypeDef* handle, float shaft_angle, int pole_pairs);
void nturt_inv_mc_setPwm(MotorControlConfig_TypeDef* handle, float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE);
void nturt_inv_mc_set_phase_volt(MotorControlConfig_TypeDef* handle, float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE,float Va,float Vb,float Vc);
void nturt_inv_mc_cal_Idq(float* current_phase, float angle_el, float* Id, float* Iq);


#endif /* INC_MOTOR_CONTROL_H_ */
