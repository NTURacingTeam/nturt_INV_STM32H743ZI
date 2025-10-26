#ifndef NTURT_INV_H
#define NTURT_INV_H

#ifdef __cplusplus
extern "C" {
#endif

// these are form old main
#include "motor_control.h"
#include "sincos_encoder.h"
#include "config.h"
#include "canid.h"

#include "foc_utils.h"
#include "PLL/PLL.h"

#include <math.h>
// #include "usbd_cdc_if.h"
#include "string.h"
// #include "as5048a.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "current_sense.h"
#include "inverter_state.h"
#include "logger.h"
// #include "stdio.h"
// #include "stm32h7xx_hal_tim_ex.h"
#include "time.h"
#include "arm_math.h"
#include "main.h"

#include <stdbool.h>

/* Constants */
#define NTURT_INV_VERSION 1

typedef struct {
	float angle_now; // rad
	uint16_t enc_index;
	uint16_t enc_sum;
	uint8_t enc_buf[ENC_TIME];
	int enc_dir; // 1 for normal encoder mounting direction, -1 for reverse
	float zero_electric_angle;
	float shaft_angle;
	float voltage_powersupply;
	int period; // period for the PWM
	int pole_pairs; // motor pole pairs / encoder pole pairs
} NTURT_INV_Angle_Encoder_TypeDef;

typedef struct {
	const int16_t adc_to_SIC_T_lut[1024];
	int16_t U;
	int16_t V;
	int16_t W;
	int16_t max;
} SIC_Temperature_TypeDef;

typedef struct {
	int16_t adc_to_Mot_T_lut[1024];
	int16_t T;
	float cal_par_I; // the diode current
} Mot_Temperature_TypeDef;

typedef struct {
	int16_t T;
	int16_t adc_to_MCU_T_lut[1024];
	float cal_par_1;
	float cal_par_2;
} MCU_Temperature_TypeDef;

typedef struct {
	FDCAN_TxHeaderTypeDef HeartBeatHeader;
	FDCAN_TxHeaderTypeDef TempHeader;
	FDCAN_TxHeaderTypeDef StateHeader;
	FDCAN_TxHeaderTypeDef StatusHeader;
	FDCAN_TxHeaderTypeDef PerameterHeader;
	FDCAN_TxHeaderTypeDef RxHeader1;
} NTURT_INV_CAN_TypeDef;

typedef struct {
	uint16_t report_DCV;
	uint16_t report_DCA;
} NTURT_INV_DCbus_TypeDef;

typedef struct {
	float V_max;
} NTURT_INV_PID_TypeDef;

typedef struct {
	/// for main loop run once every two timer interrupt
	bool nturt_main_run;
	NTURT_INV_Angle_Encoder_TypeDef angle_encoder;
	SIC_Temperature_TypeDef SIC_T;
	Mot_Temperature_TypeDef Mot_T;
	MCU_Temperature_TypeDef MCU_T;
	INV_Statustypedef inv_state;
	INV_Errortypedef err_state;
	NTURT_INV_CAN_TypeDef can;
	NTURT_INV_DCbus_TypeDef DCbus;
	NTURT_INV_PID_TypeDef PID;
}	NTURT_INV_TypeDef;

int16_t maxint16(int16_t a,int16_t b);
float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max);

bool nturt_inv_toggle_and_get_run_state(NTURT_INV_TypeDef *handle);
float nturt_inv_MCU_TemperatureCalculate(NTURT_INV_TypeDef *handle, uint16_t ts_data);

void Enter_ERROR_State(INV_Errortypedef error);
void Config_Fdcan1(void);
void CAN_Send_State(uint16_t DCV, int16_t DCA);
void CAN_Send_Status(uint16_t report_status,int16_t FB_Torque,int16_t Speed);
void CAN_Send_Temp(uint16_t ADC_arr[6]);
void CAN_Send_Heartbeat(void);
void CAN_Send_Perameter(void);
void set_time (uint8_t hr, uint8_t min, uint8_t sec);
void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day);

bool nturt_inv_init_1(NTURT_INV_TypeDef *handle);
bool nturt_inv_init_2(NTURT_INV_TypeDef *handle);


void nturt_inv_update_encoder_angle_now(NTURT_INV_TypeDef *handle, uint16_t* val_arr, float_t *rad_pll);


#ifdef __cplusplus
}
#endif

#endif /* NTURT_INV_H */