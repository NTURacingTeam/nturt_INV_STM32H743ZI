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

#include <stdbool.h>

/* Constants */
#define NTURT_INV_VERSION 1


typedef struct {
	/// for main loop run once very two timer interrupt
	bool nturt_main_run;

}	NTURT_INV_TypeDef;


bool nturt_inv_toggle_and_get_run_state(NTURT_INV_TypeDef *handle);


int16_t maxint16(int16_t a,int16_t b);
float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max);
float MCU_TemperatureCalculate(uint16_t ts_data);
void Enter_ERROR_State(INV_Errortypedef error);
void Config_Fdcan1(void);
void CAN_Send_State(uint16_t DCV, int16_t DCA);
void CAN_Send_Status(uint16_t report_status,int16_t FB_Torque,int16_t Speed);
void CAN_Send_Temp(uint16_t ADC_arr[6]);
void CAN_Send_Heartbeat(void);
void CAN_Send_Perameter(void);
void set_time (uint8_t hr, uint8_t min, uint8_t sec);
void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day);



#ifdef __cplusplus
}
#endif

#endif /* NTURT_INV_H */