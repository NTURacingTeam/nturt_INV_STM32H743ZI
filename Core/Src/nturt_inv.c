#include "nturt_inv.h"


int16_t maxint16(int16_t a,int16_t b)
{
  return a>b?a:b;
}

float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(in_value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

bool nturt_inv_toggle_and_get_run_state(NTURT_INV_TypeDef *handle)
{
		handle->nturt_main_run = !handle->nturt_main_run;
		return handle->nturt_main_run;
}

float nturt_inv_MCU_TemperatureCalculate(NTURT_INV_TypeDef *handle, uint16_t ts_data)
{
  return MCU_MapValue(ts_data, handle->MCU_T.cal_par_1, handle->MCU_T.cal_par_2, TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP);
  return 1;
}

bool nturt_inv_init_2(NTURT_INV_TypeDef *handle)
{
	if (handle == NULL) return false;
	HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_D12_GPIO_Port,LED_D12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_TIM_GPIO_Port,LED_TIM_Pin,GPIO_PIN_RESET);
  //Get temperature sensor calibration data
  /* 0x1FF1E820 Calibration ADC value at 30 °C = 0x2fc0, 12224 */
  handle->MCU_T.cal_par_1 = (float) *(uint16_t*) (TEMPSENSOR_CAL1_ADDR);
  /* 0x1FF1E840 Calibration ADC value at 110 °C = 0x3cb4, 15540 */
  handle->MCU_T.cal_par_2 = (float) *(uint16_t*) (TEMPSENSOR_CAL2_ADDR);

  //Generate MCU conversion table
  for (size_t i = 0; i < 1024; i++)
  {
    handle->MCU_T.adc_to_MCU_T_lut[i] = (int16_t) roundf(nturt_inv_MCU_TemperatureCalculate(handle, i<<6)*10);
  }
	return true;
}

bool nturt_inv_init_1(NTURT_INV_TypeDef *handle){
	if (handle == NULL) return false;

	return true;
}
void nturt_inv_update_encoder_angle_now(NTURT_INV_TypeDef *handle, uint16_t* val_arr, float_t *rad_pll)
{    
    int16_t _sv = val_arr[0]-val_arr[1];
    int16_t _cv = val_arr[2]-val_arr[3];
    
    handle->angle_encoder.angle_now = nturt_inv_mc_normalize_angle(_atan2(_sv,_cv));  
}

void nturt_inv_mc_update_all(NTURT_INV_TypeDef *handle){
	handle->mc.pole_pairs = handle->angle_encoder.pole_pairs;
	handle->mc.enc_dir = handle->angle_encoder.enc_dir;
	handle->mc.period = handle->angle_encoder.period;
	handle->mc.voltage_power_supply = handle->DCbus.voltage_power_supply;
	handle->mc.zero_electric_angle = handle->angle_encoder.zero_electric_angle;
}
