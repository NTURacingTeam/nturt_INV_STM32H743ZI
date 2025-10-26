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