#include "MS4525DO.h"


uint8_t measure_ms4525do(I2C_HandleTypeDef* i2c, float* pressure, float* temperature)
{
	uint8_t _status;
	uint8_t Press_H;
	uint8_t Press_L;
	uint8_t Temp_H;
	uint8_t Temp_L;
	uint16_t dp_raw;
	uint16_t dT_raw;

	uint8_t data[4];
	HAL_I2C_Mem_Read(i2c, I2C_ADDRESS_MS4525DO, 0x00, 1, data, 4, HAL_MAX_DELAY);




	Press_H = data[0];
	Press_L = data[1];
	Temp_H = data[2];
	Temp_L = data[3];

	 _status = (Press_H >> 6) & 0x03;
	Press_H = Press_H & 0x3f;
	dp_raw = (((uint16_t)Press_H) << 8) | Press_L;

	Temp_L = (Temp_L >> 5);
	dT_raw= (((uint16_t)Temp_H) << 3) | Temp_L;



	const float P_max = 1.0;
	const float P_min = - P_max;
	const float PSI_to_Pa = 6894.757f;

	float diff_press_PSI  = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
	*pressure  = diff_press_PSI * PSI_to_Pa;

	*temperature = ((200.0f * dT_raw) / 2047) - 50;


	return _status;
}


