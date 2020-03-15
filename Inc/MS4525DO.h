#include "stm32f1xx_hal.h"

#define I2C_ADDRESS_MS4525DO (0x28 << 1) /**< 7-bit address =0x28. 8-bit is 0x50. Depends on the order code (this is for code "I") */
#define MS4525MinScaleCounts 1638
#define MS4525FullScaleCounts 14746
#define MS4525Span=MS4525FullScaleCounts-MS4525MinScaleCounts
#define MS4525ZeroCounts=(MS4525MinScaleCounts+MS4525FullScaleCounts)/2

uint8_t measure_ms4525do(I2C_HandleTypeDef* i2c, float* pressure, float* temperature);


