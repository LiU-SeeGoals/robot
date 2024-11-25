#include "battery.h"

/* Private includes */
#include "log.h"

/* Private defines */
//...

/* Private variables */
static LOG_Module internal_log_mod;
static ADC_HandleTypeDef *hadc;

/* Private functions declarations */
//...

/*
 * Public functions implementations
 */
void BATTERY_Init(ADC_HandleTypeDef *adc_handle) {
  hadc = adc_handle;
  LOG_InitModule(&internal_log_mod, "BATTERY", LOG_LEVEL_DEBUG);
  //HAL_ADC_Start_DMA(hadc,(uint32_t*)&value_adc,1);
  //HAL_ADC_Start(hadc);
  //HAL_ADC_Start_DMA(hadc, (uint32_t *)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
}

uint32_t BATTERY_GetLevel() {
  //for (int i = 0; i < hadc->Init.NbrOfConversion; ++i) {
  if (HAL_ADC_Start(hadc) != HAL_OK) {
    Error_Handler();
  }
  for (int i = 0; i < 6; ++i) {
    uint32_t value_adc = 0;
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
      value_adc = HAL_ADC_GetValue(hadc);
      LOG_DEBUG("Value ADC (channel %d): %d\r\n", i, (value_adc*3300) / 0xFFFF);
    } else {
      LOG_DEBUG("Couldn't poll battery for channel %d\r\n", i);
    }
  }
  if (HAL_ADC_Stop(hadc) != HAL_OK) {
    Error_Handler();
  }
}


/*
 * Private functions implementations
 */
