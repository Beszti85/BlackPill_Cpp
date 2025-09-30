/*
 * pc_data_readwrite.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_data_readwrite.h"
#include "bme280.h"
#include "flash.h"

extern BME280_PhysValues_t BME280_PhysicalValues;
extern FLASH_Handler_t FlashHandler;
extern float ADC_Voltage[5u];

uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer )
{
  // return value is the length of the response
  uint8_t retval = 0u;

  switch (readId)
  {
    // Read board name
    case BOARD_ID:
      memcpy(ptrTxBuffer, "NucleoL432KC_DevBoard", sizeof("NucleoL432KC_DevBoard"));
      retval = sizeof("NucleoL432KC_DevBoard");
      break;
    // Read BME280 physical values
    case BME280_PHYSICAL_VALUES:
      memcpy(ptrTxBuffer, &BME280_PhysicalValues.Temperature, sizeof(BME280_PhysicalValues.Temperature));
      ptrTxBuffer += sizeof(BME280_PhysicalValues.Temperature);
      memcpy(ptrTxBuffer, &BME280_PhysicalValues.Humidity, sizeof(BME280_PhysicalValues.Humidity));
      ptrTxBuffer += sizeof(BME280_PhysicalValues.Humidity);
      memcpy(ptrTxBuffer, &BME280_PhysicalValues.Pressure, sizeof(BME280_PhysicalValues.Pressure));
      ptrTxBuffer += sizeof(BME280_PhysicalValues.Pressure);
      retval = 12u;
      break;
    // Read ADC data
    case ADC_PHY_VALUES:
      memcpy(ptrTxBuffer, ADC_Voltage, sizeof(ADC_Voltage));
      retval = 20u;
      break;
    // Read Humidity BME280
    case FLASH_ID:
      memcpy(ptrTxBuffer, &FlashHandler.DetectedFlash, sizeof(FlashHandler.DetectedFlash));
      retval = 1u;
      break;
    default:
      break;
  }

  return retval;
}
