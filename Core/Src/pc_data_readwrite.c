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
extern float ADC_Voltage[7u];

// Static module variables
uint32_t PcExtFlashReadAddress = 0u;
uint32_t PcExtFlashReadLength = 0u;

uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer )
{
  // return value is the length of the response
  uint8_t retval = 0u;

  switch (readId)
  {
    // Read board name
    case BOARD_ID:
      memcpy(ptrTxBuffer, "BlackPillF411_DevBoard", sizeof("BlackPillF411_DevBoard"));
      retval = sizeof("BlackPillF411_DevBoard");
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
      retval = 28u;
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

// Data write handler
void PC_WriteDataHandler( uint8_t* ptrTxBuffer )
{
  uint8_t writeId = ptrTxBuffer[0];
  switch (writeId)
  {
    // Set flash read address
    case FLASH_CFG_WRITE:
      memcpy(&PcExtFlashReadAddress, ptrTxBuffer, sizeof(PcExtFlashReadAddress));
      memcpy(&PcExtFlashReadLength, ptrTxBuffer + sizeof(PcExtFlashReadAddress), sizeof(PcExtFlashReadLength));
      break;
    case LED_PWM:
      //memcpy(&TIM1_PwmDutyCycle, ptrTxBuffer, sizeof(TIM1_PwmDutyCycle)); 
      break;
    case DAC_OUTPUT:
      break;
    default:
      break;
  }
}
