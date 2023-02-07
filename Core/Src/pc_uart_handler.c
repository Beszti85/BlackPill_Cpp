/*
 * pc_uart_handler.c
 *
 *  Created on: 2023. febr. 1.
 *      Author: drCsabesz
 */

#include "pc_uart_handler.h"
#include "pca9685pw.h"
#include "esp8266_at.h"

extern PCA9685_Handler_t LedDriverHandle;
extern UART_HandleTypeDef huart1;

uint8_t PCUART_EspAtCmdNum = 0u;

void PCUART_ProcessRxCmd( uint8_t* ptrBuffer )
{
  if( !strncmp( ptrBuffer, "LED_TOGGLE", sizeof("LED_TOGGLE")-1 ) )
  {
    PCA9685_ToggleOutputEnable(&LedDriverHandle);
  }
  else
  if( !strncmp( ptrBuffer, "ESP_AT", sizeof("ESP_AT")-1 ) )
  {
    // go to the end of the cmd prefix
    ptrBuffer += sizeof("ESP_AT")-1;
    // Check the end of the string
    if( *ptrBuffer != '0' )
    {
      // Get the number
      PCUART_EspAtCmdNum = *ptrBuffer - '0';
      ptrBuffer++;
      // Any further characters - number > 9
      if( *ptrBuffer != '0' )
      {
        PCUART_EspAtCmdNum *= 10u;
        PCUART_EspAtCmdNum += (*ptrBuffer - '0');
      }
      ESP8266_ProcessAtCmd( &huart1, ESP8266_CMD_ID_RST );
    }
  }
}
