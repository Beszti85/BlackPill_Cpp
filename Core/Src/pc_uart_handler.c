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

void PCUART_ProcessRxCmd( uint8_t* ptrBuffer )
{
  if( !strncmp( ptrBuffer, "LED_TOGGLE", sizeof("LED_TOGGLE")-1 ) )
  {
    PCA9685_ToggleOutputEnable(&LedDriverHandle);
  }
  else
  if( !strncmp( ptrBuffer, "ESP_RST", sizeof("ESP_RST")-1 ) )
  {
    ESP8266_ProcessAtCmd( &huart1, ESP8266_CMD_ID_RST );
  }
}
