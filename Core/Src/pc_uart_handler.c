/*
 * pc_uart_handler.c
 *
 *  Created on: 2023. febr. 1.
 *      Author: drCsabesz
 */

#include "pc_uart_handler.h"
#include "pca9685pw.h"
#include "esp8266_at.h"
#include "ds1307.h"
#include "usbd_cdc_if.h"

extern PCA9685_Handler_t LedDriverHandle;
extern UART_HandleTypeDef huart1;
extern DS1307_TimeDate_t DS1307_DateTime;

uint8_t PCUART_RxBuffer[80u];

uint8_t PCUART_EspAtCmdNum = 0u;
static uint8_t ResponseLength = 0uj;

void PCUART_ProcessRxCmd( uint8_t* ptrBuffer )
{
  if( !strncmp( ptrBuffer, "LED_TOGGLE", sizeof("LED_TOGGLE") - 1 ) )
  {
    PCA9685_ToggleOutputEnable(&LedDriverHandle);
  }
  else
  if( !strncmp( ptrBuffer, "ESP_AT", sizeof("ESP_AT") - 1 ) )
  {
    // go to the end of the cmd prefix
    ptrBuffer += sizeof("ESP_AT")-1;
    // Check the end of the string
    if( *ptrBuffer != ' ' )
    {
      // Get the number
      PCUART_EspAtCmdNum = *ptrBuffer - '0';
      ptrBuffer++;
      // Any further characters - number > 9
      if( *ptrBuffer != ' ' )
      {
        PCUART_EspAtCmdNum *= 10u;
        PCUART_EspAtCmdNum += (*ptrBuffer - '0');
      }
      ESP8266_ProcessAtCmd( &huart1, (ESP8266_CMD_ID)PCUART_EspAtCmdNum );
    }
  }
  else
  if( !strncmp( ptrBuffer, "READ_TIME", sizeof("READ_TIME") - 1 ) )
  {
    ResponseLength = DS1307_ConvertData( &PCUART_RxBuffer, &DS1307_DateTime );
    CDC_Transmit_FS( &PCUART_RxBuffer, ResponseLength );
  }
}
