/*
 * pc_uart_handler.c
 *
 *  Created on: 2023. febr. 1.
 *      Author: drCsabesz
 */

#include "pc_uart_handler.h"
#include "pca9685pw.h"

extern PCA9685_Handler_t LedDriverHandle;

void PCUART_ProcessRxCmd( uint8_t* ptrBuffer )
{
  if( !strncmp( ptrBuffer, "LED_TOGGLE", sizeof("LED_TOGGLE") ) )
  {
    PCA9685_ToggleOutputEnable(&LedDriverHandle);
  }
}
