/*
 * pc_act_cmd.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_act_cmd.h"
#include "nrf24l01.h"
#include "main.h"

extern NRF24L01_Handler_t RFHandler;

void PC_ExecCmdHandler( uint8_t* ptrRxBuffer, uint8_t* ptrTxBuffer )
{
  uint8_t cmd = ptrRxBuffer[0u];
  uint8_t response_length = 0u;
  // Cehck first byte
  switch (cmd)
  {
    // Read NFR24L01 register
    case NRF24L01_REG_READ:
      response_length = NRF24L01_ReadRegister(&RFHandler, ptrRxBuffer[1u]);
      memcpy(ptrTxBuffer, RFHandler.RxBuffer, response_length);
      break;

    case NRF24L01_REG_WRITE_1BYTE:
      NRF24L01_WriteRegister1Byte(&RFHandler, ptrTxBuffer[1u], ptrTxBuffer[2u ]);
      break;

    default:
      break;
  }
}
