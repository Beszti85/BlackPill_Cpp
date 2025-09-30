/*
 * pc_act_cmd.c
 *
 *  Created on: Sep 7, 2025
 *      Author: drCsabesz
 */

#include "pc_act_cmd.h"

void PC_ExecCmdHandler( uint8_t* ptrRxBuffer, uint8_t* ptrTxBuffer )
{
  uint8_t cmd = ptrRxBuffer[0];
  // Cehck first byte
  switch (cmd)
  {
    // Read external flash
    case 1:
      //FLASH_Read()
      break;
  }
}
