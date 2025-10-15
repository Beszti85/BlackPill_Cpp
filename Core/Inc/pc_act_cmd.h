/*
 * pc_act_cmd.h
 *
 *  Created on: Sep 1, 2025
 *      Author: drCsabesz
 */

#ifndef INC_PC_ACT_CMD_H_
#define INC_PC_ACT_CMD_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24L01_REG_READ         0
#define NRF24L01_REG_WRITE_1BYTE  1
#define NRF24L01_REG_WRITE_NBYTES 2
#define BOARD_LED_ENDIS           3

void PC_ExecCmdHandler( uint8_t* ptrRxBuffer, uint8_t* ptrTxBuffer );

#ifdef __cplusplus
}
#endif

#endif /* INC_PC_ACT_CMD_H_ */
