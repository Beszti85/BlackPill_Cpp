/*
 * pc_uart_handler.h
 *
 *  Created on: 2023. febr. 1.
 *      Author: drCsabesz
 */

#ifndef INC_PCUARTHANDLER_H_
#define INC_PCUARTHANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void PCUART_ProcessRxCmd( uint8_t* cmdbuffer );

#ifdef __cplusplus
}
#endif

#endif
