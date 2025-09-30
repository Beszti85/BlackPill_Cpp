/*
 * pc_data_readwrite.h
 *
 *  Created on: Sep 1, 2025
 *      Author: drCsabesz
 */

#ifndef INC_PC_DATA_READWRITE_H_
#define INC_PC_DATA_READWRITE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define BOARD_ID                 0u
#define BME280_PHYSICAL_VALUES   1u
#define ADC_PHY_VALUES           2u
#define FLASH_ID                 3u

#ifdef __cplusplus
extern "C" {
#endif

uint8_t PC_ReadDataHandler( uint8_t readId, uint8_t* ptrTxBuffer );

#ifdef __cplusplus
}
#endif

#endif /* INC_PC_DATA_READWRITE_H_ */
