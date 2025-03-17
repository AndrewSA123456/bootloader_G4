#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "main.h"

void bootloader(void);
#define NODE_ID 2
#define SDO_TX (0x600 + NODE_ID)
#define SDO_RX (0x580 + NODE_ID)
#define CMD_BOOTLOAD (0x0U)

#define SDO_NORMAL_TRANSFER_SRB_REQ (0x00U)
#define SDO_NORMAL_TRANSFER_SRB_REQ_TG (0x10U)
#define SDO_NORMAL_TRANSFER_SRB_REQ_LAST (0x1U)
#define SDO_NORMAL_TRANSFER_SRB_REQ_LAST_TG (0x11U)


void FDCAN1_FilterSet(uint32_t filterCAN_ID1,
	uint32_t filtermask);
void FDCAN1_ReceiveMessage(uint8_t *RxData,
						   uint32_t *len);
void FDCAN1_TransmitMessage(uint32_t CAN_ID,
							uint8_t *data,
							uint32_t len);
#endif // __BOOTLOADER_H
