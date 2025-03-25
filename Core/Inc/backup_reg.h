#ifndef __BACKUP_REG_H
#define __BACKUP_REG_H

#include "stm32g4xx_hal.h"

void backup_reg_write(uint32_t data,
                      volatile uint32_t *backupReg);
uint32_t backup_reg_read(volatile uint32_t *backupReg);
 
#endif //__BACKUP_REG_H
