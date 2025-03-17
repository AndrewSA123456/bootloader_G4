#include "backup_reg.h"

void backup_reg_write(uint32_t data, volatile uint32_t *backupReg)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;  // Enable power interface clock
	PWR->CR1 |= PWR_CR1_DBP;			  // Enable access to the backup domain
	*backupReg = data;					  // Write data to backup register
	PWR->CR1 &= ~PWR_CR1_DBP;			  // Disable access to the backup domain
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN; // Disable power interface clock
}
uint32_t backup_reg_read(volatile uint32_t *backupReg)
{
	return *backupReg; 
}