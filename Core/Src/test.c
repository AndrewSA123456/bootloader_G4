// #include "stm32g4xx_hal.h"
// #include "fdcan.h"
// #include "gpio.h"

// #define APP_ADDRESS 0x08008000 // Адрес начала основной программы

// uint8_t rxData[8];
// uint32_t rxId;
// uint8_t rxLen;

// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_CAN_Init(void);
// void JumpToApplication(void);

// int main(void)
// {
// 	HAL_Init();
// 	SystemClock_Config();
// 	MX_GPIO_Init();
// 	MX_CAN_Init();

// 	// Основной цикл bootloader'а
// 	while (1)
// 	{
// 		if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
// 		{
// 			// Обработка входящих сообщений
// 			if (rxHeader.StdId == 0x100)
// 			{ // Пример: команда на начало загрузки
// 				// Очистка флеш-памяти и подготовка к записи
// 				FLASH_EraseInitTypeDef eraseInitStruct;
// 				uint32_t sectorError;
// 				eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
// 				eraseInitStruct.Page = 128;	   // Начальная страница для стирания
// 				eraseInitStruct.NbPages = 128; // Количество страниц
// 				HAL_FLASH_Unlock();
// 				HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);
// 				HAL_FLASH_Lock();
// 			}
// 			else if (rxHeader.StdId == 0x101)
// 			{ // Пример: передача данных
// 				// Запись данных во флеш-память
// 				HAL_FLASH_Unlock();
// 				for (uint8_t i = 0; i < rxLen; i++)
// 				{
// 					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, APP_ADDRESS + rxHeader.StdId * 8 + i, rxData[i]);
// 				}
// 				HAL_FLASH_Lock();
// 			}
// 			else if (rxHeader.StdId == 0x102)
// 			{ // Пример: команда на завершение загрузки
// 				// Переход к основной программе
// 				JumpToApplication();
// 			}
// 		}
// 	}
// }

// void JumpToApplication(void)
// {
// 	uint32_t jumpAddress = *(__IO uint32_t *)(APP_ADDRESS + 4);
// 	void (*application)(void) = (void (*)(void))jumpAddress;

// 	// Инициализация стека и переход к основной программе
// 	__set_MSP(*(__IO uint32_t *)APP_ADDRESS);
// 	application();
// }

// #define USER_APPLICATION_BASE_ADDRESS 0x8004000U

// uint8_t srec_data[16];

// void __svc(1) EnablePrivilegedMode(void);
// void __SVC_1(void)
// {
// 	__disable_irq();
// 	__set_CONTROL((__get_CONTROL()) & 0xFFFFFFFE); // enter priv mode
// 	__enable_irq();
// }

// static void BootJump(uint32_t *Address)
// {
// 	/* 1. Make sure, the CPU is in privileged mode. */
// 	if (CONTROL_nPRIV_Msk & __get_CONTROL())
// 	{ /* not in privileged mode */
// 		EnablePrivilegedMode();
// 	}

// 	/* 2. Disable all enabled interrupts in NVIC. */
// 	NVIC->ICER[0] = 0xFFFFFFFF;
// 	NVIC->ICER[1] = 0xFFFFFFFF;
// 	NVIC->ICER[2] = 0xFFFFFFFF;
// 	NVIC->ICER[3] = 0xFFFFFFFF;
// 	NVIC->ICER[4] = 0xFFFFFFFF;
// 	NVIC->ICER[5] = 0xFFFFFFFF;
// 	NVIC->ICER[6] = 0xFFFFFFFF;
// 	NVIC->ICER[7] = 0xFFFFFFFF;

// 	/* 3. Disable all enabled peripherals which might generate interrupt requests,
// 		  and clear all pending interrupt flags in those peripherals. Because this
// 		  is device-specific, refer to the device datasheet for the proper way to
// 		  clear these peripheral interrupts.  */

// 	/* 4. Clear all pending interrupt requests in NVIC. */
// 	NVIC->ICPR[0] = 0xFFFFFFFF;
// 	NVIC->ICPR[1] = 0xFFFFFFFF;
// 	NVIC->ICPR[2] = 0xFFFFFFFF;
// 	NVIC->ICPR[3] = 0xFFFFFFFF;
// 	NVIC->ICPR[4] = 0xFFFFFFFF;
// 	NVIC->ICPR[5] = 0xFFFFFFFF;
// 	NVIC->ICPR[6] = 0xFFFFFFFF;
// 	NVIC->ICPR[7] = 0xFFFFFFFF;

// 	/* 5. Disable SysTick and clear its exception pending bit, if it is used in the bootloader, e. g. by the RTX.  */
// 	SysTick->CTRL = 0;
// 	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

// 	/* 6. Disable individual fault handlers if the bootloader used them. */
// 	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk |
// 					SCB_SHCSR_BUSFAULTENA_Msk |
// 					SCB_SHCSR_MEMFAULTENA_Msk);

// 	/* 7. Activate the MSP, if the core is found to currently run with the PSP. */
// 	if (CONTROL_SPSEL_Msk & __get_CONTROL())
// 	{ /* MSP is not active */
// 		__set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
// 	}

// 	/*8. Load the vector table address of the user application into SCB->VTOR register.
// 		 Make sure the address meets the alignment requirements of the register. */
// 	SCB->VTOR = (uint32_t)Address;

// 	/* 9. Set the MSP to the value found in the user application vector table. */
// 	__set_MSP(Address[0]);

// 	/* 10. Set the PC to the reset vector value of the user application via a function call. */
// 	((void (*)(void))Address[1])();
// }

// int main(void)
// {
// 	uint8_t uart_receive_buffer[100];
// 	uint8_t uart_rxd_old = 0x00;
// 	uint8_t uart_rxd = 0x00;
// 	int i = 0;
// 	int j = 0;
// 	uint32_t flash_address = 0;
// 	uint32_t srec_data_count = 0; 
// 	uint32_t data_count = 0;

// 	Flash_Init();
// 	Flash_Erase_Sector(1);

// 	/* Receive SREC while "S7" is reached */
// 	while ((uart_receive_buffer[1] != '7') & (uart_receive_buffer[1] != '0'))
// 	{
// 		i = 0;
// 		uart_rxd_old = 0x00;
// 		uart_rxd = 0x00;
// 		/* Receive over UART while CR, LF is sent */
// 		do
// 		{
// 			uart_rxd_old = uart_rxd;
// 			uart_rxd = uart_receive();
// 			if ((uart_rxd != 0x0d) & (uart_rxd != 0x0a))
// 			{
// 				uart_receive_buffer[i] = uart_rxd;
// 			}
// 			// uart_transmit(uart_rxd);
// 			i++;
// 		} while (((uart_rxd_old != 0x0d) & (uart_rxd != 0x0a)));

// 		/* Writting SREC data to FLASH */
// 		if ((uart_receive_buffer[0] == 'S') | (uart_receive_buffer[0] == 's')) /* A valid SREC data */
// 		{
// 			if (uart_receive_buffer[1] == 0x33) /* A 32 bit addressing */
// 			{
// 				srec_data_count = ((uart_receive_buffer[2] - 0x30) << 4) + (uart_receive_buffer[3] - 0x30);
// 				data_count = srec_data_count - 5;
// 				flash_address = (ascii_to_number(uart_receive_buffer[4]) << 28);
// 				Flash_Write_x32(flash_address, (ascii_to_number(uart_receive_buffer[12]) << 4));
// 			}
// 		}
// 	}

// 	Flash_Deinit();

// 	/* Jump to application code space */
// 	BootJump((uint32_t *)USER_APPLICATION_BASE_ADDRESS);
// }