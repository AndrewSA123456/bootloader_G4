#include "bootloader.h"
#include "fdcan.h"
#include "gpio.h"
#include "backup_reg.h"
#include "aes.h"

#include <stdbool.h>

#define RING_BUFF_SIZE 21
typedef struct
{
	uint8_t data[RING_BUFF_SIZE];
	uint32_t head;
	uint32_t tail;
} RingBuff;

bool isBuffEmpty(RingBuff *buff);
bool isBuffFull(RingBuff *buff);
bool pushBuff(RingBuff *buff, uint8_t data);
bool popBuff(RingBuff *buff, uint8_t *data);
uint8_t key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
void buffDecrypt(RingBuff *toDecrypt, uint64_t *isDecrypted);
uint8_t rebootResponse[4] = {0x60, 0x51, 0x1F, 0x01};
uint8_t transmitInitResponse[4] = {0x60, 0x50, 0x1F, 0x01};

uint8_t transmitResponse = 0x20;
uint8_t transmitToggleResponse = 0x30;

// Заложим 32кБ на бутлоадер, думаю это "по брови"
#define STD_APP_ADDR (0x08008000U)

void JumpToApplication(void);

#define WAIT_COUNTER_INIT 1000000
void bootloader(void)
{
	// Смотрим наличие команды на работу бутлоадера в бекап регистре
	uint32_t bootloaderStatus = backup_reg_read(&TAMP->BKP1R);
	if (bootloaderStatus == CMD_BOOTLOAD)
	{
		uint32_t currentProgramAppAddr = STD_APP_ADDR;
		MX_FDCAN1_Init();
		FDCAN1_FilterSet(SDO_TX, 0x7FFU);
		HAL_FDCAN_Start(&hfdcan1);
		// Отправляем подтверждение: cansend vcan0 582# 60 51 1F 01
		// FDCAN1_TransmitMessage(SDO_RX, rebootResponse, 4);
		// Ожидаем инициализацию сегментированной передачи
		uint32_t wait_counter = WAIT_COUNTER_INIT;
		uint8_t RxData[8] = {0};
		uint32_t RxDataLen = 0;
		while (wait_counter)
		{
			FDCAN1_ReceiveMessage(RxData, &RxDataLen);
			if (RxDataLen != 0)
			{
				break;
			}
			// wait_counter--;
		}
		uint32_t AppSize = 0;
		for (int i = 4; i < 8; i++)
		{
			AppSize |= (uint32_t)(RxData[i] << 8 * (i-4));
		}
		// Стираем пользовательскую прошивку
		FLASH_EraseInitTypeDef eraseInitStruct;
		uint32_t sectorError;
		eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		eraseInitStruct.Page = 32;			// Начальная страница для стирания
		eraseInitStruct.NbPages = 128 - 32; // Количество страниц
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);
		HAL_FLASH_Lock();
		// Отправляем подтверждение: cansend vcan0 582#60 50 1F 01
		FDCAN1_TransmitMessage(SDO_RX, transmitInitResponse, 4);
		//////////////////////////////////////////////////
		RingBuff toDecrypt = {0};
		uint64_t isDecripted[2] = {0};
		for (int i = 0; i < AppSize+1; i += 7)
		{
			// Ожидаем пакет прошивки
			wait_counter = WAIT_COUNTER_INIT;
			RxDataLen = 0;
			while (wait_counter)
			{
				FDCAN1_ReceiveMessage(RxData, &RxDataLen);
				if (RxDataLen != 0)
				{
					break;
				}
				// wait_counter--;
			}
			// Извлекаем из пакета 7 байт прошивки и кладем в кольцевой буфер
			for (int j = 1; j < 8; j++)
			{
				pushBuff(&toDecrypt, RxData[j]);
			}
			// Если буфер заполнен, расшифруем его
			if (isBuffFull(&toDecrypt))
			{
				buffDecrypt(&toDecrypt, isDecripted);
				__ASM("nop");
//				if (isDecripted[0] == 0 || isDecripted[1] == 0)
//				{
//					Error_Handler();
//				}
				// Расшифрованный сегмент прошивки записываем во флеш
				HAL_FLASH_Unlock();
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentProgramAppAddr, isDecripted[0]);
				currentProgramAppAddr += 8;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentProgramAppAddr, isDecripted[1]);
				currentProgramAppAddr += 8;
				HAL_FLASH_Lock();
			}
			// Отправляем подтверждение
			switch (RxData[0])
			{
			case SDO_NORMAL_TRANSFER_SRB_REQ:
			case SDO_NORMAL_TRANSFER_SRB_REQ_LAST:
			{
				FDCAN1_TransmitMessage(SDO_RX, &transmitResponse, 1);
			}
			break;
			case SDO_NORMAL_TRANSFER_SRB_REQ_TG:
			case SDO_NORMAL_TRANSFER_SRB_REQ_LAST_TG:
			{
				FDCAN1_TransmitMessage(SDO_RX, &transmitToggleResponse, 1);
			}
			break;
			}
		}
		HAL_Delay(1000);
		// Вызываем функцию JumpToApplication с новыми параметрами
		JumpToApplication();
	}
	else
	{
		// Вызываем функцию JumpToApplication со стандартными параметрами
		JumpToApplication();
	}
}
//////////////////////////////////////////////////
// Функция: Проверка, пуст ли буфер
bool isBuffEmpty(RingBuff *buff)
{
	return buff->head == buff->tail;
}
//////////////////////////////////////////////////
// Функция: Проверка, полон ли буфер
bool isBuffFull(RingBuff *buff)
{
	return (buff->tail + 1) % RING_BUFF_SIZE == buff->head;
}
//////////////////////////////////////////////////
// Функция: Добавление элемента в буфер
bool pushBuff(RingBuff *buff, uint8_t data)
{
	if (isBuffFull(buff))
	{
		return false;
	}
	buff->data[buff->tail] = data;
	buff->tail = (buff->tail + 1) % RING_BUFF_SIZE;
	return true;
}
//////////////////////////////////////////////////
// Функция: Извлечение элемента из буфера
bool popBuff(RingBuff *buff, uint8_t *data)
{
	if (isBuffEmpty(buff))
	{
		return false;
	}
	*data = buff->data[buff->head];
	buff->head = (buff->head + 1) % RING_BUFF_SIZE;
	return true;
}
//////////////////////////////////////////////////
// Функция: выдергивает 16 байт из кольцевого буфера и расшифрует их
void buffDecrypt(RingBuff *toDecrypt, uint64_t *isDecrypted)
{
	uint8_t buff = 0;
	uint8_t arr[16] = {0};
	for (int i = 0; i < 16; i++)
	{
		popBuff(toDecrypt, &buff);
		arr[i] = buff;
	}
	struct AES_ctx ctx;
	AES_init_ctx(&ctx, key);
	AES_ECB_decrypt(&ctx, arr);
	for (int i = 0; i < 4; i++)
	{
		isDecrypted[0] |= (uint64_t)(arr[i]) << (i * 8 + 32);
		isDecrypted[0] |= (uint64_t)(arr[i + 4]) << (i * 8);
		isDecrypted[1] |= (uint64_t)(arr[i + 8]) << (i * 8 + 32);
		isDecrypted[1] |= (uint64_t)(arr[i + 12]) << (i * 8);
	}
}
//////////////////////////////////////////////////
// Функция:
typedef void (*pFunction)(void);
void JumpToApplication(void)
{
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	uint32_t vectorAddr = STD_APP_ADDR;
	uint32_t steckAddr = STD_APP_ADDR;
	uint32_t procAddr = STD_APP_ADDR + 4;

	__disable_irq();

	HAL_FDCAN_MspDeInit(&hfdcan1);
	HAL_RCC_DeInit();
	HAL_DeInit();

	SCB->VTOR = vectorAddr;

	// Получаем адрес Reset Handler приложения
	JumpAddress = *(volatile uint32_t *)(procAddr);
	Jump_To_Application = (pFunction)JumpAddress;

	// Устанавливаем указатель стека (SP) из вектора сброса приложения
	__set_MSP(*(volatile uint32_t *)steckAddr);
	// Переход к приложению
	Jump_To_Application();
}

void FDCAN1_FilterSet(uint32_t filterCAN_ID1, uint32_t filtermask)
{
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = filterCAN_ID1;
	sFilterConfig.FilterID2 = filtermask;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	  Error_Handler();
	}
}

void FDCAN1_ReceiveMessage(uint8_t *RxData, uint32_t *len)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
	{
		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			Error_Handler();
		}
		*len = RxHeader.DataLength;
	}
}

void FDCAN1_TransmitMessage(uint32_t CAN_ID, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = CAN_ID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = len;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
	{
		Error_Handler();
	}
}
