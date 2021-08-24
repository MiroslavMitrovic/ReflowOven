/*
 /**
 ******************************************************************************
 * @file           : bootloader.c
 * @brief          : Bootloader functions
 ******************************************************************************
 */

#include "bootloader.h"

void bootloaderInit()
{
	Flashed_offset = 0;
	flashStatus = Unerased;
	BootloaderMode bootloaderMode;
	if (HAL_GPIO_ReadPin(BOOT1_GPIO_Port, BOOT1_Pin) == GPIO_PIN_SET)
	{
		bootloaderMode = FlashMode;
		for (uint8_t i = 0; i < 10; i++)
		{
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
			HAL_Delay(90);
		}
//		HAL_GPIO_WritePin(USB_ENABLE_GPIO_Port, USB_ENABLE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}
	else
	{
		bootloaderMode = JumpMode;
//		HAL_GPIO_WritePin(USB_ENABLE_GPIO_Port, USB_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}


		App = App1;


	if (bootloaderMode == JumpMode)
	{
		if (App == App1)
		{
			//Check if the application is there
			uint8_t emptyCellCount = 0;
			for (uint8_t i = 0; i < 10; i++)
			{
				if (readWord(APP1_START + (i * 4)) == -1)
				{
					emptyCellCount++;
				}
				else
				{

				}
			}

			if (emptyCellCount != 10)
			{
				jumpToApp(APP1_START);
			}
			else
			{
				errorBlink();
			}
		}
		else
		{

		}
	}
}

void flashWord(uint32_t dataToFlash) {
	/*TODO Debug and check functioning of write */
	if (flashStatus == Unlocked)
	{
		volatile HAL_StatusTypeDef status;
		uint8_t flash_attempt = 0;
		uint32_t address;
		do {
			if (App == App1)
			{
				address = APP1_START + Flashed_offset;
			}
			else
			{
				address = APP2_START + Flashed_offset;
			}
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
					dataToFlash);
			flash_attempt++;
		} while (status != HAL_OK && flash_attempt < 10
				&& dataToFlash == readWord(address));

		if (status != HAL_OK)
		{
			CDC_Transmit_FS((uint8_t*) &"Flashing Error!\n",
			strlen("Flashing Error!\n"));
		}
		else
		{ //Word Flash Successful
			Flashed_offset += 4;
			CDC_Transmit_FS((uint8_t*) &"Flash: OK\n", strlen("Flash: OK\n"));
		}
	}
	else
	{
		CDC_Transmit_FS((uint8_t*) &"Error: Memory not unlocked nor erased!\n",
		strlen("Error: Memory not unlocked nor erased!\n"));
	}
}

uint32_t readWord(uint32_t address) {
	uint32_t read_data;
	read_data = *(uint32_t*) (address);
	return read_data;
}

void eraseMemory() {
	/* Unock the Flash to enable the flash control register access *************/
	while (HAL_FLASH_Unlock() != HAL_OK)
		while (HAL_FLASH_Lock() != HAL_OK)
			; //Weird fix attempt

	/* Allow Access to option bytes sector */
	while (HAL_FLASH_OB_Unlock() != HAL_OK)
		while (HAL_FLASH_OB_Lock() != HAL_OK)
			; //Weird fix attempt
	/*TODO To be modified to perform erase on sectors, see FLASH_SECTOR.c from Reflow Oven!*/
	/* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	if (App == App1)
	{
		/*hack for sector*/
		/*EraseInitStruct.Sector = APP1_START;*/
		/*hack for sector*/
		EraseInitStruct.Sector = 5;
	}
	else
	{

	}
	/*EraseInitStruct.NbSectors = FLASH_SECTOR_SIZE/FLASH_SECTOR_SIZE_USER;*/
	/*hack for sector*/
	EraseInitStruct.NbSectors = 1;
	uint32_t SectorError;

	volatile HAL_StatusTypeDef status_erase;
	status_erase = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

	/* Lock the Flash to enable the flash control register access *************/
	while (HAL_FLASH_Lock() != HAL_OK)
		while (HAL_FLASH_Unlock() != HAL_OK)
			; //Weird fix attempt

	/* Lock Access to option bytes sector */
	while (HAL_FLASH_OB_Lock() != HAL_OK)
		while (HAL_FLASH_OB_Unlock() != HAL_OK)
			; //Weird fix attempt

	if (status_erase != HAL_OK) {
		errorBlink();
	} else {
		flashStatus = Erased;
		Flashed_offset = 0;
	}
}

void unlockFlashAndEraseMemory() {
	/* Unock the Flash to enable the flash control register access *************/
	while (HAL_FLASH_Unlock() != HAL_OK)
		while (HAL_FLASH_Lock() != HAL_OK)
			; //Weird fix attempt

	/* Allow Access to option bytes sector */
	while (HAL_FLASH_OB_Unlock() != HAL_OK)
		while (HAL_FLASH_OB_Lock() != HAL_OK)
			; //Weird fix attempt
//TODO Flash erase sector instead of pages
	if (flashStatus != Erased) {
		/* Fill EraseInit structure*/
		FLASH_EraseInitTypeDef EraseInitStruct;
		//EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		//FLASH_TYPEERASE_SECTORS
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		if (App == App1)
		{
		/*EraseInitStruct.Sector = APP1_START;*/
		/*hack for sector*/
		EraseInitStruct.Sector = 5;
		}
		else
		{

		}
		/*EraseInitStruct.NbSectors = FLASH_SECTOR_SIZE/FLASH_SECTOR_SIZE_USER;*/
		/*hack for sector*/
		EraseInitStruct.NbSectors = 1;
		uint32_t SectorError;

		volatile HAL_StatusTypeDef status_erase;
		status_erase = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

		if (status_erase != HAL_OK)
		{
			errorBlink();
		}
		else
		{

		}
	}

	flashStatus = Unlocked;
}

void lockFlash() {
	/* Lock the Flash to enable the flash control register access *************/
	while (HAL_FLASH_Lock() != HAL_OK)
		while (HAL_FLASH_Unlock() != HAL_OK)
			;		//Weird fix attempt

	/* Lock Access to option bytes sector */
	while (HAL_FLASH_OB_Lock() != HAL_OK)
		while (HAL_FLASH_OB_Unlock() != HAL_OK)
			;		//Weird fix attempt

	flashStatus = Locked;
}

void jumpToApp(const uint32_t address) {
	const JumpStruct *vector_p = (JumpStruct*) address;

	deinitEverything();

	/* let's do The Jump! */
	/* Jump, used asm to avoid stack optimization */
	asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
}

void deinitEverything() {
	/*TODO add all the peripherals and deinit them-check to be performed to see if this is working!!!*/

	/*-- reset peripherals to guarantee flawless start of user application*/
	HAL_GPIO_DeInit(LD5_GPIO_Port, LD5_Pin);
	HAL_GPIO_DeInit(LD6_GPIO_Port, LD6_Pin);
	USBD_DeInit(&hUsbDeviceFS);

	  /* GPIO Ports Clock Disable */  /* GPIO Ports Clock Enabled */
	__HAL_RCC_GPIOC_CLK_DISABLE(); /*__HAL_RCC_GPIOC_CLK_ENABLE();*/
	__HAL_RCC_GPIOD_CLK_DISABLE(); /*__HAL_RCC_GPIOD_CLK_ENABLE();*/
	__HAL_RCC_GPIOB_CLK_DISABLE(); /*__HAL_RCC_GPIOA_CLK_ENABLE();*/
	__HAL_RCC_GPIOA_CLK_DISABLE(); /*__HAL_RCC_GPIOH_CLK_ENABLE();*/
	__HAL_RCC_GPIOE_CLK_DISABLE(); /*__HAL_RCC_GPIOE_CLK_ENABLE();*/
	__HAL_RCC_GPIOH_CLK_DISABLE(); /*__HAL_RCC_GPIOH_CLK_ENABLE();*/

	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}

uint8_t string_compare(char array1[], char array2[], uint16_t length) {
	uint8_t comVAR = 0, i;
	for (i = 0; i < length; i++) {
		if (array1[i] == array2[i])
			comVAR++;
		else
			comVAR = 0;
	}
	if (comVAR == length)
		return 1;
	else
		return 0;
}

void errorBlink() {
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	while (1) {
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(800);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(800);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(800);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_Delay(3000);
	}
}

void messageHandler(uint8_t *Buf)
{
	if (string_compare((char*) Buf, ERASE_FLASH_MEMORY,
			strlen(ERASE_FLASH_MEMORY)) && flashStatus != Unlocked)
	{
		eraseMemory();
		CDC_Transmit_FS((uint8_t*) &"Flash: Erased!\n",
				strlen("Flash: Erased!\n"));
	}
	else if (string_compare((char*) Buf, FLASHING_START,
			strlen(FLASHING_START)))
	{
		unlockFlashAndEraseMemory();
		CDC_Transmit_FS((uint8_t*) &"Flash: Unlocked!\n",
				strlen("Flash: Unlocked!\n"));
	}
	else if (string_compare((char*) Buf, FLASHING_FINISH,
			strlen(FLASHING_FINISH)) && flashStatus == Unlocked)
	{
		lockFlash();
		CDC_Transmit_FS((uint8_t*) &"Flash: Success!\n",
				strlen("Flash: Success!\n"));
	}
	else if (string_compare((char*) Buf, FLASHING_ABORT,
			strlen(FLASHING_ABORT)) && flashStatus == Unlocked)
	{
		lockFlash();
		eraseMemory();
		CDC_Transmit_FS((uint8_t*) &"Flash: Aborted!\n",
				strlen("Flash: Aborted!\n"));
	}
	else
	{
		CDC_Transmit_FS((uint8_t*) &"Error: Incorrect step or unknown command!\n",
				strlen("Error: Incorrect step or unknown command!\n"));
	}
}
