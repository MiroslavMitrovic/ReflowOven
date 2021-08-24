/*
 ******************************************************************************
  * @file           : bootloader.h
  * @brief          : Header for bootloader.h file.
 ******************************************************************************
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_
//TODO Addresses to be defined
#define APP1_START (0x08020000)			//Sector 5 APP1 (72kB)
#define APP2_START (0x08040000)			//Sector 6  App2 Bank (72kB)
#define FLASH_SECTOR_SIZE (0x20000)		//128kB

#define FLASH_SECTOR_SIZE_USER (0x1A000)	/*104kB(APP+BL)*/

#define ERASE_FLASH_MEMORY "#$ERASE_MEM"
#define FLASHING_START "#$FLASH_START"
#define FLASHING_FINISH "#$FLASH_FINISH"
#define FLASHING_ABORT "#$FLASH_ABORT"

#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

typedef enum
{
    JumpMode,
	FlashMode
} BootloaderMode;

typedef enum
{
    App1,
	App2
} AppSelection;

typedef enum
{
    Unerased,
	Erased,
	Unlocked,
	Locked
} FlashStatus;

typedef void (application_t)(void);

typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

AppSelection App;
uint32_t Flashed_offset;
FlashStatus flashStatus;
extern USBD_HandleTypeDef hUsbDeviceFS;//it is defined in the usb_device.c

void bootloaderInit();
void flashWord(uint32_t word);
uint32_t readWord(uint32_t address);
void eraseMemory();
void unlockFlashAndEraseMemory();
void lockFlash();
void jumpToApp();
void deinitEverything();
uint8_t string_compare(char array1[], char array2[], uint16_t length);
void errorBlink();
void messageHandler(uint8_t* Buf);

#endif /* INC_BOOTLOADER_H_ */
