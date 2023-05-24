/* utils.h
 *
 * Contains common utility functions and structures for writing apps.
 *
 */
#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

#define CAN_MAX_DLC 8

// 1 page (2KiB) is reserved for persistent storage
#define FLASH_RW_START 0x0807F800
#define FLASH_RW_END (FLASH_RW_START + 2 * 1024)

// Simple error reporting codes for user applications
#define APP_OK 0
#define APP_NOT_OK 1

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLC];
} CANFrame;

// General error handler
void Error_Handler(void);
// Print message to uart
void Print(char *str);
// Send task notification to task
void NotifyTask(TaskHandle_t taskHandle);

// Setup the writeable flash memory area
int FlashRWInit(void);

// Erase the persistent storage. This needs to be done prior to first time
// programming of the flash. Returns 0 on success, 1 otherwise.
int EraseFlash(void);
// Read from FLASH_RW area. Return 0 on success, 1 otherwise.
int ReadFlash(uint32_t addr, void *data, size_t len);
// Write to FLASH_RW area. The len parameter needs to be word-aligned (32-bit)
// since we program in word mode. Return 0 on success, 1 otherwise.
int WriteFlash(uint32_t addr, const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
