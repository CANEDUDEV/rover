/*******************************************************************************
 * @file interrupts.h
 *
 * Provides interrupt and exception handlers.
 ******************************************************************************/
#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common-interrupts.h"

void USB_HP_CAN_TX_IRQHandler(void);
void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* INTERRUPTS_H */
