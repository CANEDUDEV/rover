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

void DMA1_Channel1_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void USB_LP_CAN_RX0_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* INTERRUPTS_H */
