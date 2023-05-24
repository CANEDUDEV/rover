#ifndef COMMON_INTERRUPTS_H
#define COMMON_INTERRUPTS_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_INTERRUPTS_H */
