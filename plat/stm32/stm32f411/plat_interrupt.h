#ifndef __PLAT_INTERRUPT_H__
#define __PLAT_INTERRUPT_H__
#include <stdbool.h>


typedef enum
{
    EXTI_IRQ_1 = 0,
    EXTI_IRQ_2,
    EXTI_IRQ_3,
    EXTI_IRQ_4,
    EXTI_IRQ_5,
    EXTI_IRQ_6,
    EXTI_IRQ_7,
    EXTI_IRQ_8,
    EXTI_IRQ_9,
    EXTI_IRQ_10,
    EXTI_IRQ_11,
    EXTI_IRQ_12,
    EXTI_IRQ_13,
    EXTI_IRQ_14,
    EXTI_IRQ_15,
    EXTI_IRQ_MAX,
}EXTI_IRQ_E;


void
     extiInit(void);
bool extiTest(void);

void EXTI0_Callback(void);
void EXTI1_Callback(void);
void EXTI2_Callback(void);
void EXTI3_Callback(void);
// void EXTI4_Callback(void);
void EXTI5_Callback(void);
void EXTI6_Callback(void);
void EXTI7_Callback(void);
void EXTI8_Callback(void);
void EXTI9_Callback(void);
void EXTI10_Callback(void);
void EXTI11_Callback(void);
void EXTI12_Callback(void);
void EXTI13_Callback(void);
void EXTI14_Callback(void);
void EXTI15_Callback(void);

void exti_function_set(EXTI_IRQ_E irq, void (*func)(void *), void *param);

#endif /* __EXTI_H__ */

