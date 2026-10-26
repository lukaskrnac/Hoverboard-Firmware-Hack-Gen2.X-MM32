// GCC startup for MM32SPIN05/06: vector table, .data/.bss init, then SystemInit() and main().
// Vector order follows the MindMotion Keil startup (startup_mm32spin06xx_s_keil.s).
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void SystemInit(void);
extern int main(void);

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;)
        *dst++ = 0;
    SystemInit();
    main();
    for (;;);
}

void Default_Handler(void)
{
    for (;;);
}

#define WEAK_IRQ(name) void name(void) __attribute__((weak, alias("Default_Handler")))
WEAK_IRQ(NMI_Handler);
WEAK_IRQ(HardFault_Handler);
WEAK_IRQ(SVC_Handler);
WEAK_IRQ(PendSV_Handler);
WEAK_IRQ(SysTick_Handler);
WEAK_IRQ(WWDG_IRQHandler);
WEAK_IRQ(PVD_IRQHandler);
WEAK_IRQ(RTC_BKP_IRQHandler);
WEAK_IRQ(FLASH_IRQHandler);
WEAK_IRQ(RCC_CRS_IRQHandler);
WEAK_IRQ(EXTI0_1_IRQHandler);
WEAK_IRQ(EXTI2_3_IRQHandler);
WEAK_IRQ(EXTI4_15_IRQHandler);
WEAK_IRQ(HWDIV_IRQHandler);
WEAK_IRQ(DMA1_Channel1_IRQHandler);
WEAK_IRQ(DMA1_Channel2_3_IRQHandler);
WEAK_IRQ(DMA1_Channel4_5_IRQHandler);
WEAK_IRQ(ADC1_COMP_IRQHandler);
WEAK_IRQ(TIM1_BRK_UP_TRG_COM_IRQHandler);
WEAK_IRQ(TIM1_CC_IRQHandler);
WEAK_IRQ(TIM2_IRQHandler);
WEAK_IRQ(TIM3_IRQHandler);
WEAK_IRQ(TIM14_IRQHandler);
WEAK_IRQ(TIM16_IRQHandler);
WEAK_IRQ(TIM17_IRQHandler);
WEAK_IRQ(I2C1_IRQHandler);
WEAK_IRQ(SPI1_IRQHandler);
WEAK_IRQ(SPI2_IRQHandler);
WEAK_IRQ(UART1_IRQHandler);
WEAK_IRQ(UART2_IRQHandler);
WEAK_IRQ(CAN_IRQHandler);

__attribute__((section(".isr_vector"), used))
static void (*const vectors[48])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    SVC_Handler,
    0, 0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,                // 0
    PVD_IRQHandler,
    RTC_BKP_IRQHandler,
    FLASH_IRQHandler,
    RCC_CRS_IRQHandler,
    EXTI0_1_IRQHandler,             // 5
    EXTI2_3_IRQHandler,
    EXTI4_15_IRQHandler,
    HWDIV_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_3_IRQHandler,     // 10
    DMA1_Channel4_5_IRQHandler,
    ADC1_COMP_IRQHandler,
    TIM1_BRK_UP_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,                // 15
    TIM3_IRQHandler,
    0, 0,
    TIM14_IRQHandler,
    0,                              // 20
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    I2C1_IRQHandler,
    0,
    SPI1_IRQHandler,                // 25
    SPI2_IRQHandler,
    UART1_IRQHandler,
    UART2_IRQHandler,
    0,
    CAN_IRQHandler,                 // 30
    0,
};
