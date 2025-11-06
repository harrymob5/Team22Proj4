#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void USART2_Init(void);
void SysTick_Init(void);
void ADC_Init(void);

void send_string(const char *s);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void trigger_pulse(void);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void SysTick_Handler(void);

volatile uint8_t system_flag = 0;
volatile uint32_t adc_value = 0;
volatile uint32_t pulse_width_us = 0;

int main(void) {
    SystemCoreClockUpdate();
    GPIO_Init();
    USART2_Init();
    ADC_Init();
    TIM2_Init();
    TIM3_Init();
    SysTick_Init();

    send_string("System Initialized\r\n");

    while (1) {
        delay_ms(1000);
    }
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
}

void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
}

void USART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
}

void SysTick_Init(void) {
    SysTick_Config(SystemCoreClock / 1000);
}

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;
}

void TIM3_IRQHandler(void) {
    TIM3->SR &= ~TIM_SR_UIF;
}

void EXTI15_10_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR13;
}

void SysTick_Handler(void) {
    system_flag = 1;
}

void send_string(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

void delay_us(uint32_t us) {
    us *= (SystemCoreClock / 1000000) / 5;
    while (us--) __NOP();
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

void trigger_pulse(void) {
}