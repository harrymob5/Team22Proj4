#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define POT_PIN        0   // PA0 ADC1_IN0
#define ENCODER_PIN    1   // PA1 (EXTI1) — single-channel encoder pulses
#define BTN_PIN        13  // PC13
#define LEFT_PWM_PIN   8   // PC8 TIM3_CH3
#define RIGHT_PWM_PIN  9   // PC9 TIM3_CH4

#define PWM_NEUTRAL_US   1500u
#define PWM_CW_MIN_US    1280u
#define PWM_CW_MAX_US    1480u
#define PWM_CCW_MIN_US   1520u
#define PWM_CCW_MAX_US   1720u

#define ENC_PULSES_PER_REV  20u   // set to your encoder's pulses per revolution
#define DEBOUNCE_MS         50u

volatile uint16_t last_adc = 0;
volatile uint32_t pwm_us = PWM_NEUTRAL_US;
volatile uint8_t dir_state = 1; // 0=CW,1=STOP,2=CCW
volatile bool stopped = true;

volatile uint32_t encoder_total = 0;      // cumulative pulses
volatile uint32_t encoder_snapshot = 0;   
volatile float rpm = 0.0f;
volatile int32_t rpm_tenths = 0;          // rpm * 10 for SSD

volatile uint8_t ssd_digit = 0;
volatile uint8_t tim2_halfms_count = 0;

volatile uint32_t ms_ticks = 0;
volatile uint32_t last_button_ms = 0;
volatile uint8_t one_second_flag = 0;

volatile uint8_t tim5_250ms_flag = 0;

void GPIO_Init(void);
void ADC1_Init(void);
void USART2_Init(void);
void TIM3_PWM_Init(void);
void TIM2_SSD_Init(void);
void TIM5_250ms_Init(void);
void SysTick_Init(void);

uint16_t read_ADC(void);
uint32_t map_adc_to_pwm(uint16_t adc, uint8_t dir);
void send_uart(const char *s);

int main(void) {
    SystemCoreClockUpdate();

    GPIO_Init();
    ADC1_Init();
    USART2_Init();
    SSD_init();

    TIM3_PWM_Init();    // PC8/PC9 PWM
    TIM2_SSD_Init();    // SSD refresh 0.5 ms
    TIM5_250ms_Init();  // 250 ms RPM window
    SysTick_Init();     // 1 ms tick

    // neutral
    TIM3->CCR3 = PWM_NEUTRAL_US;
    TIM3->CCR4 = PWM_NEUTRAL_US;
    pwm_us = PWM_NEUTRAL_US;

    send_uart("Project4A: started\r\n");

    while (1) {
        // sample ADC
        last_adc = read_ADC();

        // map ADC to PWM based on dir_state
        pwm_us = map_adc_to_pwm(last_adc, dir_state);

        // apply PWM to both motors (left/right)
        TIM3->CCR3 = pwm_us;
        TIM3->CCR4 = pwm_us;

        // sleep until interrupts (low power)
        __WFI();

        // On 1-second flag, serial printed inside SysTick handler logic (see below)
    }
}


void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PA0 analogue handled in ADC init

    // PA1 encoder pin -> input with pull-down, route EXTI1
    GPIOA->MODER &= ~(3u << (ENCODER_PIN * 2)); // input
    GPIOA->PUPDR &= ~(3u << (ENCODER_PIN * 2));
    GPIOA->PUPDR |= (1u << (ENCODER_PIN * 2)); // pull-up (or change to pulldown as needed)

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
    EXTI->IMR |= EXTI_IMR_IM1;
    EXTI->RTSR |= EXTI_RTSR_TR1; // rising edge
    NVIC_EnableIRQ(EXTI1_IRQn);

    // Button PC13 input pull-up + EXTI
    GPIOC->MODER &= ~(3u << (BTN_PIN * 2));
    GPIOC->PUPDR &= ~(3u << (BTN_PIN * 2));
    GPIOC->PUPDR |= (1u << (BTN_PIN * 2)); // pull-up
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13; // falling
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // PC8, PC9 AF2 (TIM3 CH3/CH4) - set in TIM3_PWM_Init (re-enabled there)
}

void ADC1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // PA0 analog
    GPIOA->MODER &= ~(3u << (POT_PIN * 2));
    GPIOA->MODER |= (3u << (POT_PIN * 2)); // analog

    ADC->CCR = 0;
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR3 = POT_PIN; // channel 0
    ADC1->SMPR2 = (7u << (3 * POT_PIN));
    ADC1->CR2 |= ADC_CR2_ADON;
}

void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2/PA3 AF7
    GPIOA->MODER &= ~(0xFFu << (2*2));
    GPIOA->MODER |=  (0xAAu << (2*2));
    GPIOA->AFR[0] |= (0x7700u);
    USART2->BRR = (SystemCoreClock / 115200U);
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void TIM3_PWM_Init(void) {
    // TIM3 channels 3 and 4 -> PC8, PC9 (AF2)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER &= ~(3u << (LEFT_PWM_PIN * 2));
    GPIOC->MODER |= (2u << (LEFT_PWM_PIN * 2));
    GPIOC->AFR[1] &= ~(0xFu << ((LEFT_PWM_PIN - 8) * 4));
    GPIOC->AFR[1] |=  (0x2u << ((LEFT_PWM_PIN - 8) * 4));

    GPIOC->MODER &= ~(3u << (RIGHT_PWM_PIN * 2));
    GPIOC->MODER |= (2u << (RIGHT_PWM_PIN * 2));
    GPIOC->AFR[1] &= ~(0xFu << ((RIGHT_PWM_PIN - 8) * 4));
    GPIOC->AFR[1] |=  (0x2u << ((RIGHT_PWM_PIN - 8) * 4));

    // 1 us tick
    TIM3->PSC = (SystemCoreClock / 1000000U) - 1;
    TIM3->ARR = 19999; // 20 ms

    // CH3 & CH4 PWM mode 1, preload
    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos);
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    TIM3->CCR3 = PWM_NEUTRAL_US;
    TIM3->CCR4 = PWM_NEUTRAL_US;

    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM2_SSD_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (SystemCoreClock / 1000000U) - 1; // 1 us ticks
    TIM2->ARR = 500 - 1; // 0.5 ms
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM5_250ms_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->PSC = (SystemCoreClock / 1000U) - 1; // 1 ms ticks
    TIM5->ARR = 250 - 1; // 250 ms
    TIM5->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 |= TIM_CR1_CEN;
}

void SysTick_Init(void) {
    // 1 ms tick
    SysTick_Config(SystemCoreClock / 1000U);
}

/* EXTI1: encoder pulse (rising edge) */
void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        encoder_total++;
    }
}
// Button Hnadler
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13; // clear
        uint32_t now = ms_ticks;
        if ((now - last_button_ms) >= DEBOUNCE_MS) {
            last_button_ms = now;
            // cycle: 0=CW -> 1=STOP -> 2=CCW -> 3=STOP -> ...
            static uint8_t state = 1;
            state = (state + 1) & 3;
            if (state == 0) { dir_state = 0; stopped = false; }
            else if (state == 1) { dir_state = 1; stopped = true; }
            else if (state == 2) { dir_state = 2; stopped = false; }
            else { dir_state = 1; stopped = true; }
            if (dir_state == 1) {
                TIM3->CCR3 = PWM_NEUTRAL_US;
                TIM3->CCR4 = PWM_NEUTRAL_US;
            }
        }
    }
}

// -----TIM2: SSD refresh at 0.5 ms----- //
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        tim2_halfms_count++;
        if ((tim2_halfms_count & 3) == 0) { // every 2 ms
            SSD_update(ssd_digit, (int)rpm_tenths, 1); //(tenths)
            ssd_digit = (ssd_digit + 1) % 4;
        }
    }
}

// -----TIM5: 250 ms window, computes RPM----- //
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;
        uint32_t now = encoder_total;
        uint32_t delta = now - encoder_snapshot;
        encoder_snapshot = now;
        // rpm = delta * 240 / PPR
        float r = ((float)delta * 240.0f) / (float)ENC_PULSES_PER_REV;
        if (r < 0.0f) r = 0.0f;
        rpm = r;
        rpm_tenths = (int32_t)(rpm * 10.0f + 0.5f);
        tim5_250ms_flag = 1;
    }
}

void SysTick_Handler(void) {
    ms_ticks++;
    if ((ms_ticks % 1000U) == 0U) {
        one_second_flag = 1;
    }
    
    if (one_second_flag) {
        one_second_flag = 0;
        // Prepare and send telemetry
        char out[128];
        const char *dstr = (dir_state==0) ? "CW" : (dir_state==2) ? "CCW" : "STOP";
        // use rpm computed by TIM5 (last 250ms window)
        float rpm_local = rpm;
        snprintf(out, sizeof(out),
                 "ADC value: %u, dir: %s,servo (us): %lu, rpm: %.3f\r\n",
                 (unsigned)last_adc, dstr, (unsigned)pwm_us, rpm_local);
        send_uart(out);
    }
}

//-----Helper Functions-----//
uint16_t read_ADC(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)(ADC1->DR & 0x0FFF);
}

uint32_t map_adc_to_pwm(uint16_t adc, uint8_t dir) {
    if (dir == 1) return PWM_NEUTRAL_US;
    if (dir == 0) {
        // CW: 0 -> 1280 ; 4095 -> 1480
        int32_t span = (int32_t)PWM_CW_MAX_US - (int32_t)PWM_CW_MIN_US;
        int32_t pwm = (int32_t)PWM_CW_MIN_US + (int32_t)((int64_t)adc * span / 4095LL);
        if (pwm < 1000) pwm = 1000;
        if (pwm > 2000) pwm = 2000;
        return (uint32_t)pwm;
    } else {
        // CCW: 0 -> 1520 ; 4095 -> 1720
        int32_t span = (int32_t)PWM_CCW_MAX_US - (int32_t)PWM_CCW_MIN_US;
        int32_t pwm = (int32_t)PWM_CCW_MIN_US + (int32_t)((int64_t)adc * span / 4095LL);
        if (pwm < 1000) pwm = 1000;
        if (pwm > 2000) pwm = 2000;
        return (uint32_t)pwm;
    }
}

void send_uart(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}