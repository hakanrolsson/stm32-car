#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMRST    RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define FUELGAUGE_TIMER    TIM4
//Maximum value for over current limit timer
#define GAUGEMAX           65535

//PCON (park-lock actuator command) on TIM1_CH2N (PB14), matching the real HV ECU's
//48.81 Hz / 20.488 ms period (BB07D7760 capture): line high 20% = ENGAGE, 40% = RELEASE.
//On the PriusG2 board PB14 goes through an inverting open-collector (ULN2003) stage
//(bench 20261005: 40% pin-high with a pull-up gave 60% high at P2.11), so the pin is
//high for the INVERSE: 80% = ENGAGE, 60% = RELEASE. The PCON wire needs a pull-up to 5 V.
#define PCON_PSC           71      //72 MHz / 72 = 1 MHz timer clock
#define PCON_PERIOD        20487   //20488 counts @ 1 MHz = 20.488 ms
#define PCON_ENGAGE        16390   //pin high 80% of 20488 -> line high 20%
#define PCON_RELEASE       12293   //pin high 60% of 20488 -> line high 40%

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
