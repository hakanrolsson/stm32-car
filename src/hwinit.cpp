/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>
#include "stm32_loader.h"
#include "my_string.h"
#include "hwdefs.h"
#include "hwinit.h"
#include "sine_core.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE25_72MHZ]);

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_TIM1); //Main PWM
   rcc_periph_clock_enable(RCC_TIM2); //Scheduler
   rcc_periph_clock_enable(RCC_TIM3); //Rotor Encoder
   rcc_periph_clock_enable(RCC_TIM4); //Overcurrent / AUX PWM
   rcc_periph_clock_enable(RCC_TIM5); //DAC timing
   rcc_periph_clock_enable(RCC_DMA1);  //ADC, Encoder and UART receive
   rcc_periph_clock_enable(RCC_DMA2);  //DAC
   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO); //CAN
   rcc_periph_clock_enable(RCC_CAN1); //CAN
   rcc_periph_clock_enable(RCC_DAC);
}

void dac_setup()
{
   static uint8_t dacdata[256];

   dac_enable(DAC1, DAC_CHANNEL1);
   dac_set_trigger_source(DAC1, DAC_CR_TSEL1_T5);
   dac_trigger_enable(DAC1, DAC_CHANNEL1);
   dac_dma_enable(DAC1, DAC_CHANNEL1);
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO0);

   timer_set_prescaler(TIM5, 0);
   timer_set_period(TIM5, 63); //Wrap at 1.125 MHz
   //timer_one_shot_mode(TIM5);
   timer_direction_up(TIM5);
   timer_enable_update_event(TIM5);
   timer_set_master_mode(TIM5, TIM_CR2_MMS_UPDATE);
   timer_generate_event(TIM5, TIM_EGR_UG);
   timer_enable_counter(TIM5);

   for (int i = 0; i < sizeof(dacdata); i++)
      dacdata[i] = SineCore::Sine(i * 256) / 270 + 128;


   dma_channel_reset(DMA2, DMA_CHANNEL3);
   dma_set_read_from_memory(DMA2, DMA_CHANNEL3);
   dma_set_memory_address(DMA2, DMA_CHANNEL3, (uint32_t)dacdata);
   dma_set_peripheral_address(DMA2, DMA_CHANNEL3, (uint32_t)&DAC_DHR8R1(DAC1));
   dma_set_peripheral_size(DMA2, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA2, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
   dma_enable_circular_mode(DMA2, DMA_CHANNEL3);
   dma_set_number_of_data(DMA2, DMA_CHANNEL3, sizeof(dacdata));
   dma_enable_memory_increment_mode(DMA2, DMA_CHANNEL3);
   dma_enable_channel(DMA2, DMA_CHANNEL3);
}

void write_bootloader_pininit()
{
   struct pincommands *flashCommands = (struct pincommands *)PINDEF_ADDRESS;
   struct pincommands commands;

   memset32((int*)&commands, 0, PINDEF_NUMWORDS);

   //Keep vacuum pump off in bootloader
   commands.pindef[0].port = GPIOB;
   commands.pindef[0].pin = GPIO1;
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 0;
   commands.pindef[0].port = GPIOC;
   commands.pindef[0].pin = GPIO13;
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 0;

   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)&commands), PINDEF_NUMWORDS);
   commands.crc = crc;

   if (commands.crc != flashCommands->crc)
   {
      flash_unlock();
      flash_erase_page(PINDEF_ADDRESS);

      //Write flash including crc, therefor <=
      for (uint32_t idx = 0; idx <= PINDEF_NUMWORDS; idx++)
      {
         uint32_t* pData = ((uint32_t*)&commands) + idx;
         flash_program_word(PINDEF_ADDRESS + idx * sizeof(uint32_t), *pData);
      }
      flash_lock();
   }
}

/**
* Setup UART3 115200 8N1
*/
void usart_setup(void)
{
   gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
               GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

   usart_set_baudrate(TERM_USART, USART_BAUDRATE);
   usart_set_databits(TERM_USART, 8);
   usart_set_stopbits(TERM_USART, USART_STOPBITS_1);
   usart_set_mode(TERM_USART, USART_MODE_TX_RX);
   usart_set_parity(TERM_USART, USART_PARITY_NONE);
   usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);
   usart_enable_rx_dma(TERM_USART);
   usart_enable_tx_dma(TERM_USART);

   dma_channel_reset(DMA1, TERM_USART_DMATX);
   dma_set_read_from_memory(DMA1, TERM_USART_DMATX);
   dma_set_peripheral_address(DMA1, TERM_USART_DMATX, (uint32_t)&TERM_USART_DR);
   dma_set_peripheral_size(DMA1, TERM_USART_DMATX, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, TERM_USART_DMATX, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, TERM_USART_DMATX);

   dma_channel_reset(DMA1, TERM_USART_DMARX);
   dma_set_peripheral_address(DMA1, TERM_USART_DMARX, (uint32_t)&TERM_USART_DR);
   dma_set_peripheral_size(DMA1, TERM_USART_DMARX, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, TERM_USART_DMARX, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, TERM_USART_DMARX);
   dma_enable_channel(DMA1, TERM_USART_DMARX);

   usart_enable(TERM_USART);
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(PWM_TIMER_IRQ); //Main PWM
   nvic_set_priority(PWM_TIMER_IRQ, 1 << 4); //Set second-highest priority

   nvic_enable_irq(NVIC_TIM1_BRK_IRQ); //Emergency shut down
   nvic_set_priority(NVIC_TIM1_BRK_IRQ, 0); //Highest priority

   nvic_enable_irq(NVIC_EXTI2_IRQ); //Encoder Index pulse
   nvic_set_priority(NVIC_EXTI2_IRQ, 0); //Set highest priority

   nvic_enable_irq(NVIC_TIM2_IRQ); //Scheduler
   nvic_set_priority(NVIC_TIM2_IRQ, 0xe << 4); //second lowest priority

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4); //lowest priority

	nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf << 4); //lowest priority
}

void rtc_setup()
{
   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
   //62.5kHz / (624 + 1) = 100Hz
   rtc_auto_awake(RCC_HSE, 624); //10ms tick
   rtc_set_counter_val(0);
}

/**
* Setup main PWM timer and timer for generating over current
* reference values and external PWM
*/
void tim_setup()
{
   /*** Setup over/undercurrent and PWM output timer */
   timer_disable_counter(FUELGAUGE_TIMER);
   //edge aligned PWM
   timer_set_alignment(FUELGAUGE_TIMER, TIM_CR1_CMS_EDGE);
   timer_enable_preload(FUELGAUGE_TIMER);
   /* PWM mode 1 and preload enable */
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC4, TIM_OCM_PWM1);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC2);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC3);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC4);

   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC3);
   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC4);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC2);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC3);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC4);
   timer_generate_event(FUELGAUGE_TIMER, TIM_EGR_UG);
   timer_set_prescaler(FUELGAUGE_TIMER, 0);
   /* PWM frequency */
   timer_set_period(FUELGAUGE_TIMER, GAUGEMAX);
   timer_enable_counter(FUELGAUGE_TIMER);

   /** setup gpio */
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8 | GPIO9);

   /* Timer 1 for AVAS */
   timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
   timer_enable_preload(PWM_TIMER);

   timer_enable_oc_preload(PWM_TIMER, TIM_OC3);
   timer_set_oc_mode(PWM_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);

   timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);
   timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3N);
   timer_set_prescaler(PWM_TIMER, 10);

   /* PWM frequency */
   timer_set_period(PWM_TIMER, 40000);
   timer_set_repetition_counter(PWM_TIMER, 1);

   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_enable_counter(PWM_TIMER);
   timer_enable_break_main_output(PWM_TIMER);


   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO14 | GPIO15);

}

