/**
 * STM32F3 NeoPixel via PWM driver using TIM and DMA peripherals
 *
 * Originally Crazyflie control firmware, which in turn is
 * mostly from Elia's electonic blog: http://eliaselectronics.com/driving-a-ws2812-rgb-led-with-an-stm32/
 *
 * Copyright (C) 2011-2014 Bitcraze AB
 * Copyright (C) Grant Likely <grant.likely@secretlab.ca>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "stm32f30x.h"
#include "ws2812.h"

// The minimum is to have 2 leds (1 per half buffer) in the buffer, this
// consume 42Bytes and will trigger the DMA interrupt at ~2KHz.
// Putting 2 there will divide by 2 the interrupt frequency but will also 
// double the memory consumption (no free lunch ;-)
#define LED_PER_HALF 1

static union {
	uint8_t buffer[4*2*LED_PER_HALF*32];
	struct {
		uint8_t begin[4*LED_PER_HALF*32];
		uint8_t end[4*LED_PER_HALF*32];
	} __attribute__((packed));
} led_dma;

void ws2812_init(void)
{
	uint16_t PrescalerValue = (uint16_t) (72000000 / 24000000) - 1;
	/* GPIOA Configuration: TIM3 Channel 1 as alternate function push-pull */
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_Speed = GPIO_Speed_50MHz,
	};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {
		.TIM_Period = 29, // 800kHz
		.TIM_Prescaler = PrescalerValue,
		.TIM_ClockDivision = 0,
		.TIM_CounterMode = TIM_CounterMode_Up,
	};
	TIM_OCInitTypeDef TIM_OCInitStructure = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 17,
	//	.TIM_OCPolarity = TIM_OCPolarity_High,
	//	.TIM_OCNPolarity = TIM_OCNPolarity_High,
	//	.TIM_OCNIdleState = TIM_OCNIdleState_Set,
	};
	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_PeripheralBaseAddr = (uint32_t)&TIM2->DMAR,
		.DMA_MemoryBaseAddr = (uint32_t)led_dma.buffer,	// this is the buffer memory 
		.DMA_DIR = DMA_DIR_PeripheralDST,		// data shifted from memory to peripheral
		.DMA_BufferSize = sizeof(led_dma.buffer),
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,		// automatically increase buffer index
		.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_Mode = DMA_Mode_Circular,
		.DMA_Priority = DMA_Priority_High,
		.DMA_M2M = DMA_M2M_Disable,
	};
	NVIC_InitTypeDef NVIC_InitStructure = {
		.NVIC_IRQChannel = DMA1_Channel5_IRQn,
		.NVIC_IRQChannelPreemptionPriority = 9,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE,
	};


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Compute the prescaler value */

	/* Time base configuration */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM Mode configuration: Channels 1-4 */
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);           // enable Timer 1

	/* configure DMA */
	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channel5 Config */
	DMA_DeInit(DMA1_Channel5);

	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, ENABLE);

	/* TIM2 CC1 DMA Request enable */
	TIM_DMAConfig(TIM2, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
}

void ws2812_set_rgbw(uint8_t *p, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	*p++ = r;
	*p++ = g;
	*p++ = b;
	*p++ = w;
}

void ws2812_set_u32(uint8_t *p, uint32_t c)
{
	ws2812_set_rgbw(p, c >> 16, c >> 8, c, c >> 24);
}

static void fillLed(uint8_t *buffer, uint8_t *color)
{
	int i;

	for(i=0; i<8; i++, buffer+=4) // GREEN data
		*buffer = ((color[1]<<i) & 0x80)?17:9;
	for(i=0; i<8; i++, buffer+=4) // RED
		*buffer = ((color[0]<<i) & 0x80)?17:9;
	for(i=0; i<8; i++, buffer+=4) // BLUE
		*buffer = ((color[2]<<i) & 0x80)?17:9;
	for(i=0; i<8; i++, buffer+=4) // WHITE
		*buffer = ((color[3]<<i) & 0x80)?17:9;
}

static int current_led = 0;
static int total_led = 0;
static uint8_t (*color_led)[4] = NULL;

static void fillLeds(uint8_t *buffer)
{
	int i;

	memset(buffer, 0, 32*4);
	for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++, buffer+=32*4) {
		if (current_led<total_led) {
			fillLed(&buffer[0], color_led[current_led]);
			fillLed(&buffer[1], color_led[current_led+total_led]);
			fillLed(&buffer[2], color_led[current_led+total_led*2]);
			fillLed(&buffer[3], color_led[current_led+total_led*3]);
		}
	}
}

void ws2812_send(uint8_t (*color)[4], int len)
{
	if(len<1)
		return;

	//Wait for previous transfer to be finished
	//xSemaphoreTake(allLedDone, portMAX_DELAY);

	// Set interrupt context ...
	current_led = 0;
	total_led = len;
	color_led = color;

	/* Preload both buffers with data */
	fillLeds(led_dma.begin);
	fillLeds(led_dma.end);

	// load number of bytes to be transferred, enable the DMA channel
	// and Go!
	DMA1_Channel5->CNDTR = sizeof(led_dma.buffer);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void DMA1_Channel5_IRQHandler(void)
{
	uint8_t * buffer;

	if (total_led == 0)
	{
		TIM_Cmd(TIM2, DISABLE);
		DMA_Cmd(DMA1_Channel5, DISABLE);
	}

	if (DMA_GetITStatus(DMA1_IT_HT5))
	{
		DMA_ClearITPendingBit(DMA1_IT_HT5);
		buffer = led_dma.begin;
	}

	if (DMA_GetITStatus(DMA1_IT_TC5))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC5);
		buffer = led_dma.end;
	}

	fillLeds(buffer);

	if (current_led >= total_led+2) {
		TIM_Cmd(TIM2, DISABLE);			// disable Timer 1
		DMA_Cmd(DMA1_Channel5, DISABLE); 	// disable DMA channel 2
		total_led = 0;
	}
}

