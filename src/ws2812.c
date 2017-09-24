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

/* Adjust brightness of single color component
 * c: color component
 * b: brightness where 0xff is full bright, and 0 is off */
uint8_t color_bright_one(uint8_t c, uint8_t b)
{
	return ((((int)c) * (((int)b)+1)) >> 8 & 0xff);
}

/* Adjust brightness of u32 color.
 * c: RGBW Color in uint32_t format
 * b: brightness where 0xff is full bright, and 0 is off */
uint32_t color_brightness(uint32_t c, uint8_t b)
{
	return COLOR(color_bright_one(Red(c), b), color_bright_one(Green(c), b),
	             color_bright_one(Blue(c), b), color_bright_one(White(c), b));
}

uint32_t color_wheel(uint8_t angle)
{
	angle = 255 - angle;
	if (angle < 85) {
		return COLOR(255 - angle * 3, 0, angle * 3, 0);
	}
	if (angle < 170) {
		angle -= 85;
		return COLOR(0, angle * 3, 255 - angle * 3, 0);
	}
	angle -= 170;
	return COLOR(angle * 3, 255 - angle * 3, 0, 0);
}


void neopixel_init(struct neopixel *neo, uint8_t format, uint8_t num, uint8_t *pixels)
{
	neo->w_off = (format >> 6) & 0b11;
	neo->r_off = (format >> 4) & 0b11;
	neo->g_off = (format >> 2) & 0b11;
	neo->b_off = (format     ) & 0b11;
	neo->num_leds = num;
	neo->pixel_width = (neo->r_off == neo->w_off) ? 3 : 4;
	neo->pixels = pixels;
}

void neopixel_set_rgbw(struct neopixel *neo, uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	uint8_t *p;
	if (n >= neo->num_leds)
		return;

	p = &neo->pixels[n * neo->pixel_width];

	/* Write the color values.
	 * This is a *little* naive. It blindly writes 4 bytes, even if the
	 * pixel width is 3. However, it is always the white pixel omitted on
	 * 3-byte pixels, so by writing it first, no important data gets
	 * overwritten */
	p[neo->w_off] = w;
	p[neo->r_off] = r;
	p[neo->g_off] = g;
	p[neo->b_off] = b;
}

/**
 * neopixel_set_u32() - Set pixel colour using a single u32 value.
 *
 * u32 pixel format is always WRGB, regardless of actual layout used by the
 * pixels. This is so that portable code can be written without needing to
 * adapt for the pixel type.
 */
void neopixel_set_u32(struct neopixel *neo, uint8_t n, uint32_t c)
{
	neopixel_set_rgbw(neo, n, Red(c), Green(c), Blue(c), White(c));
}

/* Number of bytes to send in each 1/2 DMA buffer transfer. This directly
 * translates into the size of the DMA temporary buffer. The minimum is 4
 * bytes. This equals a DMA buffer size of 256 bytes (each byte transfer
 * requires 8 bytes in the DMA buffer x 4 bytes transfered x 4 channels x 2
 * half buffers) and will trigger the DMA interrupt at ~2KHz. Doubling the
 * number of transfer bytes will divide the interrupt frequency by 2, but will
 * also double the memory consumption (no free lunch ;-)
 */
#define WS2812_NUM_CHANS 4
#define BYTES_PER_LED (8)
#define LEDS_PER_PIXEL (4)
#define BYTES_PER_PIXEL (BYTES_PER_LED * LEDS_PER_PIXEL)
#define WS2812_DMA_TXFR_SIZE (BYTES_PER_PIXEL)
#define WS2812_DMA_BUF_SIZE (WS2812_NUM_CHANS * WS2812_DMA_TXFR_SIZE)

static union {
	uint8_t buffer[2*WS2812_DMA_BUF_SIZE];
	struct {
		uint8_t begin[WS2812_DMA_BUF_SIZE];
		uint8_t end[WS2812_DMA_BUF_SIZE];
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

static void fillBits(uint8_t *buffer, uint8_t val)
{
	int i;

	for(i = 0; i < 8; i++, buffer += WS2812_NUM_CHANS)
		*buffer = ((val << i) & 0x80) ? 17 : 9;
}

static int led_index = 0;
static int led_data_size = 0;
static uint8_t *led_data = NULL;

static void fillLeds(uint8_t *buffer)
{
	int i, j;

	memset(buffer, 0, WS2812_DMA_BUF_SIZE);
	for (i = 0; i < LEDS_PER_PIXEL; i++, led_index++, buffer += BYTES_PER_LED * WS2812_NUM_CHANS) {
		if (led_index < led_data_size)
			for (j = 0; j < WS2812_NUM_CHANS; j++)
				fillBits(buffer + j, led_data[(led_data_size * j) + led_index]);
	}
}

void ws2812_send(uint8_t *p, uint16_t n)
{
	if (!n)
		return;

	// Set interrupt context ...
	led_index = 0;
	led_data = p;
	led_data_size = n;

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

	if (led_data_size == 0) {
		TIM_Cmd(TIM2, DISABLE);
		DMA_Cmd(DMA1_Channel5, DISABLE);
	}

	if (DMA_GetITStatus(DMA1_IT_HT5)) {
		DMA_ClearITPendingBit(DMA1_IT_HT5);
		buffer = led_dma.begin;
	}

	if (DMA_GetITStatus(DMA1_IT_TC5)) {
		DMA_ClearITPendingBit(DMA1_IT_TC5);
		buffer = led_dma.end;
	}

	fillLeds(buffer);

	if (led_index >= led_data_size+WS2812_DMA_TXFR_SIZE) {
		TIM_Cmd(TIM2, DISABLE);			// disable Timer 1
		DMA_Cmd(DMA1_Channel5, DISABLE); 	// disable DMA channel 2
		led_data_size = 0;
	}
}

