/**
  ******************************************************************************
  * @file    USB_Example/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include <stdio.h>
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
#include "math.h"
#include "usb_istr.h"
#include "stm32f30x_it.h"
#include "usb_desc.h"
#include "ws2812.h"
#include "acc.h"


/* LED numbers for PWM channels. Note: LED numbers starts at '1'. 0 is reserved
 * to mean 'no LED' */
#define NEOPIXELS_PER_CH (32)		/* Up to 32 pixels on each channel */
#define NEOPIXEL_SIZE (4)			/* 4 bytes per pixel */
#define NEOPIXEL_CH1 (1)
#define NEOPIXEL_CH2 (NEOPIXELS_PER_CH+1)
#define NEOPIXEL_CH3 (NEOPIXELS_PER_CH*2+1)
#define NEOPIXEL_CH4 (NEOPIXELS_PER_CH*3+1)
#define NEOPIXEL_CH_COUNT (4)
#define NEOPIXEL_BUFSIZE (NEOPIXELS_PER_CH * NEOPIXEL_SIZE)

/* LED numbers assigned to each player's control cluster */
#define PLR1_LEDS      (NEOPIXEL_CH1)
#define PLR1_SS_LEDS   (NEOPIXEL_CH1+14)
#define PLR2_LEDS      (NEOPIXEL_CH2+10)
#define PLR2_SS_LEDS   (NEOPIXEL_CH2)
#define PLR3_LEDS      (NEOPIXEL_CH1+6)
#define PLR3_SS_LEDS   (NEOPIXEL_CH1+12)
#define PLR4_LEDS      (NEOPIXEL_CH2+4)
#define PLR4_SS_LEDS   (NEOPIXEL_CH2+2)
#define TRACKBALL_LEDS (NEOPIXEL_CH2+16)

/* Player color assignment */
#define PLR1_COLOR COLOR(0,0xff,0,0)
#define PLR2_COLOR COLOR(0xff,0,0,0)
#define PLR3_COLOR COLOR(0,0,0xff,0)
#define PLR4_COLOR COLOR(0xff,0xff,0,0)

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;

struct neopixel neo1;
uint8_t neopixel_buf[NEOPIXEL_BUFSIZE * NEOPIXEL_CH_COUNT];

struct neopixel_meta {
	int8_t x, y;
	int8_t animate_factor;
};

/* store pixel location with 4mm resolution so it fits in a byte */
#define PIXEL_META_INIT(_x, _y) { .x = (_x)/4, .y = (_y)/4 }

struct neopixel_meta neopixel_meta[] = {
	/* Player 1 */
	[PLR1_LEDS]     = PIXEL_META_INIT(-123, 42),
	[PLR1_LEDS + 1] = PIXEL_META_INIT(-127,  3),
	[PLR1_LEDS + 2] = PIXEL_META_INIT(-158, 46),
	[PLR1_LEDS + 3] = PIXEL_META_INIT(-162,  6),
	[PLR1_LEDS + 4] = PIXEL_META_INIT(-190, 28),
	[PLR1_LEDS + 5] = PIXEL_META_INIT(-195, -10),
	[PLR1_SS_LEDS]     = PIXEL_META_INIT(-196, 168),
	[PLR1_SS_LEDS + 1] = PIXEL_META_INIT(-156, 168),

	/* Player 2 */
	[PLR2_LEDS]     = PIXEL_META_INIT(255, 55),
	[PLR2_LEDS + 1] = PIXEL_META_INIT(259, 17),
	[PLR2_LEDS + 2] = PIXEL_META_INIT(219, 52),
	[PLR2_LEDS + 3] = PIXEL_META_INIT(223, 13),
	[PLR2_LEDS + 4] = PIXEL_META_INIT(190, 28),
	[PLR2_LEDS + 5] = PIXEL_META_INIT(195, -11),
	[PLR2_SS_LEDS]     = PIXEL_META_INIT(156, 168),
	[PLR2_SS_LEDS + 1] = PIXEL_META_INIT(196, 168),

	/* Player 3 */
	[PLR3_LEDS]     = PIXEL_META_INIT(-356, 72),
	[PLR3_LEDS + 1] = PIXEL_META_INIT(-365, 44),
	[PLR3_LEDS + 2] = PIXEL_META_INIT(-391, 81),
	[PLR3_LEDS + 3] = PIXEL_META_INIT(-401, 43),
	[PLR3_LEDS + 4] = PIXEL_META_INIT(-426, 69),
	[PLR3_LEDS + 5] = PIXEL_META_INIT(-435, 30),
	[PLR3_SS_LEDS]     = PIXEL_META_INIT(-422, 168),
	[PLR3_SS_LEDS + 1] = PIXEL_META_INIT(-382, 168),

	/* Player 4 */
	[PLR4_LEDS]     = PIXEL_META_INIT(486, 103),
	[PLR4_LEDS + 1] = PIXEL_META_INIT(494, 65),
	[PLR4_LEDS + 2] = PIXEL_META_INIT(405, 95),
	[PLR4_LEDS + 3] = PIXEL_META_INIT(459, 57),
	[PLR4_LEDS + 4] = PIXEL_META_INIT(426, 68),
	[PLR4_LEDS + 5] = PIXEL_META_INIT(434, 30),
	[PLR4_SS_LEDS]     = PIXEL_META_INIT(382, 168),
	[PLR4_SS_LEDS + 1] = PIXEL_META_INIT(422, 168),

	/* The trackball is lit with 7 LEDs, and it is considered the center
	 * of the board */
	[TRACKBALL_LEDS...TRACKBALL_LEDS+6] = PIXEL_META_INIT(0, 0),
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define for_each_array_entry(a, i, ap) \
	for (i = 0, ap = &a[0]; i < ARRAY_SIZE(a); i++, ap++)

enum effect_mode {
	EFFECT_STATIC = 0,
	EFFECT_CIRCLE_IN,
	EFFECT_CIRCLE_OUT,
	EFFECT_UP,
	EFFECT_DOWN,
	EFFECT_LEFT,
	EFFECT_RIGHT,
	EFFECT_SQUARE_IN,
	EFFECT_SQUARE_OUT,
	NUM_EFFECTS,
};

__IO uint8_t DataReady = 0;
__IO bool usb_transfer_pending = 0;

/* Input report buffers */
/* Report ID, Buttons, X, Y, change_flag */
#if (NUM_JOYSTICKS >= 1)
static int8_t gamepad1_report[5] = {1, 0, 0, 0, 0};
#endif
#if (NUM_JOYSTICKS >= 2)
static int8_t gamepad2_report[5] = {2, 0, 0, 0, 0};
#endif
#if (NUM_JOYSTICKS >= 3)
static int8_t gamepad3_report[5] = {3, 0, 0, 0, 0};
#endif
#if (NUM_JOYSTICKS >= 4)
static int8_t gamepad4_report[5] = {4, 0, 0, 0, 0};
#endif
static int8_t trackball_report[5] = {5, 0, 0, 0, 0};

/* Gamepad GPIO defintions */
struct gpio {
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t led;
};

struct gamepad_cfg {
	uint32_t color;
	struct gpio x[2];
	struct gpio y[2];
	struct gpio btns[8];
	int8_t *report;
};

const struct gamepad_cfg gamepads[NUM_JOYSTICKS + 1] = {
#if (NUM_JOYSTICKS >= 1)
	{
		.x = {{GPIOB, GPIO_Pin_12}, {GPIOB, GPIO_Pin_14},},
		.y = {{GPIOD, GPIO_Pin_8}, {GPIOD, GPIO_Pin_10},},
		.btns = {
			{GPIOD, GPIO_Pin_11, PLR1_LEDS + 1},	/* A */
			{GPIOD, GPIO_Pin_9,  PLR1_LEDS + 3},	/* B */
			{GPIOB, GPIO_Pin_15, PLR1_LEDS + 2},	/* X */
			{GPIOB, GPIO_Pin_13, PLR1_LEDS + 5},	/* Y */
			{GPIOB, GPIO_Pin_11, PLR1_LEDS + 4},	/* L */
			{GPIOB, GPIO_Pin_10, PLR1_LEDS},	/* R */
			{GPIOD, GPIO_Pin_15, PLR1_SS_LEDS},	/* Select */
			{GPIOD, GPIO_Pin_14, PLR1_SS_LEDS + 1},	/* Start */
		},
		.report = gamepad1_report,
		.color = PLR1_COLOR,
	},
#endif
#if (NUM_JOYSTICKS >= 2)
	{
		.x = {{GPIOD, GPIO_Pin_2}, {GPIOD, GPIO_Pin_0}},
		.y = {{GPIOC, GPIO_Pin_11}, {GPIOA, GPIO_Pin_15}},
		.btns = {
			{GPIOC, GPIO_Pin_9,  PLR2_LEDS + 1},	/* A */
			{GPIOC, GPIO_Pin_8,  PLR2_LEDS + 3},	/* B */
			{GPIOA, GPIO_Pin_8,  PLR2_LEDS + 2},	/* X */
			{GPIOF, GPIO_Pin_6,  PLR2_LEDS + 5},	/* Y */
			{GPIOC, GPIO_Pin_10, PLR2_LEDS + 4},	/* L */
			{GPIOC, GPIO_Pin_12, PLR2_LEDS},	/* R */
			{GPIOD, GPIO_Pin_4,  PLR2_SS_LEDS},	/* Select */
			{GPIOD, GPIO_Pin_6,  PLR2_SS_LEDS + 1},	/* Start */
		},
		.report = gamepad2_report,
		.color = PLR2_COLOR,
	},
#endif
#if (NUM_JOYSTICKS >= 3)
	{
		.x = {{GPIOF, GPIO_Pin_2}, {GPIOA, GPIO_Pin_4},},
		.y = {{GPIOA, GPIO_Pin_9}, {GPIOB, GPIO_Pin_0},},
		.btns = {
			{GPIOE, GPIO_Pin_7,  PLR3_LEDS + 1},	/* A */
			{GPIOB, GPIO_Pin_1,  PLR3_LEDS + 3},	/* B */
			{GPIOA, GPIO_Pin_10, PLR3_LEDS + 2},	/* X */
			{GPIOF, GPIO_Pin_4,  PLR3_LEDS + 5},	/* Y */
			{GPIOC, GPIO_Pin_3,  PLR3_LEDS + 4},	/* L */
			{GPIOC, GPIO_Pin_1,  PLR3_LEDS},	/* R */
			{GPIOC, GPIO_Pin_2,  PLR3_SS_LEDS},	/* Select */
			{GPIOC, GPIO_Pin_0,  PLR3_SS_LEDS + 1},	/* Start */
		},
		.report = gamepad3_report,
		.color = PLR3_COLOR,
	},
#endif
#if (NUM_JOYSTICKS >= 4)
	{
		.x = {{GPIOF, GPIO_Pin_10}, {GPIOC, GPIO_Pin_13},},
		.y = {{GPIOB, GPIO_Pin_9}, {GPIOB, GPIO_Pin_5},},
		.btns = {
			{GPIOD, GPIO_Pin_1,  PLR4_LEDS + 1},	/* A */
			{GPIOD, GPIO_Pin_3,  PLR4_LEDS + 3},	/* B */
			{GPIOD, GPIO_Pin_5,  PLR4_LEDS + 2},	/* X */
			{GPIOD, GPIO_Pin_7,  PLR4_LEDS + 5},	/* Y */
			{GPIOB, GPIO_Pin_4,  PLR4_LEDS + 4},	/* L */
			{GPIOB, GPIO_Pin_8,  PLR4_LEDS},	/* R */
			{GPIOF, GPIO_Pin_9,  PLR4_SS_LEDS},	/* Select */
			{GPIOE, GPIO_Pin_6,  PLR4_SS_LEDS + 1},	/* Start */
		},
		.report = gamepad4_report,
		.color = PLR4_COLOR,
	},
#endif
	{
		.btns = {
			{GPIOF, GPIO_Pin_6,  TRACKBALL_LEDS},	/* Left */
			{GPIOC, GPIO_Pin_8,  TRACKBALL_LEDS + 1},	/* Right */
			{GPIOC, GPIO_Pin_9,  TRACKBALL_LEDS + 2},	/* Middle */
			{.led = TRACKBALL_LEDS + 3},
			{.led = TRACKBALL_LEDS + 4},
			{.led = TRACKBALL_LEDS + 5},
			{.led = TRACKBALL_LEDS + 6},
		},
		.report = trackball_report,
	}
};

enum effect_mode curr_mode = EFFECT_STATIC;

void effects_prepare(enum effect_mode mode)
{
	int i, factor;
	struct neopixel_meta *nmp;

	mode = mode % NUM_EFFECTS;
	curr_mode = mode;

	switch (mode) {
	case EFFECT_UP:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = nmp->y;
		break;
	case EFFECT_DOWN:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = -nmp->y;
		break;
	case EFFECT_LEFT:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = nmp->x;
		break;
	case EFFECT_RIGHT:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = -nmp->x;
		break;
	case EFFECT_CIRCLE_IN:
		for_each_array_entry(neopixel_meta, i, nmp) {
			factor = sqrt(((int)nmp->x * nmp->x) + ((int)nmp->y * nmp->y));
			nmp->animate_factor = factor;
		}
		break;
	case EFFECT_CIRCLE_OUT:
		for_each_array_entry(neopixel_meta, i, nmp) {
			factor = sqrt(((int)nmp->x * nmp->x) + ((int)nmp->y * nmp->y));
			nmp->animate_factor = -factor;
		}
		break;
	case EFFECT_SQUARE_IN:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = (nmp->x < nmp->y) ? nmp->y : nmp->x;
		break;
	case EFFECT_SQUARE_OUT:
		for_each_array_entry(neopixel_meta, i, nmp)
			nmp->animate_factor = (nmp->x < nmp->y) ? -nmp->y : -nmp->x;
		break;
	default:
		break;
	}
}

void effects_animate(int index)
{
	const struct gamepad_cfg *gpc;
	struct neopixel_meta *nmp;
	int i, j, led;

	switch(curr_mode) {
	case EFFECT_STATIC:
		for_each_array_entry(gamepads, i, gpc) {
			for (j = 0; j < 8; j++) {
				led = gpc->btns[j].led;
				if (led)
					neopixel_set_u32(&neo1, led-1, gpc->color);
			}
		}
		break;
	default:
		for_each_array_entry(neopixel_meta, i, nmp)
			if (i)
				neopixel_set_u32(&neo1, i-1, color_wheel((nmp->animate_factor<<2) + (index)));
	}
}

void gpio_init_input(const struct gpio *gpio)
{
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Pin = gpio->pin,
		.GPIO_Mode = GPIO_Mode_IN,
		.GPIO_PuPd = GPIO_PuPd_UP,
	};

	if (gpio->port)
		GPIO_Init(gpio->port, &GPIO_InitStructure);
}

bool gpio_read(const struct gpio *gpio)
{
	return gpio->port ? GPIO_ReadInputDataBit(gpio->port, gpio->pin) : 0;
}

void gamepad_init(const struct gamepad_cfg *gpcfg)
{
	int i;
	for (i = 0; i < 2; i++)
		gpio_init_input(&gpcfg->x[i]);
	for (i = 0; i < 2; i++)
		gpio_init_input(&gpcfg->y[i]);
	for (i = 0; i < 8; i++)
		gpio_init_input(&gpcfg->btns[i]);
}

int8_t gamepad_read_axis(const struct gpio *axis)
{
	bool n = gpio_read(&axis[0]);
	bool p = gpio_read(&axis[1]);
	return (n ? 0 : 0x81) + (p ? 0 : 0x7f);
}

void gamepad_update_single(const struct gamepad_cfg *gpcfg, int idx, int8_t value)
{
	if (value != gpcfg->report[idx]) {
		gpcfg->report[idx] = value;
		gpcfg->report[4] = 1;
	}
}

void gamepad_update(const struct gamepad_cfg *gpcfg)
{
	int8_t tmp, btn_state = 0;
	int i;

	if (gpcfg->report[4])
		return;

	for (i = 0; i < 8; i++) {
		tmp = gpio_read(&gpcfg->btns[i]) ? 0 : 1 << i;
		btn_state |= tmp;

		/* Turn off button when pressed */
		if (gpcfg->btns[i].led && tmp)
			neopixel_set_u32(&neo1, gpcfg->btns[i].led - 1, 0);
	}
	gamepad_update_single(gpcfg, 1, btn_state);
	gamepad_update_single(gpcfg, 2, gamepad_read_axis(gpcfg->x));
	gamepad_update_single(gpcfg, 3, gamepad_read_axis(gpcfg->y));

	/* Show direction on the Compass Rose LEDs */
	if (gpcfg->report[2] < 0)
		if (gpcfg->report[3] < 0)
			STM_EVAL_LEDOn(LED4);
		else if (gpcfg->report[3] > 0)
			STM_EVAL_LEDOn(LED8);
		else
			STM_EVAL_LEDOn(LED6);
	else if (gpcfg->report[2] > 0)
		if (gpcfg->report[3] < 0)
			STM_EVAL_LEDOn(LED5);
		else if (gpcfg->report[3] > 0)
			STM_EVAL_LEDOn(LED9);
		else
			STM_EVAL_LEDOn(LED7);
	else if (gpcfg->report[3] < 0)
		STM_EVAL_LEDOn(LED3);
	else if (gpcfg->report[3] > 0)
		STM_EVAL_LEDOn(LED10);
}

void trackball_update(const struct gamepad_cfg *gpcfg)
{
	static int16_t lastx = 0, lasty = 0;
	int16_t value;
	int8_t btn_state = 0;
	int i;
	static uint8_t color_pos = 0;
	uint32_t color;

	if (gpcfg->report[4])
		return;

	value = TIM_GetCounter(TIM3);
	color_pos += gpcfg->report[2] = value - lastx;
	lastx = value;

	value = TIM_GetCounter(TIM4);
	color_pos += gpcfg->report[3] = value - lasty;
	lasty = value;

	color = color_wheel(color_pos);
	if (curr_mode == EFFECT_STATIC)
		for (i = 0; i < 7; i++)
			neopixel_set_u32(&neo1, gpcfg->btns[0].led - 1 + i, color);

	for (i = 0; i < 3; i++)
		btn_state |= gpio_read(&gpcfg->btns[i]) ? 0 : 1 << i;
	gamepad_update_single(gpcfg, 1, btn_state);

	if (gpcfg->report[2] || gpcfg->report[3]) {
		STM_EVAL_LEDOn(LED3);
		gpcfg->report[4] = 1;
	}
}

void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP,
	};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Connect PC{6,7} pins to TIM3_CH{1,2} */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2);

	/* Connect PD{12,13} pins to TIM4_CH{1,2} */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	/* Setup Encoder mode on TIM3 */
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, ENABLE);
	/* Setup Encoder mode on TIM4 */
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  Configure the USB.
  * @param  None
  * @retval None
  */
void USB_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Pin = GPIO_Pin_12,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
	};

	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();

	/* Fix broken powerup sequence on STM32F3 Discovery by forcing the DP
	 * pin to 0, waiting a bit, and then changing it back to USB. This fix
	 * is courtesy
	 * https://stackoverflow.com/questions/35218303/stm32f3-user-usb-not-det
	 * ected */
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BRR |= GPIO_Pin_12;
	while (DataReady < 0xf0);
	DataReady = 0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USB_Init();

	while (bDeviceState != CONFIGURED);
}

void send_next_report(void)
{
	int i;

	/* Look for a report ready to be sent */
	for (i = 0; i < NUM_JOYSTICKS + 1; i++) {
		if (gamepads[i].report[4]) {
			usb_transfer_pending = 1;
			gamepads[i].report[4] = 0;
			/* Copy report position info in ENDP1 Tx Packet Memory Area */
			USB_SIL_Write(EP1_IN, (uint8_t *) gamepads[i].report, 4);
			/* Enable endpoint for transmission */
			SetEPTxValid(ENDP1);
			return;
		}
	}

	/* There are no ready reports. Say transfers are finished */
	usb_transfer_pending = 0;
}

void EP1_IN_Callback(void)
{
	send_next_report();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line
	 * number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
	 * file, line) */

	/* Infinite loop */
	while (1);
}
#endif

int main(void)
{
	int i = 0;
	int cindex = 0;
	uint8_t brightness;
	uint32_t color;

	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 500);

	/* Initialize LEDs and User Button available on STM32F3-Discovery board */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

	STM_EVAL_LEDOn(LED3);

	/* Set up the pins */
	for (i = 0; i < NUM_JOYSTICKS; i++)
		gamepad_init(&gamepads[i]);
	encoder_init();
	ws2812_init();
	neopixel_init(&neo1, NEO_GRBW, NEOPIXELS_PER_CH * 2, neopixel_buf);
	memset(neopixel_buf, 0, sizeof(neopixel_buf));

	/* Send out LED stream and wait for two timer ticks */
	ws2812_send(neopixel_buf, NEOPIXEL_BUFSIZE);
	while (DataReady < 2);
	DataReady = 0;

	/* Send out LED stream again to get rid of spurious pixel */
	ws2812_send(neopixel_buf, NEOPIXEL_BUFSIZE);
	while (DataReady < 2);
	DataReady = 0;

	effects_prepare(EFFECT_STATIC);

	/* Configure the USB */
	USB_Config();
	STM_EVAL_LEDOn(LED4);

	/* Accelerometer Configuration */
	Acc_Config();
	STM_EVAL_LEDOn(LED5);

	STM_EVAL_LEDOn(LED6);

	/* Infinite loop */
	while (1) {
		/* Wait for two timer ticks and the USB transfers to finish */
		while (usb_transfer_pending || (DataReady < 2));
		DataReady = 0;

		/* Update the animation index and cycle the effect periodically */
		cindex++;
		if (cindex > 0x7ff) {
			cindex = 0;
			effects_prepare(curr_mode + 1);
		}
		effects_animate(cindex);

		/* Turn off all the joystick LEDs */
		for (i = 0; i < 8; i++)
			STM_EVAL_LEDOff(i);

		/* Get updates from each of the logical controls */
		for (i = 0; i < NUM_JOYSTICKS; i++)
			gamepad_update(&gamepads[i]);
		trackball_update(&gamepads[NUM_JOYSTICKS]);

		/* 'Throb' one of the player button LEDs */
		brightness = (cindex & 0x1ff) > 0x100 ? 0x100 - (cindex >> 1) : cindex >> 1;
		color = color_brightness(gamepads[0].color, brightness);;
		neopixel_set_u32(&neo1, 15, color);

		send_next_report();

		/* Get Data Accelerometer */
		Acc_ReadData(AccBuffer);

		for (i = 0; i < 3; i++)
			AccBuffer[i] /= 100.0f;

		ws2812_send(neopixel_buf, NEOPIXEL_BUFSIZE);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
