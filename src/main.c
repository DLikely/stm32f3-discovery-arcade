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

uint8_t neopixel_buf[NEOPIXEL_BUFSIZE * NEOPIXEL_CH_COUNT];
static uint8_t* neopixel_addr(int num)
{
	if ((num < 1) || (num > NEOPIXELS_PER_CH * NEOPIXEL_CH_COUNT))
		return NULL;
	return &neopixel_buf[(num - 1) * NEOPIXEL_SIZE];
}

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
			{GPIOF, GPIO_Pin_6,  TRACKBALL_LEDS},	/* Y */
			{GPIOC, GPIO_Pin_8,  0},		/* B */
			{GPIOC, GPIO_Pin_9,  0},		/* A */
		},
		.report = trackball_report,
	}
};

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
	uint8_t *ledp;
	int i;

	if (gpcfg->report[4])
		return;

	for (i = 0; i < 8; i++) {
		tmp = gpio_read(&gpcfg->btns[i]) ? 0 : 1 << i;
		btn_state |= tmp;

		/* Set the button illumination */
		ledp = neopixel_addr(gpcfg->btns[i].led);
		if (ledp)
			ws2812_set_u32(ledp, tmp ? 0 : gpcfg->color);
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

uint32_t Wheel(uint8_t WheelPos)
{
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85) {
		return COLOR(255 - WheelPos * 3, 0, WheelPos * 3, 0);
	}
	if (WheelPos < 170) {
		WheelPos -= 85;
		return COLOR(0, WheelPos * 3, 255 - WheelPos * 3, 0);
	}
	WheelPos -= 170;
	return COLOR(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
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

	color = Wheel(color_pos);
	for (i = 0; i < 7; i++)
		ws2812_set_u32(neopixel_addr(gpcfg->btns[0].led + i), color);

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

	memset(neopixel_buf, 0, sizeof(neopixel_buf));
	ws2812_send(neopixel_buf, NEOPIXEL_BUFSIZE);

	/* Wait for two timer ticks */
	while (DataReady < 2);
	DataReady = 0;

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

		/* Turn off all the joystick LEDs */
		for (i = 0; i < 8; i++)
			STM_EVAL_LEDOff(i);

		/* Get updates from each of the logical controls */
		for (i = 0; i < NUM_JOYSTICKS; i++)
			gamepad_update(&gamepads[i]);
		trackball_update(&gamepads[NUM_JOYSTICKS]);

		send_next_report();

		/* Get Data Accelerometer */
		Acc_ReadData(AccBuffer);

		for (i = 0; i < 3; i++)
			AccBuffer[i] /= 100.0f;

		/* 'Throb' one of the buttons */
		cindex++;
		ws2812_set_rgbw(neopixel_addr(16), 0, (cindex & 0x1ff) > 0x100 ? 0x100 - (cindex >> 1) : cindex >> 1, 0, 0);
		ws2812_send(neopixel_buf, NEOPIXEL_BUFSIZE);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
