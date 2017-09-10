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
#include "main.h"

/** @addtogroup STM32F3_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup USB_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;

float  AccBuffer[3] = {0.0f};

__IO uint8_t DataReady = 0;
__IO uint8_t PrevXferComplete = 1;

/* Input report buffers */
/* Report ID, Buttons, X, Y, change_flag */
#if (NUM_JOYSTICKS >= 1)
static int8_t gamepad1_report[5] = {1,0,0,0,0};
#endif
#if (NUM_JOYSTICKS >= 2)
static int8_t gamepad2_report[5] = {2,0,0,0,0};
#endif
#if (NUM_JOYSTICKS >= 3)
static int8_t gamepad3_report[5] = {3,0,0,0,0};
#endif
#if (NUM_JOYSTICKS >= 4)
static int8_t gamepad4_report[5] = {4,0,0,0,0};
#endif
static int8_t trackball_report[5] = {5,0,0,0,0};

/* Gamepad GPIO defintions */
struct gpio {
	GPIO_TypeDef* port;
	uint16_t pin;
	uint8_t led;
};

struct gamepad_cfg {
	struct gpio x[2];
	struct gpio y[2];
	struct gpio btns[8];
	int8_t *report;
};

const struct gamepad_cfg gamepads[NUM_JOYSTICKS+1] = {
#if (NUM_JOYSTICKS >= 1)
	{
		.x = {{ GPIOD, GPIO_Pin_10 },{ GPIOD, GPIO_Pin_8 }},
		.y = {{ GPIOB, GPIO_Pin_14 },{ GPIOB, GPIO_Pin_12 }},
		.btns = {
			{ GPIOC, GPIO_Pin_11, 5 }, /* A */
			{ GPIOD, GPIO_Pin_2, 3 },  /* B */
			{ GPIOD, GPIO_Pin_0, 4 },  /* X */
			{ GPIOD, GPIO_Pin_6, 1 },  /* Y */
			{ GPIOD, GPIO_Pin_4, 2 },  /* L */
			{ GPIOA, GPIO_Pin_15, 6 }, /* R */
			{ GPIOC, GPIO_Pin_7, 7 },  /* Select */
			{ GPIOD, GPIO_Pin_14, 8 }, /* Start */
		},
		.report = gamepad1_report,
	},
#endif
#if (NUM_JOYSTICKS >= 2)
	{
		.x = {{ GPIOB, GPIO_Pin_13 },{ GPIOD, GPIO_Pin_11 }},
		.y = {{ GPIOB, GPIO_Pin_15 },{ GPIOD, GPIO_Pin_9 }},
		.btns = {
			{ GPIOC, GPIO_Pin_12, 15 }, /* A */
			{ GPIOD, GPIO_Pin_3, 13 },  /* B */
			{ GPIOD, GPIO_Pin_1, 14 },  /* X */
			{ GPIOD, GPIO_Pin_7, 12},  /* Y */
			{ GPIOD, GPIO_Pin_5, 11 },  /* L */
			{ GPIOC, GPIO_Pin_10, 16 }, /* R */
			{ GPIOC, GPIO_Pin_6, 9 },  /* Select */
			{ GPIOD, GPIO_Pin_15, 10 }, /* Start */
		},
		.report = gamepad2_report,
	},
#endif
#if (NUM_JOYSTICKS >= 3)
	{
		.report = gamepad3_report,
	},
#endif
#if (NUM_JOYSTICKS >= 2)
	{
		.report = gamepad4_report,
	},
#endif
	{
		.report = trackball_report,
	}
};

uint8_t led_buffer[8*4][3];

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
	for (i=0; i < 2; i++)
		gpio_init_input(&gpcfg->x[i]);
	for (i=0; i < 2; i++)
		gpio_init_input(&gpcfg->y[i]);
	for (i=0; i < 8; i++)
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

	for (i = 0; i < 8; i++) {
		tmp = gpio_read(&gpcfg->btns[i]) ? 0 : 1 << i;
		btn_state |= tmp;

		/* This is an ugly hack to get red for P1, and blue for P2. To be fixed */
		if (gpcfg->btns[i].led > 8)
			led_buffer[gpcfg->btns[i].led-1][2] = tmp ? 0x1f : 0;
		else if (gpcfg->btns[i].led > 0)
			led_buffer[gpcfg->btns[i].led-1][0] = tmp ? 0x1f : 0;
	}
	gamepad_update_single(gpcfg, 1, btn_state);
	gamepad_update_single(gpcfg, 2, gamepad_read_axis(gpcfg->x));
	gamepad_update_single(gpcfg, 3, gamepad_read_axis(gpcfg->y));
}

void trackball_update(void)
{
	static int16_t lastx = 0, lasty = 0;
	int16_t value;

	value = TIM_GetCounter(TIM3);
	if (lastx != value) {
		STM_EVAL_LEDOn(LED3);
		trackball_report[2] = value - lastx;
		trackball_report[4] = 1;
		lastx = value;
	}
	value = TIM_GetCounter(TIM4);
	if (lasty != value) {
		STM_EVAL_LEDOn(LED4);
		trackball_report[3] = value - lasty;
		trackball_report[4] = 1;
		lasty = value;
	}
}

void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP,
	};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Connect PB{4,5} pins to TIM3_CH{1,2} */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_2);

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

int main(void)
{
  int i = 0;
  int cindex = 0;
  int current_gamepad = 0;

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
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

  ws2812_send(&led_buffer[10], 16);
  while(DataReady !=0x02)
  {}
  DataReady = 0x00;

  /* Configure the USB */
  USB_Config();
  STM_EVAL_LEDOn(LED4);

  /* Accelerometer Configuration */
  Acc_Config();
  STM_EVAL_LEDOn(LED5);

  STM_EVAL_LEDOn(LED6);

  /* Infinite loop */
  while (1)
  {

    /* Wait for data ready */
    while(DataReady !=0x02)
    {}
    DataReady = 0x00;

    /* Kick off an LED transfer */
    led_buffer[7][2] = cindex++ >> 1;

    for (i = 0; i < 8; i++)
      STM_EVAL_LEDOff(i);

    for (i = 0; i < NUM_JOYSTICKS; i++) {
      gamepad_update(&gamepads[i]);

      if (gamepads[i].report[2] < 0)
        if (gamepads[i].report[3] < 0)
          STM_EVAL_LEDOn(LED8);
        else if (gamepads[i].report[3] > 0)
          STM_EVAL_LEDOn(LED4);
        else
          STM_EVAL_LEDOn(LED6);
      else if (gamepads[i].report[2] > 0)
        if (gamepads[i].report[3] < 0)
          STM_EVAL_LEDOn(LED9);
        else if (gamepads[i].report[3] > 0)
          STM_EVAL_LEDOn(LED5);
        else
          STM_EVAL_LEDOn(LED7);
      else
        if (gamepads[i].report[3] < 0)
          STM_EVAL_LEDOn(LED10);
        else if (gamepads[i].report[3] > 0)
          STM_EVAL_LEDOn(LED3);
    }

    if (current_gamepad == 4)
      trackball_update();

    if (gamepads[current_gamepad].report[4]) {
      gamepads[current_gamepad].report[4] = 0;
      /* Reset the control token to inform upper layer that a transfer is ongoing */
      PrevXferComplete = 0;
      /* Copy report position info in ENDP1 Tx Packet Memory Area*/
      USB_SIL_Write(EP1_IN, (uint8_t*)gamepads[current_gamepad].report, 4);
      /* Enable endpoint for transmission */
      SetEPTxValid(ENDP1);
    }

    current_gamepad = (current_gamepad + 1) % (NUM_JOYSTICKS+1);

    /* Get Data Accelerometer */
    Acc_ReadData(AccBuffer);

    for(i=0;i<3;i++)
      AccBuffer[i] /= 100.0f;

    ws2812_send(led_buffer, 16);
  }
}

/**
  * @brief  Configure the USB.
  * @param  None
  * @retval None
  */
void USB_Config(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();

  USB_Init();

  while (bDeviceState != CONFIGURED)
  {}
}

/**
  * @brief  Configure the Mems to Accelerometer MEMS.
  * @param  None
  * @retval None
  */
void Acc_Config(void)
{
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void Acc_ReadData(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);


  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
