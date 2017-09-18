/**
  ******************************************************************************
  * @file    USB_Example/usb_istr.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   ISTR events interrupt service routines
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

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "usb_istr.h"

__IO uint16_t wIstr;		/* ISTR register last read value */
__IO uint8_t bIntPackSOF = 0;	/* SOFs received between 2 consecutive packets */

#ifndef CTR_CALLBACK
static void CTR_Callback(void) { }
#endif

#ifndef ERR_CALLBACK
static void ERR_Callback(void) { }
#endif

#ifndef WKUP_CALLBACK
static void WKUP_Callback(void) { }
#endif

#ifndef SUSP_CALLBACK
static void SUSP_Callback(void) { }
#endif

#ifndef RESET_CALLBACK
static void RESET_Callback(void) { }
#endif

#ifndef SOF_CALLBACK
static void SOF_Callback(void) { }
#endif

#ifndef ESOF_CALLBACK
static void ESOF_Callback(void) { }
#endif

/* function pointers to non-control endpoints service routines */
void (*pEpInt_IN[7])(void) = {
	EP1_IN_Callback,
	EP2_IN_Callback,
	EP3_IN_Callback,
	EP4_IN_Callback,
	EP5_IN_Callback,
	EP6_IN_Callback,
	EP7_IN_Callback,
};

void (*pEpInt_OUT[7])(void) = {
	EP1_OUT_Callback,
	EP2_OUT_Callback,
	EP3_OUT_Callback,
	EP4_OUT_Callback,
	EP5_OUT_Callback,
	EP6_OUT_Callback,
	EP7_OUT_Callback,
};

/**
  * @brief  ISTR events interrupt service routine.
  * @param  None
  * @retval None
  */
void USB_Istr(void)
{
	wIstr = _GetISTR();

#if (IMR_MSK & ISTR_CTR)
	if (wIstr & ISTR_CTR & wInterrupt_Mask) {
		/* servicing of the endpoint correct transfer interrupt */
		/* clear of the CTR flag into the sub */
		CTR_LP();
		CTR_Callback();
	}
#endif

#if (IMR_MSK & ISTR_RESET)
	if (wIstr & ISTR_RESET & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_RESET);
		Device_Property.Reset();
		RESET_Callback();
	}
#endif

#if (IMR_MSK & ISTR_DOVR)
	if (wIstr & ISTR_DOVR & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_DOVR);
		DOVR_Callback();
	}
#endif

#if (IMR_MSK & ISTR_ERR)
	if (wIstr & ISTR_ERR & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_ERR);
		ERR_Callback();
	}
#endif

#if (IMR_MSK & ISTR_WKUP)
	if (wIstr & ISTR_WKUP & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_WKUP);
		Resume(RESUME_EXTERNAL);
		WKUP_Callback();
	}
#endif

#if (IMR_MSK & ISTR_SUSP)
	if (wIstr & ISTR_SUSP & wInterrupt_Mask) {
		/* check if SUSPEND is possible */
		if (fSuspendEnabled)
			Suspend();
		else
			Resume(RESUME_LATER); /* if not possible then resume after xx ms */

		/* clear of the ISTR bit must be done after setting of
		 * CNTR_FSUSP */
		_SetISTR((uint16_t) CLR_SUSP);
		SUSP_Callback();
	}
#endif

#if (IMR_MSK & ISTR_SOF)
	if (wIstr & ISTR_SOF & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_SOF);
		bIntPackSOF++;
		SOF_Callback();
	}
#endif

#if (IMR_MSK & ISTR_ESOF)
	if (wIstr & ISTR_ESOF & wInterrupt_Mask) {
		_SetISTR((uint16_t) CLR_ESOF);
		/* resume handling timing is made with ESOFs */
		Resume(RESUME_ESOF);	/* request without change of the
					 * machine state */
		ESOF_Callback();
	}
#endif
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
