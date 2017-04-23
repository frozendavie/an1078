/*
 * @brief State Configurable Timer (SCT) PWM example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/** @defgroup PERIPH_SCT_PWM_5410X State Configurable Timer (SCT) PWM example
 * @ingroup EXAMPLES_PERIPH_5410X
 * @include "periph_sct_pwm\readme.txt"
 */

/**
 * @}
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define SCT_PWM            LPC_SCT
#if defined(BOARD_REV1_CSP)
#define SCT_PWM_PIN_OUT    4		/* COUT4 Generate square wave */
#define SCT_PWM_PIN_LED    5		/* COUT5 [index 2] Controls LED */

#elif defined(BOARD_REV1_LQFP)
#define SCT_PWM_PIN_OUT    2		/* COUT4 Generate square wave */
#define SCT_PWM_PIN_LED    3		/* COUT5 [index 2] Controls LED */

#else
#error "Board not supported - setup SCT outputs for example"
#endif

#define SCT_PWM_OUT        1		/* Index of OUT PWM */
#define SCT_PWM_LED        2		/* Index of LED PWM */
#define SCT_PWM_RATE   10000		/* PWM frequency 10 KHz */

/* Systick timer tick rate, to change duty cycle */
#define TICKRATE_HZ     1000		/* 1 ms Tick rate */

#define LED_STEP_CNT      20		/* Change LED duty cycle every 20ms */
#define OUT_STEP_CNT    1000		/* Change duty cycle every 1 second */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup board specific pin muxing */
static void app_setup_pin(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_54102)
#if defined(BOARD_REV1_CSP)
	/* SCT_OUT5 on PIO1_2 mapped to FUNC3: LED_GREEN */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, IOCON_FUNC3 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
	/* SCT_OUT5 on PIO1_1 mapped to FUNC2: Oscilloscope input */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 1, IOCON_FUNC3 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

#else
#if defined(BOARD_REV1_LQFP)
	/* SCT_OUT2 on PIO0_29 mapped to FUNC2: LED_RED */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
	/* SCT_OUT3 on PIO0_30 mapped to FUNC2: LED_GREEN */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 30, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

#else
#warning "No SCT pin muxing defined"
#endif
#endif

#else
	/* Configure your own SCT pin muxing here if needed */
#warning "No SCT pin muxing defined"
#endif

}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	/* This is only used to wakeup up the device from sleep */
}

/* Example entry point */
int main(void)
{
	int loop = 1;	/* Prevents the unreachable statement warning */
	uint32_t cnt1 = 0;
	int led_out_0, led_out_1, led_dp = 0, led_step = 1, currLed = 0;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* Setup Board specific output pin */
	app_setup_pin();

	/* Use SCT0_OUT1 pin */
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_LED, SCT_PWM_PIN_LED);

	/* Start with 0% duty cycle */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED, 0);
	Chip_SCTPWM_Start(SCT_PWM);

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	while (loop) {
		cnt1++;
		if (cnt1 >= LED_STEP_CNT) {
			led_dp += led_step;
			if (led_dp < 0) {
				led_dp = 0;
				led_step = 1;
			}
			if (led_dp > 100) {
				led_dp = 100;
				led_step = -1;
				currLed = 1 - currLed;
			}

			/* Increment or Decrement Dutycycle by 0.5% every 10ms */
			if (currLed) {
				led_out_0 = led_dp;
				led_out_1 = 100;
			}
			else {
				led_out_0 = 100;
				led_out_1 = led_dp;
			}
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED,
									 Chip_SCTPWM_PercentageToTicks(SCT_PWM, led_out_0));
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT,
									 Chip_SCTPWM_PercentageToTicks(SCT_PWM, led_out_1));
			cnt1 = 0;
		}
		__WFI();
	}
	return 0;
}
