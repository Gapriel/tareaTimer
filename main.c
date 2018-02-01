/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    pitControla.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"

//drivers includes
#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

//type defined by the user to establish without magic numbers the flow direction using boolean logic
typedef enum {
	FORWARD, REVERSE
} flow_direction_t;

//type defined by the user to establish the color state
typedef enum {
	RED, GREEN, BLUE
} color_state_t;

typedef enum {
	FALSE, TRUE
} boolean_type_t;

static uint8_t gFlow = FORWARD;	//0 for forward, 1 for reverse
static uint8_t gState = RED;	//current led color
static uint8_t gColorStopped = FALSE;//variable to tell if the color is stopped
static PIT_Type gPit_0;			//PIT 0 base declaration

void PORTA_IRQHandler() {		//SW3 interrupt service routine
	PORT_ClearPinsInterruptFlags(PORTA, 1 << 4);//SW3 pin interrupt flag clearing
	gFlow = (FORWARD == gFlow) ? REVERSE : FORWARD;		//flow variable toggle
}

void PORTC_IRQHandler() {		//SW2 interrupt service routine
	PORT_ClearPinsInterruptFlags(PORTC, 1 << 6);//SW2 pin interrupt flag clearing
	gColorStopped = (FALSE == gColorStopped) ? TRUE : FALSE;//color stopped variable toggle
}

void PIT0_IRQHandler() {
	PIT_ClearStatusFlags(&gPit_0, kPIT_Chnl_0, kPIT_TimerFlag);	//pit0 interrupt flag cleared
	if (FORWARD == gFlow) {
		if (BLUE > gState) {	//if the state is smaller than BLUE (2)
			gState++;	//state increase
		} else {
			gState = RED;	//state reset to RED (0)
		}
	} else {
		if (RED < gState) {		//if the state is bigger than RED (0)
			gState--;	//state increase
		} else {
			gState = BLUE;	//state reset to BLUE (2)
		}
	}
}

int main(void) {

	//clock initializations
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_Pit0);

	//led ports configuration
	port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
	PORT_SetPinConfig(PORTB, 21, &config_led);	//Blue led configuration
	PORT_SetPinConfig(PORTB, 22, &config_led);	//Red led configuration
	PORT_SetPinConfig(PORTE, 26, &config_led);	//Green led configuration

	//switch ports configuration
	port_pin_config_t config_switch = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
	PORT_SetPinConfig(PORTA, 4, &config_switch);
	PORT_SetPinConfig(PORTC, 6, &config_switch);
	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptEitherEdge);

	//leds gpio configuration
	gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };
	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOB, 22, &led_config_gpio);
	GPIO_PinInit(GPIOE, 26, &led_config_gpio);

	//switch gpio configuration
	gpio_pin_config_t switch_config_gpio = { kGPIO_DigitalInput, 1 };
	GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 6, &switch_config_gpio);

	//PIT timer configuration
	pit_config_t config_pit;
	PIT_GetDefaultConfig(&config_pit);
	PIT_Init(&gPit_0, &config_pit);
	PIT_SetTimerPeriod(&gPit_0, kPIT_Chnl_0, 21000000);	//pit timer set to interrupt every second

	//interrupts configuration
	NVIC_EnableIRQ(PORTA_IRQn);		//SW3 interrupt enabled
	NVIC_EnableIRQ(PORTC_IRQn);		//SW2 interrupt enabled
	NVIC_EnableIRQ(PIT0_IRQn);		//PIT 0 interrupt enabled
	PIT_EnableInterrupts(&gPit_0, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	//pit timer enabling
	PIT_StartTimer(&gPit_0, kPIT_Chnl_0);

	//program super loop
	for (;;) {
		if (FALSE == gColorStopped) {
			switch (gState) {
			case RED:
				GPIO_WritePinOutput(GPIOB, 21, 1);
				GPIO_WritePinOutput(GPIOB, 22, 0);
				GPIO_WritePinOutput(GPIOE, 26, 1);
				break;
			case GREEN:
				GPIO_WritePinOutput(GPIOB, 21, 1);
				GPIO_WritePinOutput(GPIOB, 22, 1);
				GPIO_WritePinOutput(GPIOE, 26, 0);
				break;
			case BLUE:
				GPIO_WritePinOutput(GPIOB, 21, 0);
				GPIO_WritePinOutput(GPIOB, 22, 1);
				GPIO_WritePinOutput(GPIOE, 26, 1);
				break;
			}
		}
	}
	return 0;
}
