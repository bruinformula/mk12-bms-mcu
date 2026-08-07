/*
 * current_calculations.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ishanchitale
 */

#include "current_calculations.h"

volatile CURRENT_CONTEXT current_context;
static bool using_high_range = false;
static bool current_filter_initialized = false;
static float filtered_current_low = 0.0f;
static float filtered_current_high = 0.0f;
static uint8_t overcurrent_set_count = 0;
static uint8_t overcurrent_clear_count = 0;
static uint8_t air_weld_set_count = 0;

#define CURRENT_FILTER_ALPHA        0.10f
#define HIGH_RANGE_ENTER_CURRENT    30.0f
#define HIGH_RANGE_EXIT_CURRENT     25.0f
#define OVERCURRENT_SET_SAMPLES     5U
#define OVERCURRENT_CLEAR_SAMPLES   10U

void checkAIRWeld(float selected_current) {
	GPIO_PinState pos_air = HAL_GPIO_ReadPin(POS_AIR_GND_GPIO_Port, POS_AIR_GND_Pin);
	GPIO_PinState neg_air = HAL_GPIO_ReadPin(NEG_AIR_GND_GPIO_Port, NEG_AIR_GND_Pin);
	GPIO_PinState precharge = HAL_GPIO_ReadPin(PRECHARGE_GPIO_Port, PRECHARGE_Pin);

	bool are_airs_open = (pos_air == GPIO_PIN_RESET) &&
						(neg_air == GPIO_PIN_RESET) &&
						(precharge == GPIO_PIN_RESET);

	if (are_airs_open) {
		if (fabsf(selected_current) > AIR_WELD_CURRENT_THRESHOLD) {
			if (air_weld_set_count < AIR_WELD_SET_SAMPLES) {
				air_weld_set_count++;
			}
		} else {
			// Current not above threshold so reset sample count back to zero
			air_weld_set_count = 0;
		}
	} else {
		// AIRs closed already
		air_weld_set_count = 0;
	}

#if (BMS_FAULT_AIR_WELD == BMS_FAULT_ENABLED)
	if (air_weld_set_count >= AIR_WELD_SET_SAMPLES ) {
		BMS_SetFault(FAULT_AIR_WELD);
	}
#endif
}

void calculateCurrent() {
	float selected_current;

	taskENTER_CRITICAL();
	current_context.current_sensor_low = ((float) current_context.current_sensor_low_adc)
			* 0.01877f - 37.412f;
	current_context.current_sensor_high = ((float) current_context.current_sensor_high_adc)
			* 0.2159f - 429.326f;
	taskEXIT_CRITICAL();

	if (!current_filter_initialized) {
		filtered_current_low = current_context.current_sensor_low;
		filtered_current_high = current_context.current_sensor_high;
		current_filter_initialized = true;
	} else {
		filtered_current_low += CURRENT_FILTER_ALPHA * (current_context.current_sensor_low - filtered_current_low);
		filtered_current_high += CURRENT_FILTER_ALPHA * (current_context.current_sensor_high - filtered_current_high);
	}

	if (!using_high_range && fabsf(filtered_current_low) > HIGH_RANGE_ENTER_CURRENT) {
		using_high_range = true;
	} else if (using_high_range && fabsf(filtered_current_low) < HIGH_RANGE_EXIT_CURRENT) {
		using_high_range = false;
	}

	selected_current = using_high_range ? filtered_current_high : filtered_current_low;

	current_context.current_sensor_val = selected_current;

	// FAULT HANDLING
	checkAIRWeld(selected_current);

	uint8_t faults_set = 0;
	uint8_t faults_clear = 0;

	if (fabsf(selected_current) > OVERCURRENT_THRESHOLD) {
		if (overcurrent_set_count < OVERCURRENT_SET_SAMPLES) {
			overcurrent_set_count++;
		}
		overcurrent_clear_count = 0;
	} else {
		if (overcurrent_clear_count < OVERCURRENT_CLEAR_SAMPLES) {
			overcurrent_clear_count++;
		}
		overcurrent_set_count = 0;
	}

#if BMS_FAULT_OVERCURRENT == BMS_FAULT_ENABLED
	if (overcurrent_set_count >= OVERCURRENT_SET_SAMPLES) {
		faults_set |= FAULT_OVERCURRENT;
	} else if (overcurrent_clear_count >= OVERCURRENT_CLEAR_SAMPLES) {
		faults_clear |= FAULT_OVERCURRENT;
	}
#endif

	if (faults_set) {
		BMS_SetFault(faults_set);
	}
	if (faults_clear) {
		BMS_ClearFault(faults_clear);
	}
}
