/*
 * safety_handler.h
 *
 *  Created on: Apr 1, 2026
 *      Author: ishanchitale
 */

#ifndef INC_SAFETY_HANDLER_H_
#define INC_SAFETY_HANDLER_H_

#define NUM_FAULT_TYPES 6

#include "gpio.h"
#include "bms_state.h"
#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "freertos_handles.h"

#define BMS_SHUTDOWN_LOST_TX_ID 0x6B6

typedef struct {
    uint8_t overvoltage  : 1;
    uint8_t undervoltage : 1;
    uint8_t overtemp     : 1;
    uint8_t undertemp    : 1;
    uint8_t overcurrent  : 1;
    uint8_t isospi_disconnect : 1;
    uint8_t air_weld     : 1;
    uint8_t open_wire    : 1;
} BMS_FaultBits;

#define FAULT_OVERVOLTAGE        (1U << 0)
#define FAULT_UNDERVOLTAGE       (1U << 1)
#define FAULT_OVERTEMP    	     (1U << 2)
#define FAULT_UNDERTEMP          (1U << 3)
#define FAULT_OVERCURRENT        (1U << 4)
#define FAULT_ISOSPI_DISCONNECT  (1U << 5)
#define FAULT_AIR_WELD           (1U << 6)
#define FAULT_OPEN_WIRE          (1U << 7)

typedef union {
    uint8_t reg;
    BMS_FaultBits bits;
} BMS_FaultRegister;
extern volatile BMS_FaultRegister fault_register;

void BMS_SetFault(uint8_t fault);
void BMS_ClearFault(uint8_t fault);


#define BMS_FAULT_SET_DEBOUNCE_MS 100U
#define BMS_FAULT_CLEAR_DEBOUNCE_MS 150U

typedef struct {
    bool latched;
    bool pending;
    bool requested_active;
    uint32_t transition_time_ms;
} BMS_FaultDebounceState;

void BMS_UpdateFaultDebounced(uint8_t fault, bool active);
void BMS_ApplyFaultDebounce(uint8_t fault);
uint8_t BMS_GetFaultRegister();
void BMS_CheckFaultRegister();

void configureShutdownLostTxMsg();
void service_shutdown_power_signal();
void apply_shutdown_power_state(bool shutdown_asserted);

#endif /* INC_SAFETY_HANDLER_H_ */
