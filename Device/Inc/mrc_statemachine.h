/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef MRC_STATEMACHINE_H
#define MRC_STATEMACHINE_H

#include "mrc_protocol.h"
#include <stdint.h>
#include <stdbool.h>

// State machine context for MRC
#pragma pack(1)
typedef struct {
    MRC_Mode current_mode;      // Current working mode
    MRC_Mode target_mode;       // Target mode for transition
    float current_value;        // Current actual value (A)
    float demag_timer_ms;       // Demagnetization timer (ms)
    float demag_value;          // Demagnetization current value (A)
    bool demag_in_progress;     // Demagnetization in progress flag
    float demag_duration_ms;    // User-defined demagnetization duration (ms)
} MRC_StateMachine_t;
#pragma pack()

// Initialize the state machine
void MRC_StateMachine_Init(MRC_StateMachine_t *sm);
// Request a mode change
void MRC_StateMachine_SetMode(MRC_StateMachine_t *sm, MRC_Mode mode, float arg1);
// Call when collision is detected in ADAPTATION mode
void MRC_StateMachine_OnCollisionDetected(MRC_StateMachine_t *sm, float arg1);
// Call when collision is ended in ADAPTATION mode
void MRC_StateMachine_OnCollisionEnded(MRC_StateMachine_t *sm, float arg1);
// Call every 1ms (e.g. in a timer interrupt)
void MRC_StateMachine_Task1ms(MRC_StateMachine_t *sm);
// Query if current can be set in current mode
bool MRC_StateMachine_CanSetCurrent(MRC_StateMachine_t *sm);
// Query if voltage can be set in current mode
bool MRC_StateMachine_CanSetVoltage(MRC_StateMachine_t *sm);
// Set demagnetization duration (ms)
void MRC_StateMachine_SetDemagDuration(MRC_StateMachine_t *sm, float duration_ms);

#endif // MRC_STATEMACHINE_H 
