/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "mrc_statemachine.h"

// User should implement this function to actually set the hardware current
__weak void MRC_SetCurrent(float current_A) {
    // TODO: Implement hardware current output
    (void)current_A;
}

void MRC_StateMachine_SetDemagDuration(MRC_StateMachine_t *sm, float duration_ms) {
    if (duration_ms > 0) {
        sm->demag_duration_ms = duration_ms;
    }
}

void MRC_StateMachine_Init(MRC_StateMachine_t *sm) {
    sm->current_mode = FREE;
    sm->target_mode = FREE;
    sm->current_value = 0.0f;
    sm->demag_timer_ms = 0.0f;
    sm->demag_value = 0.0f;
    sm->demag_in_progress = false;
    MRC_StateMachine_SetDemagDuration(sm,100.0f);
}

// Call this to request a mode change
void MRC_StateMachine_SetMode(MRC_StateMachine_t *sm, MRC_Mode mode, float arg1) {
    if (sm->current_mode == mode && !sm->demag_in_progress) return;

    // If switching to FREE from FIX_LIMIT or ADAPTATION, start demagnetization
    if ((mode == FREE) && !sm->demag_in_progress && 
        (sm->current_mode == FIX_LIMIT || sm->current_mode == ADAPTATION)) {
        sm->target_mode = FREE;
        sm->demag_value = -0.5f * arg1;
        sm->demag_timer_ms = sm->demag_duration_ms; // 使用用户自定义退磁时间
        sm->demag_in_progress = true;
        MRC_SetCurrent(sm->demag_value);
        // current_mode 保持原状态，等退磁完成后再切换
    } else {
        sm->current_mode = mode;
        sm->target_mode = mode;
        if (mode == FREE) {
            sm->current_value = 0.0f;
            MRC_SetCurrent(0.0f);
        }
    }
}

// Call this when collision is detected in ADAPTATION mode
void MRC_StateMachine_OnCollisionDetected(MRC_StateMachine_t *sm, float arg1) {
    if (sm->current_mode == ADAPTATION) {
        MRC_StateMachine_SetMode(sm, FREE, arg1);
    }
}
void MRC_StateMachine_OnCollisionEnded(MRC_StateMachine_t *sm, float arg1) {
    if (sm->current_mode == FREE) {
        MRC_StateMachine_SetMode(sm, ADAPTATION, arg1);
    }
}

// Call this every 1ms (e.g. in a timer interrupt)
void MRC_StateMachine_Task1ms(MRC_StateMachine_t *sm) {
    if (sm->demag_in_progress) {
        if (sm->demag_timer_ms > 0) {
            sm->demag_timer_ms -= 1.0f;
            if (sm->demag_timer_ms <= 0) {
                // Demagnetization done, set current to 0 and switch to target_mode
                MRC_SetCurrent(0.0f);
                sm->current_value = 0.0f;
                sm->current_mode = sm->target_mode;
                sm->demag_in_progress = false;
            }
        }
    }
}

// Only allow current set in FIX_LIMIT, ADAPTATION, DEBUG
bool MRC_StateMachine_CanSetCurrent(MRC_StateMachine_t *sm) {
    return (sm->current_mode == FIX_LIMIT || sm->current_mode == ADAPTATION || sm->current_mode == DEBUG);
}

// Only allow voltage set in DEBUG (or add other logic as needed)
bool MRC_StateMachine_CanSetVoltage(MRC_StateMachine_t *sm) {
    return (sm->current_mode == DEBUG);
}
