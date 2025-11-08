/*

  ppi.c - plugin for for laser PPI (Pulses Per Inch) mode

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if PPI_ENABLE

#include <math.h>
#include <string.h>

#include "grbl/hal.h"

static struct {
    uint_fast16_t ppi;
    float ppi_distance;
    float ppi_pos;
    float next_pos;
    uint_fast16_t pulse_length; // uS
    bool on;
} laser = {
    .ppi = 600.0f,
    .ppi_distance = 25.4f / 600.0f,
    .pulse_length = 1500,
    .on = false
};

static spindle_ptrs_t *ppi_spindle;

static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static stepper_wake_up_ptr stepper_wake_up;
static stepper_pulse_start_ptr stepper_pulse_start;
static on_parser_init_ptr on_parser_init;
static on_spindle_selected_ptr on_spindle_selected;
static on_program_completed_ptr on_program_completed;
static spindle_update_pwm_ptr spindle_update_pwm;
static spindle_update_rpm_ptr spindle_update_rpm;

static void stepperWakeUp (void)
{
    laser.ppi_pos = laser.next_pos = 0.0f;

    stepper_wake_up();
}

static void stepperPulseStartPPI (stepper_t *stepper)
{
    static float mm_per_step;

    if(laser.on) {

        if(stepper->new_block)
            mm_per_step = 1.0f / stepper->exec_block->steps_per_mm;

        if(stepper->step_out.bits) {
            laser.ppi_pos += mm_per_step;
            if(laser.ppi_pos >= laser.next_pos) {
                laser.next_pos += laser.ppi_distance;
                ppi_spindle->pulse_on(ppi_spindle, laser.pulse_length);
            }
        }
    }

    stepper_pulse_start(stepper);
}

static void ppiUpdatePWM (spindle_ptrs_t *spindle, uint_fast16_t pwm)
{
    if(!laser.on && pwm > 0)
        laser.ppi_pos = laser.next_pos = 0.0f;

    laser.on = pwm > 0;

    spindle_update_pwm(spindle, pwm);

    if(stepper_wake_up)
        spindle->pulse_on(spindle, laser.pulse_length);
}

static void ppiUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    if(!laser.on && rpm > 0.0f)
        laser.ppi_pos = laser.next_pos = 0.0f;

    laser.on = rpm > 0.0f;

    spindle_update_rpm(spindle, rpm);
}

static bool enable_ppi (bool on)
{
    if(!gc_laser_ppi_enable(on ? laser.ppi : 0, laser.pulse_length)) {

        if(on && stepper_wake_up == NULL) {
            stepper_wake_up = hal.stepper.wake_up;
            hal.stepper.wake_up = stepperWakeUp;
            stepper_pulse_start = hal.stepper.pulse_start;
            hal.stepper.pulse_start = stepperPulseStartPPI;
        }

        if(!on && stepper_wake_up != NULL) {
            hal.stepper.wake_up = stepper_wake_up;
            stepper_wake_up = NULL;
            hal.stepper.pulse_start = stepper_pulse_start;
            stepper_pulse_start = NULL;
        }
    }

    return on;
}

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == LaserPPI_Enable || mcode == LaserPPI_Rate || mcode == LaserPPI_PulseLength
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case LaserPPI_Enable:
            if(!hal.driver_cap.laser_ppi_mode)
                state = Status_GcodeUnsupportedCommand;
            else if(gc_block->words.p) {
                state = Status_OK;
                gc_block->words.p = Off;
            }
            break;

        case LaserPPI_Rate:
            if(!hal.driver_cap.laser_ppi_mode)
                state = Status_GcodeUnsupportedCommand;
            else if(gc_block->words.p) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                gc_block->words.p = Off;
            }
            break;

        case LaserPPI_PulseLength:
            if(!hal.driver_cap.laser_ppi_mode)
                state = Status_GcodeUnsupportedCommand;
            else if(gc_block->words.p) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                gc_block->words.p = Off;
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    static bool ppi_on = false;

    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

        case LaserPPI_Enable:
            ppi_on = gc_block->values.p != 0.0f;
            enable_ppi(ppi_on && laser.ppi > 0 && laser.pulse_length > 0);
            break;

        case LaserPPI_Rate:
            if((laser.ppi = (uint_fast16_t)gc_block->values.p) != 0)
                laser.ppi_distance = 25.4f / (float)laser.ppi;
            enable_ppi(ppi_on && laser.ppi > 0 && laser.pulse_length > 0);
            break;

        case LaserPPI_PulseLength:
            laser.pulse_length = (uint16_t)gc_block->values.p;
            enable_ppi(ppi_on && laser.ppi > 0 && laser.pulse_length > 0);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if((hal.driver_cap.laser_ppi_mode = spindle->cap.laser && spindle->pulse_on != NULL)) {

        ppi_spindle = spindle;

        if(spindle->update_pwm) {
            spindle_update_pwm = spindle->update_pwm;
            spindle->update_pwm = ppiUpdatePWM;
        }

        if(spindle->update_rpm) {
            spindle_update_rpm = spindle->update_rpm;
            spindle->update_rpm = ppiUpdateRPM;
        }
    } else
        ppi_spindle = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void onParserInit (parser_state_t *gc_state)
{
    enable_ppi(false);

    if(on_parser_init)
        on_parser_init(gc_state);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(!check_mode)
        enable_ppi(false);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Laser PPI", "0.09");
}

void ppi_init (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    on_spindle_selected = grbl.on_spindle_selected;
    grbl.on_spindle_selected = onSpindleSelected;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_parser_init = grbl.on_parser_init;
    grbl.on_parser_init = onParserInit;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;
}

#endif
