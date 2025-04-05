/*

  co2.c - plugin for for adding initial power on laser on

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#if LASER_OVD_ENABLE

#include "grbl/hal.h"

static spindle_pwm_t *laser = NULL;

static driver_reset_ptr driver_reset;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static on_program_completed_ptr on_program_completed;

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == Laser_Overdrive && !!laser && laser->flags.rpm_controlled
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_Unhandled;

    if(gc_block->user_mcode == Laser_Overdrive) {
        if(gc_block->words.p) {
            if(gc_block->values.p >= 0.0f) {
                state = Status_OK;
                gc_block->words.p = Off;
            } else
                state = Status_GcodeValueOutOfRange;
        } else
            state = Status_GcodeValueWordMissing;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == Laser_Overdrive)
        laser->set_laser_overdrive(laser, gc_block->values.p);
    else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    laser = spindle->cap.laser && !!spindle->context.pwm && !!spindle->context.pwm->set_laser_overdrive ? spindle->context.pwm : NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(laser)
        laser->set_laser_overdrive(laser, 0.0f);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void driverReset (void)
{
    driver_reset();

    if(laser)
        laser->set_laser_overdrive(laser, 0.0f);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("CO2 laser overdrive", "0.01");
}

void laser_ovd_init (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    on_spindle_selected = grbl.on_spindle_selected;
    grbl.on_spindle_selected = onSpindleSelected;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

    driver_reset = hal.driver_reset;
    hal.driver_reset = driverReset;
}

#endif // LASER_OVD_ENABLE
