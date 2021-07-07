/*

  coolant.c - plugin for for handling laser coolant

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if LASER_COOLANT_ENABLE

#include <string.h>
#include <math.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#endif

typedef union {
    uint8_t value;
    struct {
        uint8_t enable : 1;
    };
} coolant_options_t;

typedef struct {
    coolant_options_t options;
    float min_temp;
    float max_temp;
    float on_delay;
    float off_delay;
} coolant_settings_t;

static uint8_t coolant_ok_port, coolant_temp_port;
static bool coolant_on = false, monitor_on = false, can_monitor = false;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static on_program_completed_ptr on_program_completed;
static coolant_ptrs_t on_coolant_changed;
static nvs_address_t nvs_address;
static coolant_settings_t coolant_settings;

static void coolant_settings_restore (void);
static void coolant_settings_load (void);
static void coolant_settings_save (void);

static const setting_detail_t plugin_settings[] = {
    { Setting_CoolantOnDelay, Group_Coolant, "Laser coolant on delay", "seconds", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.on_delay, NULL, NULL },
    { Setting_CoolantOffDelay, Group_Coolant, "Laser coolant off delay", "minutes", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.off_delay, NULL, NULL },
//    { Setting_CoolantMinTemp, Group_Coolant, "Laser coolant min temp", "deg", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.min_temp, NULL, NULL },
    { Setting_CoolantMaxTemp, Group_Coolant, "Laser coolant max temp", "deg", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.max_temp, NULL, NULL }
};

static setting_details_t details_all = {
    .settings = plugin_settings,
    .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
    .save = coolant_settings_save,
    .load = coolant_settings_load,
    .restore = coolant_settings_restore,
};

static setting_details_t details = {
    .settings = plugin_settings,
    .n_settings = 2,
    .save = coolant_settings_save,
    .load = coolant_settings_load,
    .restore = coolant_settings_restore,
};

static void coolant_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&coolant_settings, sizeof(coolant_settings_t), true);
}

static setting_details_t *on_get_settings (void)
{
    return can_monitor ? &details_all : &details;
}

static void coolant_settings_restore (void)
{
    coolant_settings.min_temp = 0.0f;
    coolant_settings.max_temp = 0.0f;
    coolant_settings.on_delay = 0.0f;
    coolant_settings.off_delay = 0.0f;

    coolant_settings_save();
}

static void coolant_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&coolant_settings, nvs_address, sizeof(coolant_settings_t), true) != NVS_TransferResult_OK)
        coolant_settings_restore();
}

// Start/stop tube coolant, wait for ok signal on start if delay is configured.
static void coolantSetState (coolant_state_t mode)
{
    coolant_state_t prev = hal.coolant.get_state();

    on_coolant_changed.set_state(mode);

    if(mode.flood && !prev.flood &&
        coolant_settings.on_delay > 0.0f &&
         hal.port.wait_on_input(true, coolant_ok_port, WaitMode_High, coolant_settings.on_delay) != 1) {
        mode.flood = Off;
        coolant_on = false;
        on_coolant_changed.set_state(mode);
        system_set_exec_alarm(Alarm_AbortCycle);
    }

    monitor_on = mode.flood && (coolant_settings.min_temp + coolant_settings.max_temp) > 0.0f;
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    // Keep coolant and exhaust (flood) on? Setting? Delayed task?
    if(coolant_on && !check_mode) {
        coolant_on = false;
//        hal.port.digital_out(LASER_COOLANT_ON_PORT, false);
        sys.report.coolant = On; // Set to report change immediately
    }

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static float coolant_temp_prev = 0.0f;

    char buf[20] = "";

    *buf = '\0';

    if(can_monitor) {

        float coolant_temp = (float)hal.port.wait_on_input(false, coolant_temp_port, WaitMode_Immediate, 0.0f) / 10.0f;

        if(coolant_temp_prev != coolant_temp || report.all) {
            strcat(buf, "|TCT:");
            strcat(buf, ftoa(coolant_temp, 1));
            coolant_temp_prev = coolant_temp;
        }

        if(monitor_on && coolant_temp > coolant_settings.max_temp)
            system_set_exec_alarm(Alarm_AbortCycle);
    }

    if(*buf != '\0')
        stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Laser coolant v0.02]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Laser coolant plugin failed to initialize!", Message_Warning);
}

void laser_coolant_init (void)
{
    if(hal.port.num_digital_in && hal.port.wait_on_input && (nvs_address = nvs_alloc(sizeof(coolant_settings_t)))) {

        coolant_ok_port = (-- hal.port.num_digital_in);
        if((can_monitor = !!hal.port.num_analog_in))
            coolant_temp_port = (-- hal.port.num_analog_in);

        if(hal.port.set_pin_description) {
            hal.port.set_pin_description(true, false, coolant_ok_port, "Coolant ok");
            if(can_monitor)
                hal.port.set_pin_description(true, false, coolant_temp_port, "Coolant temperature");
        }

        if(hal.port.register_interrupt_handler) {
// add handler for flow stopped
        }

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        on_program_completed = grbl.on_program_completed;
        grbl.on_program_completed = onProgramCompleted;

        memcpy(&on_coolant_changed, &hal.coolant, sizeof(coolant_ptrs_t));
        hal.coolant.set_state = coolantSetState;

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;

    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
