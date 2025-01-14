/*

  coolant.c - plugin for for handling laser coolant

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
    uint8_t coolant_ok_port;
    uint8_t coolant_temp_port;
} laser_coolant_settings_t;

static uint8_t coolant_ok_port, coolant_temp_port;
static bool coolant_on = false, monitor_on = false, can_monitor = false, coolant_off_pending = false;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static coolant_ptrs_t on_coolant_changed;
static nvs_address_t nvs_address;
static laser_coolant_settings_t coolant_settings;
static uint8_t n_ain, n_din;
static char max_aport[4], max_dport[4];

static void coolant_lost_handler (uint8_t port, bool state)
{
    if(coolant_on && !coolant_off_pending)
        system_set_exec_alarm(Alarm_AbortCycle);
}

static void coolant_flood_off (void *data)
{
    coolant_state_t mode = hal.coolant.get_state();
    mode.flood = Off;
    on_coolant_changed.set_state(mode);
    coolant_off_pending = coolant_on = false;
    sys.report.coolant = On; // Set to report change immediately
}

// Start/stop tube coolant, wait for ok signal on start if delay is configured.
static void coolantSetState (coolant_state_t mode)
{
    static bool irq_checked = false;

    bool changed = mode.flood != hal.coolant.get_state().flood || (mode.flood && coolant_off_pending);

    if(changed && !mode.flood) {

        if(coolant_settings.off_delay > 0.0f && !sys.reset_pending) {
            mode.flood = On;
            coolant_off_pending = task_add_delayed(coolant_flood_off, NULL, (uint32_t)(coolant_settings.off_delay * 60.0f * 1000.0f));
            on_coolant_changed.set_state(mode);
            return;
        }

        coolant_on = false;
    }

    on_coolant_changed.set_state(mode);

    if(changed && mode.flood) {
        task_delete(coolant_flood_off, NULL);
        coolant_off_pending = false;
        if(coolant_settings.on_delay > 0.0f && hal.port.wait_on_input(Port_Digital, coolant_ok_port, WaitMode_High, coolant_settings.on_delay) != 1) {
            mode.flood = Off;
            coolant_on = false;
            on_coolant_changed.set_state(mode);
            system_set_exec_alarm(Alarm_AbortCycle);
        } else
            coolant_on = true;
    }

    if(!irq_checked) {

        irq_checked = true;

        if(hal.port.get_pin_info) {
            xbar_t *port = hal.port.get_pin_info(Port_Digital, Port_Input, coolant_ok_port);
            if(port && (port->cap.irq_mode & IRQ_Mode_Falling))
                hal.port.register_interrupt_handler(coolant_ok_port, IRQ_Mode_Falling, coolant_lost_handler);
        }
    }

    monitor_on = mode.flood && (coolant_settings.min_temp + coolant_settings.max_temp) > 0.0f;
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static float coolant_temp_prev = 0.0f;

    char buf[20] = "";

    if(can_monitor) {

        float coolant_temp = (float)hal.port.wait_on_input(Port_Analog, coolant_temp_port, WaitMode_Immediate, 0.0f) / 10.0f;

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

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status;

    if((status = isintf(value) ? Status_OK : Status_BadNumberFormat) == Status_OK)
      switch(setting) {

        case Setting_LaserCoolantTempPort:
            coolant_settings.coolant_temp_port = value < 0.0f ? 0xFF : (uint8_t)value;
            break;

        case Setting_LaserCoolantOkPort:
            coolant_settings.coolant_ok_port = value < 0.0f ? 0xFF : (uint8_t)value;
            break;

        default: break;
    }

    return status;
}

static float get_port (setting_id_t setting)
{
    float value = -1.0f;

    switch(setting) {

        case Setting_LaserCoolantTempPort:
            value = coolant_settings.coolant_temp_port >= n_ain ? -1.0f : (float)coolant_settings.coolant_temp_port;
            break;

        case Setting_LaserCoolantOkPort:
            value = coolant_settings.coolant_ok_port >= n_din ? -1.0f : (float)coolant_settings.coolant_ok_port;
            break;

        default: break;
    }

    return value;
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    return n_ain > 0;
}

static const setting_detail_t plugin_settings[] = {
    { Setting_LaserCoolantOnDelay, Group_Coolant, "Laser coolant on delay", "seconds", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.on_delay, NULL, NULL },
    { Setting_LaserCoolantOffDelay, Group_Coolant, "Laser coolant off delay", "minutes", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.off_delay, NULL, NULL },
//    { Setting_LaserCoolantMinTemp, Group_Coolant, "Laser coolant min temp", "deg", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.min_temp, NULL, NULL, false },
    { Setting_LaserCoolantMaxTemp, Group_Coolant, "Laser coolant max temp", "deg", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.max_temp, NULL, is_setting_available },
    { Setting_LaserCoolantTempPort, Group_AuxPorts, "Coolant temperature port", NULL, Format_Decimal, "-#0", "-1", max_aport, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_LaserCoolantOkPort, Group_AuxPorts, "Coolant ok port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t plugin_settings_descr[] = {
    { Setting_LaserCoolantOnDelay, "" },
    { Setting_LaserCoolantOffDelay, "" },
    { Setting_LaserCoolantMaxTemp, "" },
    { Setting_LaserCoolantTempPort, "Aux port number to use for coolant temperature monitoring." },
    { Setting_LaserCoolantOkPort, "Aux port number to use for coolant ok signal." },
};

#endif

static void coolant_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&coolant_settings, sizeof(laser_coolant_settings_t), true);
}

static void coolant_settings_restore (void)
{
    coolant_settings.min_temp =
    coolant_settings.max_temp =
    coolant_settings.on_delay =
    coolant_settings.off_delay = 0.0f;

    coolant_settings.coolant_ok_port = ioport_find_free(Port_Digital, Port_Input, (pin_cap_t){ .claimable = On }, "Coolant ok");
    coolant_settings.coolant_temp_port = ioport_find_free(Port_Analog, Port_Input, (pin_cap_t){ .claimable = On }, "Coolant temperature");

    coolant_settings_save();
}

static void coolant_settings_load (void)
{
    bool ok = true;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&coolant_settings, nvs_address, sizeof(laser_coolant_settings_t), true) != NVS_TransferResult_OK)
        coolant_settings_restore();

    // Sanity checks
    if(coolant_settings.coolant_temp_port >= n_ain)
        coolant_settings.coolant_temp_port = 0xFF;
    if(coolant_settings.coolant_ok_port >= n_din)
        coolant_settings.coolant_ok_port = 0xFF;

    coolant_temp_port = coolant_settings.coolant_temp_port;
    coolant_ok_port = coolant_settings.coolant_ok_port;

    if((coolant_temp_port = coolant_settings.coolant_temp_port) != 0xFF)
        ok = (can_monitor = ioport_claim(Port_Analog, Port_Input, &coolant_temp_port, "Coolant temperature"));

    if(ok && (coolant_ok_port = coolant_settings.coolant_ok_port) != 0xFF)
        ok = ioport_claim(Port_Digital, Port_Input, &coolant_ok_port, "Coolant ok");

    if(ok) {

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        memcpy(&on_coolant_changed, &hal.coolant, sizeof(coolant_ptrs_t));
        hal.coolant.set_state = coolantSetState;
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Laser coolant", "0.06");
}

void laser_coolant_init (void)
{
    static setting_details_t setting_details = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = plugin_settings_descr,
        .n_descriptions = sizeof(plugin_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = coolant_settings_save,
        .load = coolant_settings_load,
        .restore = coolant_settings_restore,
    };

    if(ioport_can_claim_explicit() &&
       (n_din = ioports_available(Port_Digital, Port_Input)) &&
        (nvs_address = nvs_alloc(sizeof(laser_coolant_settings_t)))) {

        strcpy(max_dport, uitoa(n_din - 1));

        if((n_ain = ioports_available(Port_Analog, Port_Input)))
            strcpy(max_aport, uitoa(n_ain - 1));

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_register(&setting_details);

    } else
        protocol_enqueue_foreground_task(report_warning, "Laser coolant plugin failed to initialize!");
}

#endif // LASER_COOLANT_ENABLE
