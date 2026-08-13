/*

  lb_clusters.c - plugin for unpacking LightBurn cluster encoded engraving moves.

  Part of grblHAL

  Copyright (c) 2022-2026 Terje Io

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

#if LB_CLUSTERS_ENABLE

#include "grbl/hal.h"
#include "grbl/gcode.h"
#include "grbl/protocol.h"

#include <string.h>

#ifndef LB_CLUSTER_SIZE
#define LB_CLUSTER_SIZE 16
#endif

#ifndef LB_SVALUE_SCALING
#define LB_SVALUE_SCALING 0 // Change to 1 if S-values is to be multiplied by spindle max RPM.
#endif

static struct {
    char block[LINE_BUFFER_SIZE];
    char *s;
    char *cmd;
    char eol;
    bool overflow;
    volatile uint_fast16_t length;
} input = {0};

static struct cluster {
    char block[LINE_BUFFER_SIZE];
    char param[LINE_BUFFER_SIZE + 40];
    char *s;
    char *cmd;
    uint_fast8_t count;
    uint_fast8_t next;
    bool restore_s;
    char sval[LB_CLUSTER_SIZE][11];
} cluster;

static stream_read_ptr file_read = NULL, stream_read = NULL;
static on_stream_changed_ptr on_stream_changed;
static on_report_handlers_init_ptr on_report_handlers_init;
static status_message_ptr status_message = NULL;
static on_report_options_ptr on_report_options;
static on_reset_ptr on_reset;

static inline char *get_value (char *v, uint_fast8_t *offset, uint_fast16_t scale)
{
    float val;

    read_float(v, offset, &val);

    val /= (float)scale;

    return ftoa(val, val < 1.0f ? 10 : (val < 1000.0f ? 8 : 4));
}

#if LB_SVALUE_SCALING

static inline char *get_s_value (char *v)
{
    char *s;
    float val;
    uint_fast8_t cc = 0;

    read_float(v, &cc, &val);

    s = ftoa(val * gc_spindle_get(-1)->hal->rpm_max, 0);

    *strchr(s, '.') = input.eol;

    return s;
}

#endif

static inline int_fast8_t parse_block (void)
{
    if(input.length > 5 && !strncasecmp(input.block, "G1", 2) && strchr(input.block, ':') && (strchr(input.block, 'S') || strchr(input.block, 's'))) {

        char c, *s = cluster.block, *s2 = input.block;
        uint_fast8_t params = 0;

        *strchr(s2, input.eol) = '\0';

        while((c = *s2++)) {
            if(c == 'S' || c == 's')
                break;
            if(c != ' ')
                *s++ = c;
        }
        *s = '\0';

        cluster.s = s + 1;
        cluster.cmd = cluster.block;
        strcpy(cluster.param, cluster.block);
        cluster.count = cluster.next = 0;

        s2 = strtok(s2, ":");
        while(s2) {
            if(cluster.count == LB_CLUSTER_SIZE)
                return -1;
#if LB_SVALUE_SCALING
            strcpy(cluster.sval[cluster.count++] + 1, get_s_value(s2));
#else
            strcpy(cluster.sval[cluster.count] + 1, s2);
            strcat(cluster.sval[cluster.count++], "\n");
#endif
            s2 = strtok(NULL, ":");
        }

        s = cluster.cmd + 3;
        s2 = get_value(s, &params, cluster.count);
        if(strchr(s, '\0') != s + params)
            strcpy(cluster.param, s + params);
        else
            *cluster.param = '\0';
        strcpy(s, s2);
        cluster.s = strchr(s, '\0');
        while(*(cluster.s - 1) == '0')
            *(--cluster.s) = '\0';
    }

    return (int32_t)cluster.count;
}

static inline bool make_block (void)
{
    if(*cluster.param) {
        strcat(strcpy(cluster.s, cluster.param), cluster.sval[0]);
        *cluster.param = '\0';
    } else
        strcpy(cluster.s, cluster.sval[cluster.next]);

    if(++cluster.next == cluster.count) {
        if((cluster.restore_s = strcmp(cluster.s, cluster.sval[0])))
            strcat(strcat(cluster.s, "G1X0"), cluster.sval[0]);
    }
    input.s = cluster.cmd;
    input.length = strlen(cluster.cmd);

    return cluster.next == cluster.count;
}

// File stream decoder

static inline void file_fill_buffer (void)
{
    int16_t c;

    if(cluster.count == 0) {

        char *s = input.block;

        input.s = s;
        input.length = 0;
		cluster.restore_s = false;

        while((c = file_read()) != SERIAL_NO_DATA) {

            if(++input.length >= LINE_BUFFER_SIZE - 1) {

                if(!input.overflow)
                    system_raise_alarm(Alarm_BufferOverflow);

                input.overflow = true;

                return;
            }

            *s++ = (char)c;
            if((char)c == '\n' || (char)c == '\r') {
                if(input.length == 1 && input.eol && input.eol != (char)c) {
                    input.eol = '\0';
                    input.s = s;
                    input.length = 0;
                }
                input.eol = c;
                break;
            }
        }

        *s = '\0';

        if(parse_block() < 0) {
            cluster.count = input.length = 0;
            return;
        }
    }

    if(cluster.count && make_block())
        cluster.count = 0;
}

FLASHMEM static int32_t file_decoder (void)
{
    int32_t c;

    if(input.length == 0)
        file_fill_buffer();

    if(input.length) {
        c = *input.s++;
        input.length--;
    } else
        c = SERIAL_NO_DATA;

    return c;
}

// "Normal" stream decoder

FLASHMEM static int32_t stream_fill_buffer (void)
{
    static char *s = NULL;

    int32_t c;

    if(s == NULL || input.s == NULL) {
        input.s = s = input.block;
        cluster.count = input.length = 0;
        cluster.restore_s = false;
    }

    if(cluster.count == 0) {

        c = stream_read();

        if(c == SERIAL_NO_DATA || c == ASCII_CAN) {

            if(ABORTED) {
                cluster.count = input.length = 0;
                s = NULL;
            }

            return c;
        }

        if(++input.length >= LINE_BUFFER_SIZE - 1) {

            if(!input.overflow)
                system_raise_alarm(Alarm_BufferOverflow);

            input.overflow = true;

            return SERIAL_NO_DATA;
        }

        *s++ = (char)c;

        if((char)c == '\n' || (char)c == '\r') {
            if(input.length == 1 && input.eol && input.eol != (char)c) {
                input.eol = '\0';
                input.length = 0;
                s--;
                return SERIAL_NO_DATA;
            }
            input.eol = (char)c;
        } else
            return SERIAL_NO_DATA;

        *s = '\0';

        switch(parse_block()) {

            case -1:
                s = NULL;
                return SERIAL_NO_DATA;

            case 0:
                s = NULL;
                break;
        }
    }

    if(cluster.count && make_block()) {
        s = NULL;
        cluster.count = 0;
    }

    return 0;
}

FLASHMEM static int32_t stream_decoder (void)
{
    static bool buffering = true;

    int32_t c;

    if(buffering) {
        while((c = stream_fill_buffer()) != 0) {
            if(ABORTED)
                break;
            return c;
        }
        buffering = false;
    }

    if(input.length) {
        c = *input.s++;
        input.length--;
    } else {
        buffering = true;
        c = SERIAL_NO_DATA;
    }

    return c;
}

// Only respond with a single "ok" message for each cluster
// or terminate cluster unpacking if error status reported.
FLASHMEM static status_code_t cluster_status_message (status_code_t status_code)
{
    if(status_code != Status_OK) {
        status_message(status_code);
        if(cluster.next) {
            input.s = NULL;
            cluster.count = cluster.next = input.length = 0;
        }
    } else if(cluster.count == 0) {
        if(cluster.restore_s)
            cluster.restore_s = false;
        else
            status_message(status_code);
    }

    return status_code;
}

FLASHMEM static void reset_data (void)
{
    memset(&input, 0, sizeof(input));
    memset(&cluster, 0, offsetof(struct cluster, sval));
}

FLASHMEM static void stream_changed (stream_type_t type)
{
    if(on_stream_changed)
        on_stream_changed(type);

    if(type == StreamType_File) {
        file_read = hal.stream.read;
        hal.stream.read = file_decoder;
    } else if(hal.stream.read != stream_decoder) {
        stream_read = hal.stream.read;
        hal.stream.read = stream_decoder;
    }

    reset_data();
}

FLASHMEM static void cluster_reset (void)
{
    if(on_reset)
        on_reset();

    reset_data();
}

FLASHMEM static void cluster_report (void)
{
    if(on_report_handlers_init)
        on_report_handlers_init();

    status_message = grbl.report.status_message;
    grbl.report.status_message = cluster_status_message;
}

FLASHMEM static void report_options (bool newopt)
{
    if(!newopt) {
        hal.stream.write("[CLUSTER:");
        hal.stream.write(uitoa(LB_CLUSTER_SIZE));
        hal.stream.write("]" ASCII_EOL);
        hal.stream.write("[PLUGIN:LightBurn clusters v0.07]" ASCII_EOL);
    }

    on_report_options(newopt);
}

FLASHMEM void lb_clusters_init (void)
{
    uint_fast8_t idx = LB_CLUSTER_SIZE;

    do {
        *cluster.sval[--idx] = 'S';
    } while(idx);

    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = stream_changed;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    on_reset = grbl.on_reset;
    grbl.on_reset = cluster_reset;

    on_report_handlers_init = grbl.on_report_handlers_init;
    grbl.on_report_handlers_init = cluster_report;

    stream_changed(hal.stream.type);
}

#endif // LB_CLUSTERS_ENABLE
