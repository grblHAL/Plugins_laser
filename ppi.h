/*

  ppi.h - plugin for for laser PPI (Pulses Per Inch) mode

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

#ifndef _LASER_PPI_H_
#define _LASER_PPI_H_

typedef void (*ppi_spindle_onoff_ptr)(spindle_ptrs_t *spindle);

typedef struct {
    spindle_ptrs_t *spindle;
    ppi_spindle_onoff_ptr spindle_on;
    ppi_spindle_onoff_ptr spindle_off;
} laser_ppi_t;

void ppi_init (void);

#endif // _LASER_PPI_H_
