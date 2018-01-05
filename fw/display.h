/* Megumin LED display firmware
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "global.h"
#include "transpose.h"

extern volatile struct framebuf *read_fb, *write_fb;

enum FB_OPERATION { FB_WRITE, FB_FORMAT, FB_UPDATE };
extern volatile enum FB_OPERATION fb_op;

extern volatile int nbits;
extern volatile int frame_duration_us;

void display_init(void);

#endif/*__DISPLAY_H__*/
