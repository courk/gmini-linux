/*************************************************************************************
*       Copyright (C) 2007-2016
*       Copyright ? 2007 Marvell International Ltd.
*
*       This program is free software; you can redistribute it and/or
*       modify it under the terms of the GNU General Public License
*       as published by the Free Software Foundation; either version 2
*       of the License, or (at your option) any later version.
*
*       This program is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY; without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*       GNU General Public License for more details.
*
*       You should have received a copy of the GNU General Public License
*       along with this program; if not, write to the Free Software
*       Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
***************************************************************************************/

#ifndef __PCM_H__
#define __PCM_H__

#include <linux/atomic.h>
#include <linux/input.h>

enum berlin_xrun_t {
	PCM_OVERRUN,
	FIFO_OVERRUN,
	PCM_UNDERRUN,
	FIFO_UNDERRUN,
	IRQ_DISABLE,
	XRUN_T_MAX
};

struct snd_berlin_chip {
	struct snd_card *card;
	struct snd_hwdep *hwdep;
	struct snd_pcm *pcm;
	struct input_dev *mic_mute_state_input;
	atomic_long_t xruns[XRUN_T_MAX];
	bool is_mic_mute;
	unsigned int zero_chunk_count;
};

void berlin_report_xrun(enum berlin_xrun_t xrun_type);

#endif
