/*************************************************************************************
*       Copyright (C) 2007-2011
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

#ifndef __API_PLAYBACK_H__
#define __API_PLAYBACK_H__

int snd_berlin_playback_open(struct snd_pcm_substream *substream);

int snd_berlin_playback_close(struct snd_pcm_substream *substream);

int snd_berlin_playback_hw_free(struct snd_pcm_substream *substream);

int snd_berlin_playback_hw_params(struct snd_pcm_substream *substream,
                                        struct snd_pcm_hw_params *params);

int snd_berlin_playback_prepare(struct snd_pcm_substream *substream);

int snd_berlin_playback_trigger(struct snd_pcm_substream *substream, int cmd);

snd_pcm_uframes_t
snd_berlin_playback_pointer(struct snd_pcm_substream *substream);

int snd_berlin_playback_ack(struct snd_pcm_substream *substream);

int snd_berlin_playback_isr(struct snd_pcm_substream *substream);

#endif
