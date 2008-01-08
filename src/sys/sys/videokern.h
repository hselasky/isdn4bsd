/*-
 *  Video for Linux v2.0 kernel interface definition file
 *
 *  Copyright (C) 1999-2007 the contributors
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *  3. The names of its contributors may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *	See http://linuxtv.org for more info
 *
 *	Author: Bill Dirks <bill@thedirks.org>
 *		Justin Schoeman
 *		Hans Verkuil <hverkuil@xs4all.nl>
 *		et al.
 */

#ifndef __SYS_VIDEOKERN_H
#define	__SYS_VIDEOKERN_H

#if !defined(_KERNEL)
#error "This file should not be included by userland software!"
#endif

#include <sys/videodev.h>

/* prority handling */
struct v4l2_prio_state {
	volatile uint32_t prios[4];
};

/*  Video standard functions  */
uint32_t v4l2_video_std_fps(struct v4l2_standard *vs);
const char *v4l2_norm_to_name(v4l2_std_id id);
int32_t	v4l2_video_std_construct(struct v4l2_standard *vs, int32_t id, const char *name);

int32_t	v4l2_prio_init(struct v4l2_prio_state *global);
int32_t	v4l2_prio_change(struct v4l2_prio_state *global, enum v4l2_priority *local, enum v4l2_priority new);
int32_t	v4l2_prio_open(struct v4l2_prio_state *global, enum v4l2_priority *local);
int32_t	v4l2_prio_close(struct v4l2_prio_state *global, enum v4l2_priority *local);
enum v4l2_priority v4l2_prio_max(struct v4l2_prio_state *global);
int32_t	v4l2_prio_check(struct v4l2_prio_state *global, enum v4l2_priority *local);

/* names for fancy debug output */
extern char *v4l2_field_names[];
extern char *v4l2_type_names[];

/* common abstraction */
#ifdef __FreeBSD__
typedef struct usb_cdev v4l2_file_t;

#else
typedef struct file v4l2_file_t;

#endif

typedef int32_t (vidioc_enum_fmt_t)
        (v4l2_file_t *file, void *fh, struct v4l2_fmtdesc *f);

typedef int32_t (vidioc_xxx_fmt_t)
        (v4l2_file_t *file, void *fh, struct v4l2_format *f);

struct video_device {

	/* Device info */

	char	name[32];

	uint32_t type;			/* VID_TYPE_XXX */

	int32_t	debug;			/* Activates debug level */

	/* Video standard vars */
	v4l2_std_id tvnorms;		/* Supported tv norms */
	v4l2_std_id current_norm;	/* Current tvnorm */

	/* device operations */

	int32_t (*vid_open)
	        (v4l2_file_t *file, void *fh, uint8_t oread, uint8_t owrite);
	void    (*vid_start_read)
	        (v4l2_file_t *file, void *fh);
	void    (*vid_stop_read)
	        (v4l2_file_t *file, void *fh);
	void    (*vid_start_write)
	        (v4l2_file_t *file, void *fh);
	void    (*vid_stop_write)
	        (v4l2_file_t *file, void *fh);

	/* callbacks */
	void    (*release)
	        (struct video_device *vfd);

	/* ioctl callbacks */

	/* VIDIOC_QUERYCAP handler */
	int32_t (*vidioc_querycap)
	        (v4l2_file_t *file, void *fh, struct v4l2_capability *cap);

	/* Priority handling */
	int32_t (*vidioc_g_priority)
	        (v4l2_file_t *file, void *fh, enum v4l2_priority *p);
	int32_t (*vidioc_s_priority)
	        (v4l2_file_t *file, void *fh, enum v4l2_priority p);

	/* VIDIOC_ENUM_FMT handlers */
	vidioc_enum_fmt_t *vidioc_enum_fmt_cap;
	vidioc_enum_fmt_t *vidioc_enum_fmt_overlay;
	vidioc_enum_fmt_t *vidioc_enum_fmt_vbi;
	vidioc_enum_fmt_t *vidioc_enum_fmt_vbi_capture;
	vidioc_enum_fmt_t *vidioc_enum_fmt_video_output;
	vidioc_enum_fmt_t *vidioc_enum_fmt_vbi_output;
	vidioc_enum_fmt_t *vidioc_enum_fmt_type_private;

	/* VIDIOC_G_FMT handlers */
	vidioc_xxx_fmt_t *vidioc_g_fmt_cap;
	vidioc_xxx_fmt_t *vidioc_g_fmt_overlay;
	vidioc_xxx_fmt_t *vidioc_g_fmt_vbi;
	vidioc_xxx_fmt_t *vidioc_g_fmt_vbi_output;
	vidioc_xxx_fmt_t *vidioc_g_fmt_vbi_capture;
	vidioc_xxx_fmt_t *vidioc_g_fmt_video_output;
	vidioc_xxx_fmt_t *vidioc_g_fmt_type_private;

	/* VIDIOC_S_FMT handlers */
	vidioc_xxx_fmt_t *vidioc_s_fmt_cap;
	vidioc_xxx_fmt_t *vidioc_s_fmt_overlay;
	vidioc_xxx_fmt_t *vidioc_s_fmt_vbi;
	vidioc_xxx_fmt_t *vidioc_s_fmt_vbi_output;
	vidioc_xxx_fmt_t *vidioc_s_fmt_vbi_capture;
	vidioc_xxx_fmt_t *vidioc_s_fmt_video_output;
	vidioc_xxx_fmt_t *vidioc_s_fmt_type_private;

	/* VIDIOC_TRY_FMT handlers */
	vidioc_xxx_fmt_t *vidioc_try_fmt_cap;
	vidioc_xxx_fmt_t *vidioc_try_fmt_overlay;
	vidioc_xxx_fmt_t *vidioc_try_fmt_vbi;
	vidioc_xxx_fmt_t *vidioc_try_fmt_vbi_output;
	vidioc_xxx_fmt_t *vidioc_try_fmt_vbi_capture;
	vidioc_xxx_fmt_t *vidioc_try_fmt_video_output;
	vidioc_xxx_fmt_t *vidioc_try_fmt_type_private;

	/* Buffer handlers */
	int32_t (*vidioc_reqbufs)
	        (v4l2_file_t *file, void *fh, struct v4l2_requestbuffers *b);
	int32_t (*vidioc_querybuf)
	        (v4l2_file_t *file, void *fh, struct v4l2_buffer *b);
	int32_t (*vidioc_qbuf)
	        (v4l2_file_t *file, void *fh, struct v4l2_buffer *b);
	int32_t (*vidioc_dqbuf)
	        (v4l2_file_t *file, void *fh, struct v4l2_buffer *b);
	int32_t (*vidioc_overlay)
	        (v4l2_file_t *file, void *fh, uint32_t i);
	int32_t (*vidioc_g_fbuf)
	        (v4l2_file_t *file, void *fh, struct v4l2_framebuffer *a);
	int32_t (*vidioc_s_fbuf)
	        (v4l2_file_t *file, void *fh, struct v4l2_framebuffer *a);

	/* Stream on/off */
	int32_t (*vidioc_streamon)
	        (v4l2_file_t *file, void *fh, enum v4l2_buf_type i);
	int32_t (*vidioc_streamoff)
	        (v4l2_file_t *file, void *fh, enum v4l2_buf_type i);

	/*
	 * Standard handling G_STD and ENUMSTD are handled by videodev.c
	 */
	int32_t (*vidioc_s_std)
	        (v4l2_file_t *file, void *fh, v4l2_std_id * norm);
	int32_t (*vidioc_querystd)
	        (v4l2_file_t *file, void *fh, v4l2_std_id * a);

	/* Input handling */
	int32_t (*vidioc_enum_input)
	        (v4l2_file_t *file, void *fh, struct v4l2_input *inp);
	int32_t (*vidioc_g_input)
	        (v4l2_file_t *file, void *fh, uint32_t *i);
	int32_t (*vidioc_s_input)
	        (v4l2_file_t *file, void *fh, uint32_t i);

	/* Output handling */
	int32_t (*vidioc_enumoutput)
	        (v4l2_file_t *file, void *fh, struct v4l2_output *a);
	int32_t (*vidioc_g_output)
	        (v4l2_file_t *file, void *fh, uint32_t *i);
	int32_t (*vidioc_s_output)
	        (v4l2_file_t *file, void *fh, uint32_t i);

	/* Control handling */
	int32_t (*vidioc_queryctrl)
	        (v4l2_file_t *file, void *fh, struct v4l2_queryctrl *a);
	int32_t (*vidioc_g_ctrl)
	        (v4l2_file_t *file, void *fh, struct v4l2_control *a);
	int32_t (*vidioc_s_ctrl)
	        (v4l2_file_t *file, void *fh, struct v4l2_control *a);
	int32_t (*vidioc_g_ext_ctrls)
	        (v4l2_file_t *file, void *fh, struct v4l2_ext_controls *a);
	int32_t (*vidioc_s_ext_ctrls)
	        (v4l2_file_t *file, void *fh, struct v4l2_ext_controls *a);
	int32_t (*vidioc_try_ext_ctrls)
	        (v4l2_file_t *file, void *fh, struct v4l2_ext_controls *a);
	int32_t (*vidioc_querymenu)
	        (v4l2_file_t *file, void *fh, struct v4l2_querymenu *a);

	/* Audio ioctls */
	int32_t (*vidioc_enumaudio)
	        (v4l2_file_t *file, void *fh, struct v4l2_audio *a);
	int32_t (*vidioc_g_audio)
	        (v4l2_file_t *file, void *fh, struct v4l2_audio *a);
	int32_t (*vidioc_s_audio)
	        (v4l2_file_t *file, void *fh, struct v4l2_audio *a);

	/* Audio out ioctls */
	int32_t (*vidioc_enumaudout)
	        (v4l2_file_t *file, void *fh, struct v4l2_audioout *a);
	int32_t (*vidioc_g_audout)
	        (v4l2_file_t *file, void *fh, struct v4l2_audioout *a);
	int32_t (*vidioc_s_audout)
	        (v4l2_file_t *file, void *fh, struct v4l2_audioout *a);
	int32_t (*vidioc_g_modulator)
	        (v4l2_file_t *file, void *fh, struct v4l2_modulator *a);
	int32_t (*vidioc_s_modulator)
	        (v4l2_file_t *file, void *fh, struct v4l2_modulator *a);

	/* Crop ioctls */
	int32_t (*vidioc_cropcap)
	        (v4l2_file_t *file, void *fh, struct v4l2_cropcap *a);
	int32_t (*vidioc_g_crop)
	        (v4l2_file_t *file, void *fh, struct v4l2_crop *a);
	int32_t (*vidioc_s_crop)
	        (v4l2_file_t *file, void *fh, struct v4l2_crop *a);

	/* Compression ioctls */
	int32_t (*vidioc_g_jpegcomp)
	        (v4l2_file_t *file, void *fh, struct v4l2_jpegcompression *a);
	int32_t (*vidioc_s_jpegcomp)
	        (v4l2_file_t *file, void *fh, struct v4l2_jpegcompression *a);
	int32_t (*vidioc_g_enc_index)
	        (v4l2_file_t *file, void *fh, struct v4l2_enc_idx *a);
	int32_t (*vidioc_encoder_cmd)
	        (v4l2_file_t *file, void *fh, struct v4l2_encoder_cmd *a);
	int32_t (*vidioc_try_encoder_cmd)
	        (v4l2_file_t *file, void *fh, struct v4l2_encoder_cmd *a);

	/* Stream type-dependent parameter ioctls */
	int32_t (*vidioc_g_parm)
	        (v4l2_file_t *file, void *fh, struct v4l2_streamparm *a);
	int32_t (*vidioc_s_parm)
	        (v4l2_file_t *file, void *fh, struct v4l2_streamparm *a);

	/* Tuner ioctls */
	int32_t (*vidioc_g_tuner)
	        (v4l2_file_t *file, void *fh, struct v4l2_tuner *a);
	int32_t (*vidioc_s_tuner)
	        (v4l2_file_t *file, void *fh, struct v4l2_tuner *a);
	int32_t (*vidioc_g_frequency)
	        (v4l2_file_t *file, void *fh, struct v4l2_frequency *a);
	int32_t (*vidioc_s_frequency)
	        (v4l2_file_t *file, void *fh, struct v4l2_frequency *a);

	/* Sliced VBI cap */
	int32_t (*vidioc_g_sliced_vbi_cap)
	        (v4l2_file_t *file, void *fh, struct v4l2_sliced_vbi_cap *a);

	/* Log status ioctl */
	int32_t (*vidioc_log_status)
	        (v4l2_file_t *file, void *fh);

	/* Debugging ioctls */
#ifdef CONFIG_VIDEO_ADV_DEBUG
	int32_t (*vidioc_g_register)
	        (v4l2_file_t *file, void *fh, struct v4l2_register *reg);
	int32_t (*vidioc_s_register)
	        (v4l2_file_t *file, void *fh, struct v4l2_register *reg);
#endif

	/* Internal use only variables: */

};

/* Version 2 functions */
extern int32_t video_register_device(struct video_device *vfd, uint32_t type, uint32_t nr);
extern void video_unregister_device(struct video_device *);

/* helper functions to alloc and release "struct video_device",
 * the later can be used for "video_device->release()"
 */
struct video_device *video_device_alloc(void);
void	video_device_release(struct video_device *vfd);

/* helper functions to get and set the private data of
 * a "v4l2_file_t" structure
 */
void   *video_get_privdata(v4l2_file_t *file);
void	video_set_privdata(v4l2_file_t *file, void *priv);

#endif					/* __SYS_VIDEOKERN_H */
