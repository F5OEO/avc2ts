/*

   (c) 2014 Séverin Lemaignan <severin.lemaignan@epfl.ch>
   (c) 2008 Hans de Goede <hdegoede@redhat.com> for yuyv_to_rgb24

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or (at
 your option) any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA

 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <string.h> // strerrno
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdexcept>

#include "webcam.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define uint8 unsigned char
#define uint32 unsigned int
/*
 *  Copyright 2011 The LibYuv Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS. All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stdlib.h>
//#define HAVE_JPEG

#include "libyuv/convert.h"

#include "libyuv/video_common.h"

#ifdef __cplusplus
namespace libyuv
{
extern "C"
{
#endif

    // Convert camera sample to I420 with cropping, rotation and vertical flip.
    // src_width is used for source stride computation
    // src_height is used to compute location of planes, and indicate inversion
    // sample_size is measured in bytes and is the size of the frame.
    //   With MJPEG it is the compressed size of the frame.
    LIBYUV_API
    int ConvertToI420(const uint8 *sample,
                      size_t sample_size,
                      uint8 *y, int y_stride,
                      uint8 *u, int u_stride,
                      uint8 *v, int v_stride,
                      int crop_x, int crop_y,
                      int src_width, int src_height,
                      int crop_width, int crop_height,
                      enum RotationMode rotation,
                      uint32 fourcc)
    {
        uint32 format = CanonicalFourCC(fourcc);
        int aligned_src_width = (src_width + 1) & ~1;
        const uint8 *src;
        const uint8 *src_uv;
        const int abs_src_height = (src_height < 0) ? -src_height : src_height;
        // TODO(nisse): Why allow crop_height < 0?
        const int abs_crop_height = (crop_height < 0) ? -crop_height : crop_height;
        int r = 0;
        LIBYUV_BOOL need_buf = (rotation && format != FOURCC_I420 &&
                                format != FOURCC_NV12 && format != FOURCC_NV21 &&
                                format != FOURCC_YV12) ||
                               y == sample;
        uint8 *tmp_y = y;
        uint8 *tmp_u = u;
        uint8 *tmp_v = v;
        int tmp_y_stride = y_stride;
        int tmp_u_stride = u_stride;
        int tmp_v_stride = v_stride;
        uint8 *rotate_buffer = NULL;
        const int inv_crop_height =
            (src_height < 0) ? -abs_crop_height : abs_crop_height;

        if (!y || !u || !v || !sample ||
            src_width <= 0 || crop_width <= 0 ||
            src_height == 0 || crop_height == 0)
        {
            return -1;
        }

        // One pass rotation is available for some formats. For the rest, convert
        // to I420 (with optional vertical flipping) into a temporary I420 buffer,
        // and then rotate the I420 to the final destination buffer.
        // For in-place conversion, if destination y is same as source sample,
        // also enable temporary buffer.
        if (need_buf)
        {
            int y_size = crop_width * abs_crop_height;
            int uv_size = ((crop_width + 1) / 2) * ((abs_crop_height + 1) / 2);
            rotate_buffer = (uint8 *)malloc(y_size + uv_size * 2);
            if (!rotate_buffer)
            {
                return 1; // Out of memory runtime error.
            }
            y = rotate_buffer;
            u = y + y_size;
            v = u + uv_size;
            y_stride = crop_width;
            u_stride = v_stride = ((crop_width + 1) / 2);
        }

        switch (format)
        {
        // Single plane formats
        case FOURCC_YUY2:
            src = sample + (aligned_src_width * crop_y + crop_x) * 2;
            r = YUY2ToI420(src, aligned_src_width * 2,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_UYVY:
            src = sample + (aligned_src_width * crop_y + crop_x) * 2;
            r = UYVYToI420(src, aligned_src_width * 2,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_RGBP:
            src = sample + (src_width * crop_y + crop_x) * 2;
            r = RGB565ToI420(src, src_width * 2,
                             y, y_stride,
                             u, u_stride,
                             v, v_stride,
                             crop_width, inv_crop_height);
            break;
        case FOURCC_RGBO:
            src = sample + (src_width * crop_y + crop_x) * 2;
            r = ARGB1555ToI420(src, src_width * 2,
                               y, y_stride,
                               u, u_stride,
                               v, v_stride,
                               crop_width, inv_crop_height);
            break;
        case FOURCC_R444:
            src = sample + (src_width * crop_y + crop_x) * 2;
            r = ARGB4444ToI420(src, src_width * 2,
                               y, y_stride,
                               u, u_stride,
                               v, v_stride,
                               crop_width, inv_crop_height);
            break;
        case FOURCC_24BG:
            src = sample + (src_width * crop_y + crop_x) * 3;
            r = RGB24ToI420(src, src_width * 3,
                            y, y_stride,
                            u, u_stride,
                            v, v_stride,
                            crop_width, inv_crop_height);
            break;
        case FOURCC_RAW:
            src = sample + (src_width * crop_y + crop_x) * 3;
            r = RAWToI420(src, src_width * 3,
                          y, y_stride,
                          u, u_stride,
                          v, v_stride,
                          crop_width, inv_crop_height);
            break;
        case FOURCC_ARGB:
            src = sample + (src_width * crop_y + crop_x) * 4;
            r = ARGBToI420(src, src_width * 4,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_BGRA:
            src = sample + (src_width * crop_y + crop_x) * 4;
            r = BGRAToI420(src, src_width * 4,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_ABGR:
            src = sample + (src_width * crop_y + crop_x) * 4;
            r = ABGRToI420(src, src_width * 4,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_RGBA:
            src = sample + (src_width * crop_y + crop_x) * 4;
            r = RGBAToI420(src, src_width * 4,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        case FOURCC_I400:
            src = sample + src_width * crop_y + crop_x;
            r = I400ToI420(src, src_width,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        // Biplanar formats
        case FOURCC_NV12:
            src = sample + (src_width * crop_y + crop_x);
            src_uv = sample + (src_width * src_height) +
                     ((crop_y / 2) * aligned_src_width) + ((crop_x / 2) * 2);
            r = NV12ToI420Rotate(src, src_width,
                                 src_uv, aligned_src_width,
                                 y, y_stride,
                                 u, u_stride,
                                 v, v_stride,
                                 crop_width, inv_crop_height, rotation);
            break;
        case FOURCC_NV21:
            src = sample + (src_width * crop_y + crop_x);
            src_uv = sample + (src_width * src_height) +
                     ((crop_y / 2) * aligned_src_width) + ((crop_x / 2) * 2);
            // Call NV12 but with u and v parameters swapped.
            r = NV12ToI420Rotate(src, src_width,
                                 src_uv, aligned_src_width,
                                 y, y_stride,
                                 v, v_stride,
                                 u, u_stride,
                                 crop_width, inv_crop_height, rotation);
            break;
        case FOURCC_M420:
            src = sample + (src_width * crop_y) * 12 / 8 + crop_x;
            r = M420ToI420(src, src_width,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        // Triplanar formats
        case FOURCC_I420:
        case FOURCC_YV12:
        {
            const uint8 *src_y = sample + (src_width * crop_y + crop_x);
            const uint8 *src_u;
            const uint8 *src_v;
            int halfwidth = (src_width + 1) / 2;
            int halfheight = (abs_src_height + 1) / 2;
            if (format == FOURCC_YV12)
            {
                src_v = sample + src_width * abs_src_height +
                        (halfwidth * crop_y + crop_x) / 2;
                src_u = sample + src_width * abs_src_height +
                        halfwidth * (halfheight + crop_y / 2) + crop_x / 2;
            }
            else
            {
                src_u = sample + src_width * abs_src_height +
                        (halfwidth * crop_y + crop_x) / 2;
                src_v = sample + src_width * abs_src_height +
                        halfwidth * (halfheight + crop_y / 2) + crop_x / 2;
            }
            r = I420Rotate(src_y, src_width,
                           src_u, halfwidth,
                           src_v, halfwidth,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height, rotation);
            break;
        }
        case FOURCC_I422:
        case FOURCC_YV16:
        {
            const uint8 *src_y = sample + src_width * crop_y + crop_x;
            const uint8 *src_u;
            const uint8 *src_v;
            int halfwidth = (src_width + 1) / 2;
            if (format == FOURCC_YV16)
            {
                src_v = sample + src_width * abs_src_height +
                        halfwidth * crop_y + crop_x / 2;
                src_u = sample + src_width * abs_src_height +
                        halfwidth * (abs_src_height + crop_y) + crop_x / 2;
            }
            else
            {
                src_u = sample + src_width * abs_src_height +
                        halfwidth * crop_y + crop_x / 2;
                src_v = sample + src_width * abs_src_height +
                        halfwidth * (abs_src_height + crop_y) + crop_x / 2;
            }
            r = I422ToI420(src_y, src_width,
                           src_u, halfwidth,
                           src_v, halfwidth,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        }
        case FOURCC_I444:
        case FOURCC_YV24:
        {
            const uint8 *src_y = sample + src_width * crop_y + crop_x;
            const uint8 *src_u;
            const uint8 *src_v;
            if (format == FOURCC_YV24)
            {
                src_v = sample + src_width * (abs_src_height + crop_y) + crop_x;
                src_u = sample + src_width * (abs_src_height * 2 + crop_y) + crop_x;
            }
            else
            {
                src_u = sample + src_width * (abs_src_height + crop_y) + crop_x;
                src_v = sample + src_width * (abs_src_height * 2 + crop_y) + crop_x;
            }
            r = I444ToI420(src_y, src_width,
                           src_u, src_width,
                           src_v, src_width,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           crop_width, inv_crop_height);
            break;
        }
        /* case FOURCC_I411: {
      int quarterwidth = (src_width + 3) / 4;
      const uint8* src_y = sample + src_width * crop_y + crop_x;
      const uint8* src_u = sample + src_width * abs_src_height +
          quarterwidth * crop_y + crop_x / 4;
      const uint8* src_v = sample + src_width * abs_src_height +
          quarterwidth * (abs_src_height + crop_y) + crop_x / 4;
      r = I411ToI420(src_y, src_width,
                     src_u, quarterwidth,
                     src_v, quarterwidth,
                     y, y_stride,
                     u, u_stride,
                     v, v_stride,
                     crop_width, inv_crop_height);
      break;
    }*/
#ifdef HAVE_JPEG
        case FOURCC_MJPG:
            r = MJPGToI420(sample, sample_size,
                           y, y_stride,
                           u, u_stride,
                           v, v_stride,
                           src_width, abs_src_height, crop_width, inv_crop_height);
            break;
#endif
        default:
            r = -1; // unknown fourcc - return failure code.
        }

        if (need_buf)
        {
            if (!r)
            {
                r = I420Rotate(y, y_stride,
                               u, u_stride,
                               v, v_stride,
                               tmp_y, tmp_y_stride,
                               tmp_u, tmp_u_stride,
                               tmp_v, tmp_v_stride,
                               crop_width, abs_crop_height, rotation);
            }
            free(rotate_buffer);
        }

        return r;
    }

#ifdef __cplusplus
} // extern "C"
} // namespace libyuv
#endif

using namespace std;

static int xioctl(int fh, unsigned long int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

/*****
 * Taken from libv4l2 (in v4l-utils)
 *
 * (C) 2008 Hans de Goede <hdegoede@redhat.com>
 *
 * Released under LGPL
 */
/*
#define CLIP(color) (unsigned char)(((color) > 0xFF) ? 0xff : (((color) < 0) ? 0 : (color)))

static void v4lconvert_yuyv_to_rgb24(const unsigned char *src, 
                                     unsigned char *dest,
                                     int width, int height, 
                                     int stride)
{
    int j;

    while (--height >= 0) {
        for (j = 0; j + 1 < width; j += 2) {
            int u = src[1];
            int v = src[3];
            int u1 = (((u - 128) << 7) +  (u - 128)) >> 6;
            int rg = (((u - 128) << 1) +  (u - 128) +
                    ((v - 128) << 2) + ((v - 128) << 1)) >> 3;
            int v1 = (((v - 128) << 1) +  (v - 128)) >> 1;

            *dest++ = CLIP(src[0] + v1);
            *dest++ = CLIP(src[0] - rg);
            *dest++ = CLIP(src[0] + u1);

            *dest++ = CLIP(src[2] + v1);
            *dest++ = CLIP(src[2] - rg);
            *dest++ = CLIP(src[2] + u1);
            src += 4;
        }
        src += stride - (width * 2);
    }
}
*/
int Webcam::ConvertColor(unsigned char *out, unsigned char *in)
{

    unsigned char *inprocess = in;

    int WidthMissing = fmt.fmt.pix.width - yuv420frame.width;

    unsigned char *PlanY = out;
    unsigned char *PlanU = out + yuv420frame.width * yuv420frame.height;
    unsigned char *PlanV = PlanU + (yuv420frame.width * yuv420frame.height) / 4;

    //fprintf(stderr,"WidthMissin %d\n",WidthMissing);
    switch (fmt.fmt.pix.pixelformat)
    {
    case V4L2_PIX_FMT_UYVY:
        for (unsigned int j = 0; j < yuv420frame.height; j++)
        {
            for (unsigned int i = 0; i < yuv420frame.width / 2; i++)
            {

                *(PlanU) = *(inprocess++);
                if ((j % 2 == 0))
                    PlanU++;
                *(PlanY++) = *(inprocess++);
                *(PlanV) = *(inprocess++);
                if ((j % 2 == 0))
                    PlanV++;
                *(PlanY++) = *(inprocess++);
            }

            inprocess += WidthMissing * 2;
        }
        break;
    case V4L2_PIX_FMT_YUYV:
        for (unsigned int j = 0; j < yuv420frame.height; j++)
        {
            for (unsigned int i = 0; i < yuv420frame.width / 2; i++)
            {
                *(PlanY++) = *(inprocess++);
                *(PlanU) = *(inprocess++);
                if ((j % 2 == 0))
                    PlanU++;
                *(PlanY++) = *(inprocess++);
                *(PlanV) = *(inprocess++);
                if ((j % 2 == 0))
                    PlanV++;
            }

            inprocess += WidthMissing * 2;
        }
        break;
    }
    return 0;
    //fprintf(stderr,"Count =%d\n",count);
}

/*******************************************************************/

Webcam::Webcam(const string &device) : device(device)
{
    open_device();
    init_device();
    // xres and yres are set to the actual resolution provided by the cam

    // frame stored as UYVY
    yuv420frame.width = (xres >> 5) << 5;  //32 pixels aligned
    yuv420frame.height = (yres >> 4) << 4; //16 pixels aligned
    yuv420frame.size = (yuv420frame.width * yuv420frame.height * 3) / 2;
    //yuv420frame.data = (unsigned char *) malloc(yuv420frame.size * sizeof(char));
}

void Webcam::GetCameraSize(int &Width, int &Height)
{
    Width = yuv420frame.width;
    Height = yuv420frame.height;
}

void Webcam::SetOmxBuffer(unsigned char *Buffer)
{
    yuv420frame.data = Buffer;
}
Webcam::~Webcam()
{
    stop_capturing();
    uninit_device();
    close_device();

    //free(yuv420frame.data); Not its own buffer
}

const YUV420Image &Webcam::frame(int timeout)
{
    if (!StatusCapturing)
    {
        start_capturing();
        StatusCapturing = true;
    }
    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        tv.tv_sec = timeout;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;
            throw runtime_error("select");
        }

        if (0 == r)
        {
            fprintf(stderr,"Timeout\n");
            throw runtime_error(device + ": select timeout");
        }

        int idx = read_frame();
        if (idx != -1)
        {
            //ConvertColor(yuv420frame.data,(unsigned char *) buffers[idx].data);

            return yuv420frame;
        }
        else
            fprintf(stderr,"#");
        /* EAGAIN - continue select loop. */
    }
}

bool Webcam::read_frame()
{

    struct v4l2_buffer buf;
    //unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return -1;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            throw runtime_error("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);
    //fprintf(stderr,"Image =%d/%d\n",buf.bytesused,buffers[buf.index].size);
    if (buf.bytesused == buffers[buf.index].size)
    {
        ConvertColor(yuv420frame.data, (unsigned char *)buffers[buf.index].data);
    }
    else
    {
        //ConvertColor(yuv420frame.data,(unsigned char *) buffers[buf.index].data);
    }
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        throw runtime_error("VIDIOC_QBUF");

    return buf.index;
}

void Webcam::open_device(void)
{
    struct stat st;

    if (-1 == stat(device.c_str(), &st))
    {
        throw runtime_error(device + ": cannot identify! " + to_string(errno) + ": " + strerror(errno));
    }

    if (!S_ISCHR(st.st_mode))
    {
        throw runtime_error(device + " is no device");
    }

    fd = open(device.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        throw runtime_error(device + ": cannot open! " + to_string(errno) + ": " + strerror(errno));
    }
}

void Webcam::init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            throw runtime_error(device + " does not support memory mapping");
        }
        else
        {
            throw runtime_error("VIDIOC_REQBUFS");
        }
    }
    fprintf(stderr,"%d buffers for Video\n", req.count);
    if (req.count < 2)
    {
        throw runtime_error(string("Insufficient buffer memory on ") + device);
    }

    buffers = (buffer *)calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        throw runtime_error("Out of memory");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            throw runtime_error("VIDIOC_QUERYBUF");

        buffers[n_buffers].size = buf.length;
        buffers[n_buffers].data =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].data)
            throw runtime_error("mmap");
    }
}

void Webcam::close_device(void)
{
    if (-1 == close(fd))
        throw runtime_error("close");

    fd = -1;
}

void Webcam::init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    //struct v4l2_format fmt;
    //unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            throw runtime_error(device + " is no V4L2 device");
        }
        else
        {
            throw runtime_error("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        throw runtime_error(device + " is no video capture device");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        throw runtime_error(device + " does not support streaming i/o");
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (force_format)
    {
        fmt.fmt.pix.width = xres;
        fmt.fmt.pix.height = yres;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            throw runtime_error("VIDIOC_S_FMT");

        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
            // note that libv4l2 (look for 'v4l-utils') provides helpers
            // to manage conversions
            throw runtime_error("Webcam does not support YUYV format. Support for more format need to be added!");

        /* Note VIDIOC_S_FMT may change width and height. */
        xres = fmt.fmt.pix.width;
        yres = fmt.fmt.pix.height;

        stride = fmt.fmt.pix.bytesperline;
    }
    else
    {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw runtime_error("VIDIOC_G_FMT");
        xres = fmt.fmt.pix.width;
        yres = fmt.fmt.pix.height;
        switch (fmt.fmt.pix.pixelformat)
        {
        case V4L2_PIX_FMT_UYVY:
            fprintf(stderr,"Video Format UYVY supported\n");
            break;
        case V4L2_PIX_FMT_YUYV:
            fprintf(stderr,"Video Format YUYV supported\n");
            break;
        default:
            fprintf(stderr,"Video Format %d NOT IMPLENTED !!!!\n", fmt.fmt.pix.pixelformat);
        }
    }

    init_mmap();
}

void Webcam::uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].data, buffers[i].size))
            throw runtime_error("munmap");

    free(buffers);
}

void Webcam::start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            throw runtime_error("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        throw runtime_error("VIDIOC_STREAMON");
}

void Webcam::stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        throw runtime_error("VIDIOC_STREAMOFF");
}
