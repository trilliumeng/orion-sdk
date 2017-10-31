#ifndef FFMPEG_H
#define FFMPEG_H

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/rational.h"
#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)

#define av_frame_unref(x)
#define av_frame_get_buffer get_video_buffer
#define av_frame_copy frame_copy_video

#define AV_PIX_FMT_RGB24 PIX_FMT_RGB24

static void get_frame_defaults(AVFrame *frame)
{
    if (frame->extended_data != frame->data)
        av_freep(&frame->extended_data);

    memset(frame, 0, sizeof(*frame));

    frame->pts                 =
    frame->pkt_dts             = AV_NOPTS_VALUE;
    frame->pkt_pts             = AV_NOPTS_VALUE;
    frame->key_frame           = 1;
    frame->sample_aspect_ratio = (AVRational){ 0, 1 };
    frame->format              = -1; /* unknown */
    frame->extended_data       = frame->data;
}

AVFrame *av_frame_alloc(void)
{
    AVFrame *frame = av_mallocz(sizeof(*frame));

    if (!frame)
        return NULL;

    frame->extended_data = NULL;
    get_frame_defaults(frame);

    return frame;
}

void av_frame_free(AVFrame **frame)
{
    if (!frame || !*frame)
        return;

    av_frame_unref(*frame);
    av_freep(frame);
}

#define av_pix_fmt_desc_get(x) &av_pix_fmt_descriptors[x]

int av_pix_fmt_count_planes(enum PixelFormat pix_fmt)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    int i, planes[4] = { 0 }, ret = 0;

    if (!desc)
        return AVERROR(EINVAL);

    for (i = 0; i < desc->nb_components; i++)
        planes[desc->comp[i].plane] = 1;
    for (i = 0; i < FF_ARRAY_ELEMS(planes); i++)
        ret += planes[i];
    return ret;
}

static int frame_copy_video(AVFrame *dst, const AVFrame *src)
{
    const uint8_t *src_data[4];
    int i, planes;

    if (dst->width  < src->width ||
        dst->height < src->height)
        return AVERROR(EINVAL);

    planes = av_pix_fmt_count_planes(dst->format);
    for (i = 0; i < planes; i++)
        if (!dst->data[i] || !src->data[i])
            return AVERROR(EINVAL);

    memcpy(src_data, src->data, sizeof(src_data));
    av_image_copy(dst->data, dst->linesize,
                  src_data, src->linesize,
                  dst->format, src->width, src->height);

    return 0;
}

#define av_buffer_alloc av_mallocz
#define AV_CEIL_RSHIFT(a,b) (!av_builtin_constant_p(b) ? -((-(a)) >> (b)) \
                                                       : ((a) + (1<<(b)) - 1) >> (b))

static int get_video_buffer(AVFrame *frame, int align)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frame->format);
    int ret, i;

    if (!desc)
        return AVERROR(EINVAL);

    if ((ret = av_image_check_size(frame->width, frame->height, 0, NULL)) < 0)
        return ret;

    if (!frame->linesize[0]) {
        if (align <= 0)
            align = 32; /* STRIDE_ALIGN. Should be av_cpu_max_align() */

        for(i=1; i<=align; i+=i) {
            ret = av_image_fill_linesizes(frame->linesize, frame->format,
                                          FFALIGN(frame->width, i));
            if (ret < 0)
                return ret;
            if (!(frame->linesize[0] & (align-1)))
                break;
        }

        for (i = 0; i < 4 && frame->linesize[i]; i++)
            frame->linesize[i] = FFALIGN(frame->linesize[i], align);
    }

    for (i = 0; i < 4 && frame->linesize[i]; i++) {
        int h = FFALIGN(frame->height, 32);
        if (i == 1 || i == 2)
            h = AV_CEIL_RSHIFT(h, desc->log2_chroma_h);

        frame->data[i] = av_buffer_alloc(frame->linesize[i] * h + 16 + 16/*STRIDE_ALIGN*/ - 1);
        if (!frame->data[i])
            goto fail;
    }

    frame->extended_data = frame->data;

    return 0;
fail:
    av_frame_unref(frame);
    return AVERROR(ENOMEM);
}

#endif

#endif //  FFMPEG_H
