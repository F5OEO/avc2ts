
using namespace std;
#include "ffmpegsrc.h"

#include <unistd.h>

ffmpegsrc::ffmpegsrc(const string &source) : source(source)
{
	open_device();
}

ffmpegsrc::~ffmpegsrc()
{
	close_device();
}
void ffmpegsrc::open_device(void)
{

	AVCodec *codec = NULL;

	unsigned int i;

	av_register_all();
	avdevice_register_all();
	avformat_network_init();

	//AVInputFormat *inputFormat =av_find_input_format("v4l2");

	/* Open the video */
	if (avformat_open_input(&format_ctx, source.c_str(), NULL /*inputFormat*/, NULL) < 0)
	{
		fprintf(stderr, "Error opening file '%s'\n", source.c_str());
		return;
	}

	/* Read stream info from the file */
	if (avformat_find_stream_info(format_ctx, NULL) < 0)
	{
		fprintf(stderr, "Error reading stream information from file\n");
	}

	/* Dump some useful information to stderr */
	fprintf(stderr, "Opening '%s'...\n", source.c_str());
	av_dump_format(format_ctx, 0, source.c_str(), 0);

	/* Find the first video stream */
	for (i = 0; i < format_ctx->nb_streams; i++)
	{
		if (format_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
		{
			break;
		}
	}

	if (i == format_ctx->nb_streams)
	{
		fprintf(stderr, "No video streams found\n");
		return;
	}

	video_stream = i;

	/* Get a pointer to the codec context for the video stream */
	codec_ctx = format_ctx->streams[i]->codec;

	/* Find the decoder for the video stream */
	codec = avcodec_find_decoder(codec_ctx->codec_id);
	if (codec == NULL)
	{
		fprintf(stderr, "Unsupported video codec\n");
		return;
	}

	/* Open codec */
	if (avcodec_open2(codec_ctx, codec, NULL) < 0)
	{
		fprintf(stderr, "Error opening video codec\n");
		return;
	}
	framey420 = av_frame_alloc();
	/* Allocate video frame */
	frame = av_frame_alloc();
	len = avpicture_get_size(codec_ctx->pix_fmt, codec_ctx->width, codec_ctx->height);
	fprintf(stderr, "ffmpeg YUV should %d\n", len);

	return;
}

void ffmpegsrc::close_device()
{

	av_free(framey420);
	av_free(frame);
	avformat_close_input(&format_ctx);

	return;
}

void ffmpegsrc::SetOmxBuffer(unsigned char *Buffer)
{
	OmxBuffer = Buffer;
	/* Assign appropriate parts of buffer to image planes in frame_rgb
	 * Note that frame_rgb is an AVFrame, but AVFrame is a superset
	 * of AVPicture */
	avpicture_fill((AVPicture *)framey420, Buffer, AV_PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);

	/* initialize SWS context for software scaling */
	sws_ctx = sws_getContext(
		codec_ctx->width,
		codec_ctx->height,
		codec_ctx->pix_fmt,
		codec_ctx->width,
		codec_ctx->height,
		AV_PIX_FMT_YUV420P,
		SWS_BICUBIC,
		NULL,
		NULL,
		NULL);
}

void ffmpegsrc::GetVideoSize(int &Width, int &Height)
{
	Width = codec_ctx->width;
	Height = codec_ctx->height;
}

bool ffmpegsrc::read_frame(int timeout = 1)
{

	AVPacket packet;
	//uint32_t *frame = NULL;
	int i = 0;
	int ret;
	while (i == 0)
	{

		while ((ret = av_read_frame(format_ctx, &packet)) >= 0)
		{
			/* Only handle frames from the video stream */
			if (packet.stream_index == video_stream)
			{
				/* Decode video frame */
				avcodec_decode_video2(codec_ctx, frame, &i, &packet);

				if (i)
				{
					sws_scale(
						sws_ctx,
						(uint8_t const *const *)frame->data,
						frame->linesize,
						0,
						codec_ctx->height,
						framey420->data,
						framey420->linesize);
					// memcpy(OmxBuffer,frame->data[0],len);
					av_free_packet(&packet);
					//printf("Frame %d\n",i);
					break;
				}
			}

			av_free_packet(&packet);
		}
		if (ret < 0)
			break;
	}

	return (ret >= 0);
}
