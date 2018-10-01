
#include <string>
#include <memory> // unique_ptr
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavdevice/avdevice.h>
}

class ffmpegsrc
{

  public:
    ffmpegsrc(const std::string &source = "/dev/video0");

    ~ffmpegsrc();

    void GetVideoSize(int &Width, int &Height);

    void SetOmxBuffer(unsigned char *Buffer);
    bool read_frame(int timeout);

  private:
    void open_device();
    void close_device();

    std::string source;

    AVFormatContext *format_ctx;
    AVCodecContext *codec_ctx;
    struct SwsContext *sws_ctx;
    AVFrame *frame;
    AVFrame *framey420;
    uint8_t *OmxBuffer = NULL;
    int video_stream;
    int len;
};
