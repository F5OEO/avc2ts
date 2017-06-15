git clone git://github.com/F5OEO/libmpegts
cd libmpegts
./configure
make
cd ../

git clone https://chromium.googlesource.com/libyuv/libyuv
cd libyuv
#should patch linux.mk with -DHAVE_JPEG on CXX and CFLAGS
#seems to be link with libjpeg9-dev
make V=1 -f linux.mk
cd ../

# Required for ffmpegsrc.cpp
sudo apt-get -y install libvncserver-dev libavcodec-dev libavformat-dev libswscale-dev libavdevice-dev

# For transcoding youtube
curl -L https://yt-dl.org/downloads/latest/youtube-dl -o youtube-dl
chmod +x youtube-dl
# Example :  ./youtube-dl -o - https://www.youtube.com/watch?v=gOlqWk3E7Sc | ./avc2ts -b 2100000 -m 2530000 -x 640 -y 480 -f 25 -n 230.0.0.1:10000 -t 5 -i 25 -d 140 -e /dev/stdin -v

