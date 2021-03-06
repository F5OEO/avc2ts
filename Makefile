CC=g++
LD=g++

CFLAGS = -std=c++11 -fpermissive -Wno-deprecated-declarations -Wno-write-strings -Wno-multichar -Wall -Wno-format -g -I/opt/vc/include/IL -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux -Ilibyuv/include -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -L/usr/local/lib -I/usr/local/include -O3

LDFLAGS = -Xlinker -R/opt/vc/lib -L/opt/vc/lib/ -Xlinker -L/usr/local/lib -Xlinker -R/usr/local/lib   # -Xlinker --verbose
#LIBS = -L/opt/vc/lib  -rdynamic -lopenmaxil -lvcos -lbcm_host -lpthread -L/libmpegts/libmpegts.o -lm 
LIBS = -lopenmaxil -lbcm_host -lvcos -lpthread -lm -lrt -lvncclient -ljpeg -lfdk-aac

CFLAGS+=`pkg-config --cflags libavcodec libavformat libswscale libavutil libavdevice`
LDFLAGS+=`pkg-config --libs libavcodec libavformat libswscale libavutil libavdevice`

OFILES = vncclient.o grabdisplay.o avc2ts.o webcam.o ffmpegsrc.o b101.o ./libmpegts/libmpegts.a libyuv/libyuv.a 

.PHONY: all clean install dist

all: avc2ts

.cpp.o:
	$(CC) $(CFLAGS) -c $<

avc2ts: $(OFILES)
	$(CC) $(LDFLAGS) $(LIBS) -o avc2ts  $(OFILES)

clean:
	rm -f *.o avc2ts
	
install:
	cp avc2ts ../../bin/

dist: clean
	
