mkfifo audioin.wav
arecord -f S16_LE -r 24000 -c 2 -B 40000 -D plughw:1 > audioin.wav &
./avc2ts -a audioin.wav -z 10000 -b 780000 -m 900000 -x 1920 -y 1080 -f 25 -d 00 -n 230.0.0.10:10000 

