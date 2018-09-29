mkfifo audioin.wav
arecord -f S16_LE -r 48000 -c 2 -B 100 -D plughw:1 > audioin.wav &
./avc2ts -b 300000 -m 400000 -x 352 -y 288 -f 25 -n 230.0.0.10:10000 

