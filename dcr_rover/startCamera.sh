#!/bin/bash

cd ~/Downloads/mjpg-streamer/mjpg-streamer-experimental

./mjpg_streamer -o "output_http.so -w ./www -p 8080" -i "input_uvc.so -d /dev/video0" &
./mjpg_streamer -o "output_http.so -w ./www -p 8090" -i "input_uvc.so -d /dev/video2" &
./mjpg_streamer -o "output_http.so -w ./www -p 8091" -i "input_uvc.so -d /dev/video4" &

wait
