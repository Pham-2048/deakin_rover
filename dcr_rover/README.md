# Start the Camera

```bash
cd ~/Downloads/mjpg-streamer/mjpg-streamer-experimental

./mjpg_streamer -o "output_http.so -w ./www -p 8080" -i "input_uvc.so -d /dev/video0"

./mjpg_streamer -o "output_http.so -w ./www -p 8090" -i "input_uvc.so -d /dev/video2"

./mjpg_streamer -o "output_http.so -w ./www -p 8091" -i "input_uvc.so -d /dev/video4"
```

# Script to run inside docker container
```bash
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000

ros2 launch rover_bringup bringup.launch.xml
```