xhost +local:

docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /dev/bus/usb/001/007:/dev/bus/usb/001/007 \
-v /dev/bus/usb/001/009:/dev/bus/usb/001/009 \
--device=/dev/astradepth \
--device=/dev/astrauvc \
--device=/dev/video0 \
--device=/dev/myserial \
--device=/dev/rplidar \
--device=/dev/ttyUSB0 \
--device=/dev/ttyUSB1 \
--runtime nvidia \
-p 9090:9090 \
-p 8888:8888 \
nvcr.io/nvidia/l4t-jetpack:r36.3.0 /bin/bash

