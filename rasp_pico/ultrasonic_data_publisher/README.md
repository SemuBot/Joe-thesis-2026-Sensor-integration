== How to compile program

1. Remove build folder and contents

2. mkdir build
3. cd build
4. cmake ..
5. make

cp build/ultrasonic_data_publisher.uf2 /media/semubot-laptop/RP2350/

Setup docker agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:kilted serial --dev /dev/ttyACM0 -b 115200