## Compiling and Upload firmware to MCU.

Works with Arduino 1.8.13 and Teensyduino 1.53, or VSCode+PlatformIO!

First, generate `ros_lib` libraries from your catkin_workspace.

    $ source devel/setup.bash
    $ rosrun rosserial_arduino make_libraries.py .

Copy `ros_lib` directory to arduino library directory.
Compile `bioin-tacto-firmware.ino`, upload it to your MCU.

Run using:

    $ rosrun rosserial_python serial_node.py _baud:=250000 _port:=/dev/ttyACM0
