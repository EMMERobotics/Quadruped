# EMME Robotics Quadruped

## Guidelines
- Build with `catkin build`
- C++11 is used

### Dependencies for ESP32
[ESP32-PS3](https://github.com/jvpernis/esp32-ps3) -- Download this to your Arduino
library directory 
 **for running on ESP32 with PS3 Controller**

## Dependencies for Raspberry Pi 4B

### OS
Ubuntu 20.04 LTS can be flashed to the SD Card with Raspberry Pi Imager.

### ROS
Install ROS **NOETIC** headless (base version) as you normally do on desktop.

### Adafruit PCA9685 Servo Driver
*If getting an error message about* ***"board" not found or "board" has no attribute*** -> [Circuit Python FAQ](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/faq-troubleshooting)

**Pre-requistites**
- python3
- pip
- setuptools
```
sudo pip install --upgrade setuptools
```

**Libraries**

PlatformDetect
```
sudo pip install Adafruit-PlatformDetect
```

Circuit python
```
sudo pip install adafruit-blinka
```

circuitpython-pca9685
```
sudo pip install adafruit-circuitpython-pca9685
```

circuitpython-servokit
```
sudo pip install adafruit-circuitpython-servokit
```

circuitpython-motor
```
sudo pip install adafruit-circuitpython-motor
```
**NOTICE:** sudo is required to run the motor python code, therefore it is crucial to install the library system wide.

## Resources

### ROS
Cmake guide: [ROS_Wiki/catkin/cmakelists.txt](http://wiki.ros.org/catkin/CMakeLists.txt#target_link_libraries)

### Adafruit
[Servo Driver Circuit Python Guide](https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython)

[github: circuitpython bundle](https://github.com/adafruit/Adafruit_CircuitPython_Bundle)

[github: circuitpython-motor](https://github.com/adafruit/Adafruit_CircuitPython_Motor/tree/31c819f377cf71f61cfb84eae159f1f948980db7)

[github: cicuitpython-pca9685](https://github.com/adafruit/Adafruit_CircuitPython_PCA9685)