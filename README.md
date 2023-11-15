# ROS2_Reward_Dispenser
A ROS2 node that communicates with a microcontroller running the 'Serial-Relay' code. Part of the Delta task workspace.


## Installation
Make sure [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Windows-Install-Binary.html) or a newer ROS2 distribution is installed on your PC, as well as Python >=3.8, and the [Arduino IDE](https://www.arduino.cc/en/software) 

You will need to install some python packages using [pip](https://pip.pypa.io/en/stable/) if the dependencies aren't already met:

```
pip install pyserial bluetooth asyncio
```

You may need 3 additional libraries to run the sketch with the weight sensing and LED matrix display: `MD_Parola`, `MD_MAX72XX`, and `Queuetue HX711 Library`. These libraries can be installed using the built-in Library Manager from the `Tools > Manage Libraries` menu:
 - Queuetue HX711
 - MD_MAX27XX
 - MD_Parola

Clone the repository in your ros workspace, build with colcon, finally source
``` 
cd /path/to/ros/workspace/src
git clone https://github.com/Neuro-Mechatronics-Interfaces/ROS2_Reward_Dispenser.git src/ros2_reward_dispenser
colcon build --packages-select reward_dispenser
```
+ For Windows 10 source using `call install/local_setup.bat`
+ For UBuntu use `source install/setup.bash`

Use the [Arduino IDE](https://www.arduino.cc/en/software) to flash the accompanied `serial_relay_scale_LEMatrix.ino` script onto the microcontroller. 

#### Device Permissions
Device permissions may also need to be set up if interfacing with a USB device for the first time. Add the user to the `dialout` and `tty` groups:
```
sudo usermod -a -G dialout your-username
sudo usermod -a -G tty your-username
sudo chmod a+rw /dev/ttyACM*
```

If using the ROS2 node, note the COM port assigned to it for the config file.

## Quick Start ##
To quickly run the node, enter the following command in the terminal:
```
ros2 run reward_dispenser node
```

The default serial port is set to `/dev/ttyACM0` but is also a configurable parameter. To define a specific port or baudrate, pass it as an argument:
```
ros2 run reward_dispenser node --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
```

The parameters can also be defined using the provided example `.yaml` file and passed as an argument:
```
ros2 run reward_dispenser node --params-file ~/ros2_ws/src/ros2_reward_dispenser/reward_dispenser/config/reward.yaml
```

The reward dispener node is monitoring a topic called `manual_dispense` which will send a 200ms dispense command whenever a new boolean value is published to it.

## Troubleshooting ##
 - If you get the `ImportError: You must be root to use this library on linux` error message [issue](https://github.com/boppreh/keyboard/issues/420), you may need to run the node with admin privilidges. Adding the current user to sudo is an option that should fix it, another option is to create device rules: 

```
pip install git+https://github.com/boppreh/keyboard.git#egg=keyboard
sudo usermod -a -G tty,input $USER
sudo chmod +0666 /dev/uinput
echo 'KERNEL=="uinput", TAG+="uaccess""' > /etc/udev/rules.d/50-uinput.rules
echo 'SUBSYSTEM=="input", MODE="0666" GROUP="plugdev"' > /etc/udev/rules.d/12-input.rules
echo 'SUBSYSTEM=="misc", MODE="0666" GROUP="plugdev"' >> /etc/udev/rules.d/12-input.rules
echo 'SUBSYSTEM=="tty", MODE="0666" GROUP="plugdev"' >> /etc/udev/rules.d/12-input.rules
loginctl terminate-user $USER
```

If you get an error message relating to the user not being in the `input` group, run the following command to add user to the required group.
```
sudo usermod -a -G input USERNAME
```


