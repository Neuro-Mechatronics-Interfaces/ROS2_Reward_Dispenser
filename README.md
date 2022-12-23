# ROS2_Reward_Dispenser
A ROS2 node that communicates with a microcontroller running the 'Serial-Relay' code. Part of the Delta task workspace.


## Installation
- [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Windows-Install-Binary.html)
- Python 3.8 with package dependencies: serial, bluetooth, asyncio which can be install by [pip](https://pip.pypa.io/en/stable/).
```
pip install pyserial bluetooth asyncio
```
Clone the repository in your ros workspace, build with colcon, finally source
``` 
git clone https://github.com/Neuro-Mechatronics-Interfaces/ROS2_Reward_Dispenser.git 
cd ..
colcon build --packages-select reward_dispenser
```
+ For Windows 10 source using `call install/local_setup.bat`
+ For UBuntu use `source install/setup.bash`

Use the [Arduino IDE](https://www.arduino.cc/en/software) to flash the accompanied `serial_relay.ino` script onto the microcontroller. Note the COM port assigned to it for the config file.

## Quick Start ##
1. Open the `reward_serial_node_2` script and change the default port to match the locally assigned port. 
   + In the future, there will be a `config.yaml` file and fill in the `COM` port value to match the locally assigned microcontroller port. Leave `Baudrate` alone if it wasn't changed in the script.

2. Run the reward dispenser node 
```
ros2 run reward_dispenser reward
```

