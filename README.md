# OmniPicker ROS2

## Compatibility

| **Supported OS**          | **Supported ROS2 distribution**                         |
|---------------------------|---------------------------------------------------------|
| Ubuntu 24.04              | [Jazzy](https://docs.ros.org/en/jazzy/index.html)       |

|**Name**        | **Protocol**              | **Test USBTOCANFD Hardware**        |
|----------------------|---------------------------|-------------------------------------|
|[OmniPicker](https://www.zhiyuan-robot.com/DOCS/PM/X1) | CANFD                     | ZQWL-UCANFD-100K                    |

## Getting Started

This project was developed for ROS2 Jazzy on Ubuntu 24.04. Other versions of Ubuntu and ROS2 may work, but not supported.

1. Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. Install `colcon` and additional ROS package:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```
3. Setup workspace:

    ```bash
    mkdir -p ~/omnipicker_ws/src
    cd ~/omnipicker_ws/src
    git clone -b main https://github.com/XieShaosong/omnipicker_ros2.git
    ```
4. Install dependencies:

    ```bash
    cd ~/omnipicker_ros2
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
    ```

5. Build and source the workspace:

    ```bash
    cd ~/omnipicker_ros2
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

**NOTE**: Remember to source the setup file and the workspace whenever a new terminal is opened:

```bash
source /opt/ros/jazzy/setup.bash
source ~/omnipicker_ros2/install/setup.bash
```

## Protocol Description
### Topic: omnipicker_state
Frequency: 10Hz
Message Type: omnipicker_interfaces/msg/OmniPickerState
```yaml
uint8 can0_flag           #can0 enable flag
uint8 can0_state          #can0 state
uint8[] raw_data          #received raw data
string picker_fault_code  #picker fault code
string picker_state       #picker state
float32 rt_pos            #realtime pos
float32 rt_force          #realtime force
float32 rt_vel            #realtime vel
```
### Service: omnipicker_control
Service Type: omnipicker_interfaces/srv/OmniPickerControl

```yaml
#Request
string mode         # 1. auto: control the omnipicker to open or clamp. 
                    # 2. manual：manually set the omnipicker position, torque, speed, acceleration, and deceleration 
string cmd          # vaild when mode = auto
                    # 1. open: control the omnipicker to open
                    # 2. clamp: control the omnipicker to clamp
float32 pos         # value * 0xFF, value range: [0,1] (vaild when mode = manual)
float32 force       # value * 0xFF, value range: [0,1] (vaild when mode = manual)
float32 vel         # value * 0xFF, value range: [0,1] (vaild when mode = manual)
float32 acc         # value * 0xFF, value range: [0,1] (vaild when mode = manual)
float32 dec         # value * 0xFF, value range: [0,1] (vaild when mode = manual)
---
#Response
bool result         # execution result
string result_code  # execution result code
```

## Usage Examples

```bash
# Change device permissions
sudo chmod 777 /dev/ttyACM*
# Run
ros2 run omnipicker_driver omnipicker_driver --ros-args -p omnipicker_port:=/dev/ttyACM*

# Monitor current state
ros2 topic echo /omnipicker_state

# Control omnipicker
# auto-open
ros2 service call /omnipicker_control omnipicker_interfaces/srv/OmniPickerControl \
'{mode: "auto", cmd: "open", pos: 0.0, force: 0.0, vel: 0.0, acc: 0.0, dec: 0.0}'

# auto-clamp
ros2 service call /omnipicker_control omnipicker_interfaces/srv/OmniPickerControl \
'{mode: "auto", cmd: "clamp", pos: 0.0, force: 0.0, vel: 0.0, acc: 0.0, dec: 0.0}'

# manual 0.5
ros2 service call /omnipicker_control omnipicker_interfaces/srv/OmniPickerControl \
'{mode: "manual", cmd: "", pos: 0.5, force: 0.5, vel: 0.5, acc: 0.5, dec: 0.5}'
```
