# Swift Navigation PGM-EVK ROS Node
**InDro Robotics**  
Created by [Austin Greisman](mailto:austin.greisman@indrorobotics.com)  
Date Created: Oct 20th, 2021  
Date Updated: Nov 5rd, 2021  

> Quick start guide can be found in `docs/`
## General Info
- Ethernet IP: `192.168.42.152`
- Ethernet Port: `55556`
- Ethernet Port `55555` is currently reserved for the `swiftpgm/heading` topic.

## ToDo
1. Change protical to UDP (Found some UDP code on swift-nav git)
2. Check out `docs/SBP.pdf` page 4 for more message type options

## Published Nodes
| Topic     | Message Type                 | Units                  | Description                                                                                                                       |
|-----------|------------------------------|------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `swiftpgm/imu/raw` | `sensor_msgs/IMU`            | Accel = G Gyro = deg/s | Unfiltered IMU output                                                                                                             |
| `swiftpgm/imu/abs` | `std_msgs/Float32MultiArray` | deg                    | IMU output with fusion applied to gather Roll, Pitch, Yaw                                                                         |
| `swiftpgm/gps`     | `sensor_msgs/NavSatFix`      | deg                    | Latitude and Longitude GPS Fix - Will only display once Swift Nav has GPS lock                                                    |
| ~~`swiftpgm/temp`~~    | ~~`sensor_msgs/Temperature`~~    | ~~°C~~                     | ~~Temperature from the IMU sensors in the Swift Nav~~                                                                                 |
| `swiftpgm/heading` | `std_msgs/Float32`           | deg                    | Once the Swift Nav begins to move, the internal RTK will start up and begin outputting Heading information. 0° towards true North |
| `swiftpgm/socket_dump` | `swift_pgm/SwiftNavOutput`           | heading, GPS, IMU, ~~IMU_conf~~                    | This will output all data for the GPS and IMU nodes. Below describes its configuration |

### SwiftNavOutput Custom Message
| Type                             | Name               | Description                                                    |
|----------------------------------|--------------------|----------------------------------------------------------------|
| `std_msg/Header`                 | `header`           | Normal header for message                                      |
| `std_msgs/Float32MultiArray`     | `gps_raw`          | {tow, lat, lon, height, h_accuracy, v_accuracy, n_sats, flags} |
| `std_msgs/Float32MultiArray`     | `IMU_raw`          | {tow, tow_f, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z}         |
| ~~`std_msgs/Float32MultiArray`~~ | ~~`IMU_conf_raw`~~ | ~~{imu_type, temp, imu_conf}~~                                 |

## Getting your system ready
1. Follow the [Quick Start Guide](docs/PGM_EV_ Quick_Start_Guide_GD-110014-01.pdf) to set up the PGM  
2. Clone the following repo **outside of your catkin directory**  
 `git clone https://github.com/swift-nav/libsbp.git`   
3. Then run `git submodule update --init --recursive` right after.  
4. Then run the following commands to build the system.
```bash
    cd libsbp/c
    mkdir build
    cd build
    cmake ../
    make
    sudo make install
```
5. Make sure to `catkin_make` from your workspace before running as there is a custom message in this package.

## Running Code
1. The system can be started by running `roslaunch swift_pgm swift_nav_ROS.launch`
2. If you would like to run individual nodes, make sure to always start `pgm_node.py` as it runs the `swiftpgm/socket_dump` topic.
