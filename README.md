# tellos_ffc
A ROS package for formation flight using [Tello EDUs](https://www.ryzerobotics.com/jp/tello-edu). It should be noted that **multiple [Tellos](https://www.ryzerobotics.com/jp/tello) cannot be controlled using this software**. Tello EDUs can only be used. In below, Tello EDU is referred to Tello.



# How to install

I confirmed that tellos_ffc works on Ubuntu 20.04 with ROS noetic.

```
$ cd [your_ros_workspace]/src
$ git clone https://github.com/NaokiAkai/tellos_ffc.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```



# How to use

## Parameter setting

There is a parameter file in the sh directory named **param**. Parameters in the file must be appropriately set.

- tello_num: Number of Tellos
- initial_x, initial_y, initial_yaw: Initial pose parameters for simulation
- mocap_server: IP address of the motion capture server
- tello_ip: IP addresses of Tellos



## Simulation

tellos_ffc has a simple simulation node. The simple simulation can be used as follow.

```
$ cd tellos_ffc/sh
$ ./multiple_tellos_sim.sh
```

Then, the formation flight controller can be simulated.

```
$ ./formation_flight.sh
```

The simulation can be visualized using rviz.

```
$ cd tellos_ffc/rviz
$ rviz -d sim.rviz
```



## Real equipments

### Setting for Tello EDUs

Before testing the formation flight, Tello EDUs must be appropriately set.



- Wi-Fi connection

In default, Tellos have a function as an access point and PCs can be connected to them. However in the default setting, multiple Tellos cannot be controlled using a PC. To control multiple Tellos, Tellos and PC must be connected to one access point. Tello's Wi-Fi connection can be changed using the python script of the tello_navi package.

```
$ cd tellos_ffc/tello_navi/scripts
$ nano tello_wifi_access.py
```

Edit tello_wifi_access.py based on your environment.

```python
ssid = 'Your_SSID'
password = 'Your_SSID_Password'
```

Then, turn on Tello and connect to it with PC (Tello's SSID can be seen from PC). After connected, run tello_wifi_access.py.

```
$ python3 tello_wifi_access.py
```

After changing, Tello must be turn off. From the next, Tello connects to a target access point.



- IP address

IP address of Tellos must be fixed. To fix the address, setting of the access point is needed to be changed. It should be noted that there are some access points that cannot be changed their settings.

After fixing IP addresses for Tellos, these addresses must be written in the parameter file locating in the sh directory.

```
$ cd tellos_ffc/sh
$ nano params
```

In default, IP addresses are set as follow

```shell
tello_ip=(192.168.11.100 192.168.11.101 192.168.11.102 192.168.11.103 192.168.11.104 192.168.11.105)
```

In this setting, 6 Tellos are used and these IP addresses are set to from 192.168.11.100 to 192.168.11.105. Note that the number of IP address must be equal to tello_num.



## Communication with Tellos

To communicate with Tellos, we used [tello_dirver](http://). Note that we simply modified the driver to use on Ubuntu 20.04 with ROS noetic.

```
$ cd tellos_ffc/sh
$ ./tello_drivers.sh
```

If cannot be communicated, tello_ip written in params must be checked.



## Motion capture

We used the motion capture system produced by Acuity Inc.

First, please check IP address of the server that the motion capture works and set it to mocap_server written in param. Then, run the motion capture script.

```
$ cd tellos_ffc/sh
$ ./motion_capture.sh
```



## Takeoff and land

Takeoff and land commands are prepared in the sh directory.

```
$ ./takeoff.sh # for takeoff
$ ./land.sh # for land
```



## Formation flight

Formation flight can be performed by executing two scripts.

```
$ ./formation_flight.sh
$ ./target_sender.sh
```

The formation flight node is executed by formation_flight.sh receives poses of Tellos and their targets for control. The node controls Tellos based on the poses. The control does not work if both poses are not received. The targets for control is published by the node executed by target_sender.sh.



## Visualization

The rviz file is prepared in the rviz directory.

```
$ cd tellos_ffc/rviz
$ rviz -d real.rviz
```

