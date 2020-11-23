Rover 1
=======

Username: `rosuser`

Password: `rospassword`

WiFi
----
SSID: `rover_wireless`

Password: `rover_password`

IP address: `10.42.0.1`

_If the connection fails, try again. You may try many times to succeed._

Cable
-----
Ip addess: `192.168.1.2`


Rover 3
======

Username: `rosuser`

Password: `rospassword`

WiFi
-------------
SSID: `rover3_network`

Password: `Doesroundorsquarewheelsworkbest?`

IP address: `10.10.0.1`


Preinstalled motor driver
=========================

Preinstalled motor driver is in the workspace `unik4490_ws`. The motor driver contains a PID-controller that control the Rover speed and steering.

**The motordriver shall only be installed on the Rovers, and shall not be shared!**

Test the motor driver
---------------------

The motor driver is in the ROS package `ros_motordriver`. It contains the executables `driver.py` and `driver_pid.py`. `driver.py` has no feedback from the wheels, and `driver_pid.py` controls the wheel speed with a PID controller.

First we need to source all the executables

```
$ cd ~/unik4490_ws
$ source setup/devel.bash
```

Then we can launch the `driver_pid.py`
```
$ rosrun rover_motordriver driver_pid.py
```

The Rover can be controlled by the topic `/cmd_vel`.
```
$ rostopic pub /cmd_vel rover_motordriver/Velocity
```
_Use `Tab` to autocomplete._

The message will then look like this
```
rostopic pub /cmd_vel rover_motordriver/Velocity "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
left_vel: 0.0
right_vel: 0.0
duration: 0.0"
```

Try to set `left_vel` and `right_vel` to 2.0, and `duration` to 10.0. The Rover will now drive strait ahead with 2 m/s for 10 seconds.


