Installation
============

Follow the installation guide in INF3480:
https://github.uio.no/INF3480/crustcrawler_simulation/wiki/Setup

In addition, clone the following repos into the same workspace
```
$ git clone https://github.uio.no/INF3480/crustcrawler_hardware.git
$ git clone https://github.uio.no/UNIK4490/DynamixelSDK.git
```

The software needs also access to the USB. Type
```
$ sudo usermod -a -G dialout $USER
```


Try the CrustCrawler with the default trajectory controller
===========================================================

Start the hardware driver with
```
$ roslaunch crustcrawler_hardware control.launch  control:=trajectory full_arm:=true 
```
Where the `control` field opens different controllers. This field can be `position` (default), `velocity`, `effort` or `trajectory`. `full_arm` must be `true`!

The CrustCrawler is default disabled. To enable the CrustCrawler, publish a message that contains `true` to the topic `/crustcrawler/enable`
```
$ rostopic pub /crustcrawler/enable std_msgs/Bool "data: true"
```

And to disable the CrustCrawler, publish a message that contains `false` to the topic `/crustcrawler/enable`
```
$ rostopic pub /crustcrawler/enable std_msgs/Bool "data: false"
```

The trajectory controller has already a rqt plugin for controlling a robot arm. Open the rqt plugin Joint trajectory controller.

Measurements from the CrustCrawler is published on the topic `/crustcrawler/joint_states`
 

Dynamic model
=============
https://github.uio.no/UNIK4490/crustcrawler_setup/wiki/CrustCrawler-Parameters
