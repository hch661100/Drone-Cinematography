## A modified [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS) ##

This is a modified version of [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS), which uses standard ros message types, provides limited function of [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS). In addition, this package supports the communicaiton between Android device and onboard computer. The example about mobile communication can be found in https://github.com/hch661100/Drone-Cinematography/blob/master/djiros-3.3/djiros-3.3/src/demo/mobile_comm.cpp


### ROS Interfaces ###

#### Parameters ####
```
~serial_name             [string] : Path to the serial port device (e.g. /dev/ttyUSB0)
~baud_rate               [int]    : Baudrate for serial port
~app_id                  [int]    : App Id for dji sdk
~enc_key                 [string] : App Key for dji sdk
~sensor_mode             [bool]   : No activation and control is needed, just output imu, rc, gps, ...
~align_with_fmu          [bool]   : Use ticks from FMU/ ros::Time::now() when data is received.
~gravity                 [double] : scale multiplied on accelerometer output
~ctrl_cmd_stream_timeout [double] : timeout for judging if control command is streaming in or stopped
~ctrl_cmd_wait_timeout   [double] : timeout for waiting for control command after switch into api mode
```

#### Topics ###

Publishers:

```
 ~imu  : [sensor_msgs/IMU]                               IMU message at 400 Hz, in the [ROS REP 103](http://www.ros.org/reps/rep-0103.html) frame definition.
 ~rc   : [sensor_msgs/Joy]                               RC joysticks at 50 Hz, remapped to [-1, +1]. See [include/djiros/DjiRos.h](include/djiros/DjiRos.h) for the direction.
 ~velo : [geometry_msgs/Vector3Stamped]                  Velocity message in NED frame.
 ~gps  : [sensor_msgs/NavSatFix]                         GPS message.
 ~gps_health : [std_msgs::UInt8]                         The strength of GPS Signal 0--->4 (bad --->good))
 ~from_mobile_data : [dji_sdk::MobileData]               Publish the data sent from Android device
 ~gimbal_angle : [geometry_msgs::Vector3Stamped]         The angle of gimbal camera
 
```

* See the code and [official documents](https://developer.dji.com/onboard-sdk/documentation/) for published topics and their details.
* For other messages, please modify the code.
* Pay attention to the frame_id of the published topics which represent the frame. Frame definition can be seen in [include/djiros/DjiRos.h](include/djiros/DjiRos.h).

Subscribers:
```
~ctrl: [sensor_msgs/Joy] For controlling the drone
~
```
Services:
```
~send_data_to_mobile : [dji_sdk::SendMobileData]          Send Data to Android device
```
<!-- #### TODO #### -->

