## YDLidar_Apollo_Driver
YDLidar_Apollo_Driver can convert YDLIDAR LiDAR's raw data to LidarScan  and standard format of point cloud, and write to point cloud channels.

### LidarScan output channels

Each LiDAR corresponds to a lidar scan channel.
 ```
channel: /apollo/sensor/ydlidar/front/center/LidarScan
type: apollo::drivers::ydlidar::LidarScan
proto: [modules/drivers/ydlidar/proto/ydlidar.proto]https://github.com/YDLIDAR/ydlidar_apollo_driver/blob/master/modules/drivers/ydlidar/proto/ydlidar.proto

```

### Point cloud output channels

Each LiDAR corresponds to a point cloud channel.
 ```
channel: /apollo/sensor/ydlidar/front/center/PointCloud
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

```

### Config file

```

frame_id: "ydlidar_horizon_front_center"
scan_channel: "/apollo/sensor/ydlidar/front/center/LidarScan"
pc_channel: "/apollo/sensor/ydlidar/front/center/PointCloud"
port: "/dev/ydlidar"
ignore_array: ""
baudrate: 512000
lidar_type: 0
device_type: 0
sample_rate: 20
abnormal_check_count: 4
resolution_fixed: false
reversion: true
inverted: true
auto_reconnect: true
isSingleChannel: false
intensity: false
support_motor_dtr: false
max_angle: 180.0
min_angle: -180.0
max_range: 64.0
min_range: 0.01
frequency: 10.0

```

* `scan_channel` represents LiDAR's lidar scan output channel.
* `pc_channel` represents LiDAR's point cloud output channel. 
* `port` represents LiDAR's serial port or network ip address．
* `ignore_array` represents the culling angle range of lidar, separated by commas．
* `baudrate` represents LiDAR's baudrate or network port．
* `lidar_type` represents LiDAR's Type, it can be configured to: (1) TYPE_TOF(0)．(2) TYPE_TRIANGLE(1). (3) TYPE_TOF_NET(2).
* `device_type` represents LiDAR's Device type, it can be configured to: (1) YDLIDAR_TYPE_SERIAL(0). (2) YDLIDAR_TYPE_TCP(1)．
* `sample_rate` represents LiDAR's sample rate
* `abnormal_check_count` represents the maximum number of exception attempts when lidar is connected．
* `resolution_fixed` represents whether lidar is a fixed angle resolution output．
* `reversion` represents whether to rotate 180 degrees．
* `inverted` represents wether to switch to the counter clockwise direction．
* `auto_reconnect` represents wether to automatically reconnect．
* `isSingleChannel` represents wether it is a single channel lidar.
* `intensity` represents wether to support intensity．
* `support_motor_dtr` represents wether to support DTR．
* `max_angle` represents LiDAR's maximum effective angle.
* `min_angle` represents LiDAR's minimum effective angle．
* `max_range` represents LiDAR's maximum effective range．
* `min_range` represents LiDAR's minimum effective range．
* `frequency` represents LiDAR's scan frequency

**Notice**

    1. Please refer to lidar data sheet for specific lidar parameters.

    2. The intensity information can only be set to true with the support of lidar.


### How to add to Apollo Project

YDLidar_Apollo_Driver has the same directory structure as Apollo5.0

1. copy ydlidar_apollo_driver to the same folder of your apollo project：
  ```bash
  cp -r ydlidar_apollo_driver/apollo/modules/drivers/ydlidar  your_apollo_project/apollo/modules/drivers/
  cp ydlidar_apollo_driver/apollo/docker/build/installers/install_ydlidar_sdk.sh  your_apollo_project/apollo/docker/build/installers/
  ```
2. start and enter apollo docker:
  ```bash
  bash /apollo/docker/scripts/dev_start.sh
  bash /apollo/docker/scripts/dev_into.sh
  ```
3. install ydlidar_sdk dynamic link library:
  ```bash
  sudo bash /apollo/docker/build/installers/install_ydlidar_sdk.sh
  ```
4. build project:
  ```bash
  bash apollo.sh build
  ```

### Start YDLidar_Apollo_Driver

```bash
#in docker
cd /apollo && cyber_launch start modules/drivers/ydlidar/launch/ydlidar.launch
```

### YDLidar_Apollo_Driver test

```bash
#in docker
cd /apollo && ./bazel-bin/modules/drivers/ydlidar/driver/driver_test
```

