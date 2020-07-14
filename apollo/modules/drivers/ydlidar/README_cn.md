## YDLidar_Apollo_Driver
YDLidar_Apollo_Driver 可以将 YDLIDAR LiDAR 原始数据转换为LidarScan和标准点云数据格式,并输出到Scan以及点云 channels.

### LidarScan 输出 channels

每个 LiDAR 都对应一个lidar scan通道.
 ```
channel: /apollo/sensor/ydlidar/front/center/LidarScan
type: apollo::drivers::ydlidar::LidarScan
proto: [modules/drivers/ydlidar/proto/ydlidar.proto]https://github.com/YDLIDAR/ydlidar_apollo_driver/blob/master/modules/drivers/ydlidar/proto/ydlidar.proto
```

### 点云输出 channels

每个 LiDAR 都对应一个点云通道.
 ```
channel: /apollo/sensor/ydlidar/front/center/PointCloud
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

```

### 配置文件

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

* `scan_channel` 表示 LiDAR 的输出Scan通道.
* `pc_channel` 表示 LiDAR 的输出点云通道. 
* `port` 表示 LiDAR 的端口或者IP地址．
* `ignore_array` 表示 LiDAR 的剔除角度范围，角度之间以逗号分隔．
* `baudrate` 表示 LiDAR 的波特率或者网络端口．
* `lidar_type` 表示 LiDAR 的类型，可配置为：(1) TYPE_TOF(0)．(2) TYPE_TRIANGLE(1). (3) TYPE_TOF_NET(2).
* `device_type` 表示 LiDAR 的设备连接方式, 可配置为: (1) YDLIDAR_TYPE_SERIAL(0). (2) YDLIDAR_TYPE_TCP(1)．
* `sample_rate` 表示 LiDAR 采样频率．
* `abnormal_check_count` 表示 LiDAR 连接时，最大异常尝试次数．
* `resolution_fixed` 表示 LiDAR 是否是固定角度分辨率输出．
* `reversion` 表示 LiDAR 零位是否旋转180度．
* `inverted` 表示 LiDAR 数据是否转换到逆时针方向．
* `auto_reconnect` 表示 LiDAR 异常是否自动重新连接．
* `isSingleChannel` 表示 LiDAR　是否是单通道雷达．
* `intensity` 表示 LiDAR 是否支持光强信息．
* `support_motor_dtr` 表示 LiDAR 电机是否支持DTR控制．
* `max_angle` 表示 LiDAR 最大有效角度．
* `min_angle` 表示 LiDAR 最小有效角度．
* `max_range` 表示 LiDAR 最大有效距离．
* `min_range` 表示 LiDAR 最小有效距离．
* `frequency` 表示 LiDAR 扫描频率．

**注意**

    1. 具体雷达参数，请参照雷达数据表填写.
    2. 光强信息需雷达支持，才能设置到true.


### 如何加入 Apollo Project

YDLidar_Apollo_Driver 和 Apollo5.0 的目录一致

1. 将 ydlidar_apollo_driver 代码拷贝到 apollo 工程的同级目录中:
  ```bash
  cp -r ydlidar_apollo_driver/apollo/modules/drivers/ydlidar  your_apollo_project/apollo/modules/drivers/
  cp ydlidar_apollo_driver/apollo/docker/build/installers/install_ydlidar_sdk.sh  your_apollo_project/apollo/docker/build/installers/
  ```
2. 启动和进入 apollo docker:
  ```bash
  bash /apollo/docker/scripts/dev_start.sh
  bash /apollo/docker/scripts/dev_into.sh
  ```
3. 安装 ydlidar_sdk 动态链接库:
  ```bash
  sudo bash /apollo/docker/build/installers/install_ydlidar_sdk.sh
  ```
4. 构建项目:
  ```bash
  bash apollo.sh build
  ```

### 启动 YDLidar_Apollo_Driver

```bash
#in docker
cd /apollo && cyber_launch start modules/drivers/ydlidar/launch/ydlidar.launch
```

### YDLidar_Apollo_Driver 测试

```bash
#in docker
cd /apollo && ./bazel-bin/modules/drivers/ydlidar/driver/driver_test
```

