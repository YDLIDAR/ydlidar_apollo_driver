/******************************************************************************
 * Copyright 2020 EAIBOT. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/ydlidar/driver/ydlidar_driver_component.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace ydlidar {

bool YDLidarDriverComponent::Init() {
  AINFO << "YDLidar driver component init";
  Config ydlidar_config;

  if (!GetProtoConfig(&ydlidar_config)) {
    return false;
  }

  AINFO << "ydlidar config: " << ydlidar_config.DebugString();

  if (!ydlidar_config.has_port()) {
    AERROR << "lidar port is empty!";
    return false;
  }

  auto writer = node_->CreateWriter<apollo::drivers::ydlidar::LidarScan>
                (ydlidar_config.scan_channel());


  if (!writer) {
    return false;
  }

  auto pc_writer = node_->CreateWriter<apollo::drivers::PointCloud>
                   (ydlidar_config.pc_channel());

  if (!pc_writer) {
    return false;
  }

  laser_scan_msg_ = std::make_shared<apollo::drivers::ydlidar::LidarScan>();

  if (!laser_scan_msg_) {
    return false;
  }

  laser_scan_msg_->mutable_header()->set_frame_id(ydlidar_config.frame_id());

  point_cloud_ = std::make_shared<apollo::drivers::PointCloud>();
  point_cloud_->Clear();

  if (!point_cloud_) {
    return false;
  }

  point_cloud_->mutable_header()->set_frame_id(ydlidar_config.frame_id());



  // start the writer
  writer_ = std::move(writer);
  pc_writer_ = std::move(pc_writer);


  auto driver = std::unique_ptr<CYdLidar>(new CYdLidar());

  if (!driver) {
    return false;
  }

// start the driver
  dvr_ = std::move(driver);

//////////////////////string property/////////////////
//port
  std::string str_optvalue = ydlidar_config.port();
  dvr_->setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
                    str_optvalue.size());

  //ignore array
  str_optvalue = ydlidar_config.ignore_array();
  dvr_->setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
                    str_optvalue.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = ydlidar_config.baudrate();
  dvr_->setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = ydlidar_config.lidar_type();
  dvr_->setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  /// device type
  optval = ydlidar_config.device_type();
  dvr_->setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  /// sample rate
  optval = ydlidar_config.sample_rate();
  dvr_->setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = ydlidar_config.abnormal_check_count();
  dvr_->setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));


  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = ydlidar_config.resolution_fixed();
  dvr_->setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = ydlidar_config.reversion();
  dvr_->setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = ydlidar_config.inverted();
  dvr_->setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = ydlidar_config.auto_reconnect();
  dvr_->setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = ydlidar_config.issinglechannel();
  dvr_->setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = ydlidar_config.intensity();
  dvr_->setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = ydlidar_config.support_motor_dtr();
  dvr_->setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = ydlidar_config.max_angle();
  dvr_->setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  laser_scan_msg_->mutable_config()->set_max_angle(f_optvalue);

  f_optvalue = ydlidar_config.min_angle();
  dvr_->setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  laser_scan_msg_->mutable_config()->set_min_angle(f_optvalue);


  /// unit: m
  f_optvalue = ydlidar_config.max_range();
  dvr_->setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  laser_scan_msg_->mutable_config()->set_max_range(f_optvalue);


  f_optvalue = ydlidar_config.min_range();
  dvr_->setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  laser_scan_msg_->mutable_config()->set_min_range(f_optvalue);


  /// unit: Hz
  f_optvalue = ydlidar_config.frequency();
  dvr_->setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  if (!dvr_->initialize()) {
    return false;
  }

  if (!dvr_->turnOn()) {
    return false;
  }

  // spawn device poll thread
  runing_ = true;
  device_thread_ = std::shared_ptr<std::thread>(
                     new std::thread(std::bind(&YDLidarDriverComponent::scan_data_poll, this)));
  device_thread_->detach();

  return true;
}

/** @brief Device poll thread main loop. */
void YDLidarDriverComponent::scan_data_poll() {
  while (!apollo::cyber::IsShutdown()) {
    // poll scan until end of one ring data
    LaserScan scan;
    bool ret = dvr_->doProcessSimple(scan);

    if (ret) {
      common::util::FillHeader(node_->Name(), laser_scan_msg_.get());
      laser_scan_msg_->mutable_header()->set_lidar_timestamp(scan.stamp);

      common::util::FillHeader(node_->Name(), point_cloud_.get());
      point_cloud_->mutable_header()->set_lidar_timestamp(scan.stamp);


      laser_scan_msg_->mutable_config()->set_angle_increment(
        scan.config.angle_increment);
      laser_scan_msg_->mutable_config()->set_scan_time(scan.config.scan_time);
      laser_scan_msg_->mutable_config()->set_time_increment(
        scan.config.time_increment);

      apollo::drivers::ydlidar::LaserPoint point;
      laser_scan_msg_->clear_points();
      point_cloud_->clear_point();

      for (size_t i = 0; i < scan.points.size(); i++) {
        point.set_angle(scan.points[i].angle);
        point.set_range(scan.points[i].range);
        point.set_intensity(scan.points[i].intensity);
        laser_scan_msg_->add_points()->CopyFrom(point);

        PointXYZIT *point_new = point_cloud_->add_point();
        point_new->set_x(scan.points[i].range * cos(scan.points[i].angle));
        point_new->set_y(scan.points[i].range * sin(scan.points[i].angle));
        point_new->set_z(0.0);
        point_new->set_intensity(scan.points[i].intensity);
        point_new->set_timestamp(static_cast<uint64_t>(scan.stamp +  i *
                                 scan.config.time_increment * 1e9));
      }

      writer_->Write(laser_scan_msg_);
      pc_writer_->Write(point_cloud_);
      AINFO << "publish scan msg[" << scan.stamp << "]: " <<
            laser_scan_msg_->points_size()
            << " in " << 1.0 / scan.config.scan_time << "Hz";
    } else {
      AWARN << "Lidar poll data failed";
    }
  }

  dvr_->turnOff();
  dvr_->disconnecting();
  AERROR << "CompYDDriver thread exit";
  runing_ = false;
}

}  // namespace ydlidar
}  // namespace drivers
}  // namespace apollo
