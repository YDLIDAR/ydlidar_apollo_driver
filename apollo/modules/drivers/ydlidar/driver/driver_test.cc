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
#include <src/CYdLidar.h>
#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/ydlidar/proto/config.pb.h"
#include "modules/drivers/ydlidar/proto/ydlidar.pb.h"

#include "gtest/gtest.h"

using apollo::cyber::base::CCObjectPool;

namespace apollo {
namespace drivers {
namespace ydlidar {

TEST(YDLidarDriverTest, General) {
  AINFO << "YDLidar driver component init";
  std::unique_ptr<CYdLidar> driver =  std::unique_ptr<CYdLidar>(new CYdLidar());
  EXPECT_NE(driver, nullptr);
  Config config;
  config.set_port("/dev/ydlidar");
  config.set_baudrate(512000);
  config.set_lidar_type(0);
  config.set_issinglechannel(false);
  std::string str_optvalue = config.port();
  EXPECT_TRUE(driver->setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
                                  str_optvalue.size()));

  int optval = config.baudrate();
  EXPECT_TRUE(driver->setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int)));
  /// tof lidar
  optval = config.lidar_type();
  EXPECT_TRUE(driver->setlidaropt(LidarPropLidarType, &optval, sizeof(int)));

  bool b_optvalue = config.issinglechannel();
  EXPECT_TRUE(driver->setlidaropt(LidarPropSingleChannel, &b_optvalue,
                                  sizeof(bool)));

  if (driver->initialize()) {
    EXPECT_TRUE(driver->turnOn());
    LaserScan scan;
    EXPECT_TRUE(driver->doProcessSimple(scan));
  }

  driver->turnOff();
  driver->disconnecting();
}

}  // namespace ydlidar
}  // namespace drivers
}  // namespace apollo

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
