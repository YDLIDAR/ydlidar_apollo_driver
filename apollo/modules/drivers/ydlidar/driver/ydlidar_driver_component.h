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

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"

#include <src/CYdLidar.h>
#include "modules/common/util/message_util.h"
#include "modules/drivers/ydlidar/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/ydlidar/proto/ydlidar.pb.h"

namespace apollo {
namespace drivers {
namespace ydlidar {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::PointCloud;

class YDLidarDriverComponent : public Component<> {
 public:
  ~YDLidarDriverComponent() {}
  bool Init() override;

 private:
  void scan_data_poll();
  volatile bool runing_;  ///< device thread is running
  std::shared_ptr<std::thread> device_thread_;
  std::shared_ptr<apollo::drivers::ydlidar::LidarScan> laser_scan_msg_ = nullptr;
  std::shared_ptr<PointCloud> point_cloud_ = nullptr;
  std::unique_ptr<CYdLidar> dvr_ = nullptr;  ///< driver implementation class
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::ydlidar::LidarScan>>
      writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> pc_writer_ =
        nullptr;
};

CYBER_REGISTER_COMPONENT(YDLidarDriverComponent)

}  // namespace ydlidar
}  // namespace drivers
}  // namespace apollo
