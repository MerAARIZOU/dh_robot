// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node {
public:
  FixedFrameBroadcaster() : Node("tf2_broadcaster") {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback() {
    geometry_msgs::msg::TransformStamped t1, t2;
    tf2::Quaternion q1, q2;

    q1.setRPY(1.57, 0, 1.57);

    t1.transform.rotation.x = q1.x();
    t1.transform.rotation.y = q1.y();
    t1.transform.rotation.z = q1.z();
    t1.transform.rotation.w = q1.w();
    t1.transform.translation.x = 0.0;
    t1.transform.translation.y = 1.0;
    t1.transform.translation.z = 0.0;

    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = "base";
    t1.child_frame_id = "frame1";
    tf_broadcaster_->sendTransform(t1);

    q2.setRPY(0.0, 3.14, 0.0);

    t2.transform.rotation.x = q2.x();
    t2.transform.rotation.y = q2.y();
    t2.transform.rotation.z = q2.z();
    t2.transform.rotation.w = q2.w();
    t2.transform.translation.x = 1.0;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 2.5;

    t2.header.stamp = this->get_clock()->now();
    t2.header.frame_id = "base";
    t2.child_frame_id = "frame2";
    tf_broadcaster_->sendTransform(t2);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}