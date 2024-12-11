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
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

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
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q3, q2, q1;
    tf2::Vector3 v3, v2, v1;
    tf2::Transform t3, t2, t1, t_final, t_inter;

    q3.setRPY(0.0, 0.0, 0.0);
    t3.setRotation(q3);
    v3 = tf2::Vector3(1.0, 0.0, 0.0);
    t3.setOrigin(v3);

    q2.setRPY(0.0, 3.14, 0.0);
    t2.setRotation(q2);
    v2 = tf2::Vector3(0.0, 0.0, 0.0);
    t2.setOrigin(v2);

    q1.setRPY(0.0, 0.0, 0.0);
    t1.setRotation(q1);
    v1 = tf2::Vector3(0.0, 0.0, 2.5);
    t1.setOrigin(v1);

    t_inter.mult(t3, t2);
    t_final.mult(t_inter, t1);

    t.transform.rotation.x = t_final.getRotation().getX();
    t.transform.rotation.y = t_final.getRotation().getY();
    t.transform.rotation.z = t_final.getRotation().getZ();
    t.transform.rotation.w = t_final.getRotation().getW();
    t.transform.translation.x = t_final.getOrigin().getX();
    t.transform.translation.y = t_final.getOrigin().getY();
    t.transform.translation.z = t_final.getOrigin().getZ();
    t.header.stamp = this->get_clock()->now();

    t.header.frame_id = "base";
    t.child_frame_id = "frame3";
    tf_broadcaster_->sendTransform(t);
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