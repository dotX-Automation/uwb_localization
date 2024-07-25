/**
 * UWB localization subscriptions ruotine.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 *
 * July 24, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <uwb_localization/uwb_localization.hpp>

namespace uwb_localization
{

/**
 * @brief Receives message from Uwb1.
 *
 * @param msg UWBTag message to parse.
 */
void UWBLocalizationNode::uwb_clbk(const UWBTag::SharedPtr msg)
{
  rclcpp::Time stamp = this->get_clock()->now();

  TransformStamped tf_global_base;
  TransformStamped tf_base_uwb;
  try {
    tf_global_base = tf_buffer_->lookupTransform(
      global_link_,
      base_link_,
      rclcpp::Time());
    tf_base_uwb = tf_buffer_->lookupTransform(
      base_link_,
      msg->header.frame_id,
      rclcpp::Time());
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "TF exception: %s", e.what());
    return;
  }

  tf2::Vector3 traslation = tf2::quatRotate(
    tf2::Quaternion(
      tf_global_base.transform.rotation.x,
      tf_global_base.transform.rotation.y,
      tf_global_base.transform.rotation.z,
      tf_global_base.transform.rotation.w),
    tf2::Vector3(
      tf_base_uwb.transform.translation.x,
      tf_base_uwb.transform.translation.y,
      tf_base_uwb.transform.translation.z));

  unsigned int count;
  std::vector<double> distances;
  std::vector<double> anchors_x;
  std::vector<double> anchors_y;
  std::vector<double> anchors_z;

  count = msg->n_anchors;
  distances.reserve(count);
  anchors_x.reserve(count);
  anchors_y.reserve(count);
  anchors_z.reserve(count); 
  for (unsigned int i = 0; i < count; i++) {
    distances[i] = msg->anchors[i].distance;
    anchors_x[i] = msg->anchors[i].position.x;
    anchors_y[i] = msg->anchors[i].position.y;
    anchors_z[i] = msg->anchors[i].position.z;
  }

  std::array<double, 3> pos0;
  pos0[0] = pos_[0] + traslation.x();
  pos0[1] = pos_[1] + traslation.y();
  pos0[2] = pos_[2] + traslation.z();

  Function* function = new Function(
    two_d_mode_,
    squared_cost_,
    count,
    distances,
    anchors_x,
    anchors_y,
    anchors_z);
  Result result = solve(function, pos0);

  pos_[0] = result.position[0] - traslation.x();
  pos_[1] = result.position[1] - traslation.y();
  pos_[2] = result.position[2] - traslation.z();

  publish_pose(stamp);

  if(verbose_) {
    RCLCPP_INFO(this->get_logger(), "Estimation: %0.3f %0.3f %0.3f", pos_[0], pos_[1], pos_[2]);
    std::cout << result.summary.BriefReport() << std::endl;
  }
}


} // uwb_localization
