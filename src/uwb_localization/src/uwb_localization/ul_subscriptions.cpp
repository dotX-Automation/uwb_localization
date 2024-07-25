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
  std::string uwb_link = msg->anchors[0].header.frame_id;
  std::string tag_link = msg->header.frame_id;

  TransformStamped tf_glob_uwb;
  TransformStamped tf_uwb_base;
  TransformStamped tf_base_tag;
  try {
    tf_glob_uwb = tf_buffer_->lookupTransform(
      global_link_,
      uwb_link,
      rclcpp::Time());
    tf_uwb_base = tf_buffer_->lookupTransform(
      uwb_link,
      base_link_,
      rclcpp::Time());
    tf_base_tag = tf_buffer_->lookupTransform(
      base_link_,
      tag_link,
      rclcpp::Time());
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "TF exception: %s", e.what());
    return;
  }

  Eigen::Quaterniond quat_glob_uwb(
    tf_glob_uwb.transform.rotation.w,
    tf_glob_uwb.transform.rotation.x,
    tf_glob_uwb.transform.rotation.y,
    tf_glob_uwb.transform.rotation.z);
  Eigen::Quaterniond quat_uwb_base(
    tf_uwb_base.transform.rotation.w,
    tf_uwb_base.transform.rotation.x,
    tf_uwb_base.transform.rotation.y,
    tf_uwb_base.transform.rotation.z);
  /*Eigen::Quaterniond quat_base_tag(
    tf_base_tag.transform.rotation.w,
    tf_base_tag.transform.rotation.x,
    tf_base_tag.transform.rotation.y,
    tf_base_tag.transform.rotation.z);*/

  Eigen::Vector3d vect_glob_uwb(
    tf_glob_uwb.transform.translation.x,
    tf_glob_uwb.transform.translation.y,
    tf_glob_uwb.transform.translation.z);
  /* Eigen::Vector3d vect_uwb_base(
    tf_uwb_base.transform.translation.x,
    tf_uwb_base.transform.translation.y,
    tf_uwb_base.transform.translation.z);*/
  Eigen::Vector3d vect_base_tag(
    tf_base_tag.transform.translation.x,
    tf_base_tag.transform.translation.y,
    tf_base_tag.transform.translation.z);

  Eigen::Vector3d pBase_wrt_glob;
  Eigen::Vector3d pBase_wrt_uwb;
  Eigen::Vector3d pTag_wrt_uwb;
  
  if(use_tag_estimate_) {
    pTag_wrt_uwb.x() = msg->tag_position.point.x;
    pTag_wrt_uwb.y() = msg->tag_position.point.y;
    pTag_wrt_uwb.z() = msg->tag_position.point.z;
  } else  {
    pBase_wrt_glob.x() = pos_[0];
    pBase_wrt_glob.y() = pos_[1];
    pBase_wrt_glob.z() = pos_[2];

    pBase_wrt_uwb = quat_glob_uwb.inverse() * (pBase_wrt_glob - vect_glob_uwb);
    pTag_wrt_uwb = pBase_wrt_uwb + quat_uwb_base * vect_base_tag;

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
    pos0[0] = pTag_wrt_uwb.x();
    pos0[1] = pTag_wrt_uwb.y();
    pos0[2] = pTag_wrt_uwb.z();

    Function* function = new Function(
      two_d_mode_,
      squared_cost_,
      count,
      distances,
      anchors_x,
      anchors_y,
      anchors_z);
    Result result = solve(function, pos0);

    pTag_wrt_uwb = Eigen::Vector3d(
      result.position[0],
      result.position[1],
      result.position[2]);

    if(verbose_) {
      RCLCPP_INFO(this->get_logger(), result.summary.BriefReport().c_str());
    }
  }

  pBase_wrt_uwb = pTag_wrt_uwb - quat_uwb_base * vect_base_tag;
  pBase_wrt_glob = vect_glob_uwb + quat_glob_uwb * pBase_wrt_uwb;

  pos_[0] = pBase_wrt_glob.x();
  pos_[1] = pBase_wrt_glob.y();
  pos_[2] = pBase_wrt_glob.z();

  publish_pose(stamp);

  if(verbose_) {
    RCLCPP_INFO(this->get_logger(), "Estimation: %0.3f %0.3f %0.3f", pos_[0], pos_[1], pos_[2]);
  }
}


} // uwb_localization
