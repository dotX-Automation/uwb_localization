/**
 * UWB localization publishers routine.
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
 * @brief Publish estimated pose.
 *
 * @param stamp Message timestamp.
 */
void UWBLocalizationNode::publish_pose(const rclcpp::Time &stamp){
  PoseWithCovarianceStamped msg;

  msg.header.frame_id = global_link_;
  msg.header.stamp = stamp;
  msg.pose.pose.position.x = pos_[0];
  msg.pose.pose.position.y = pos_[1];
  msg.pose.pose.position.z = pos_[2];
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.covariance = cov_;

  pose_pub_->publish(msg);
}

/**
 * @brief Publish estimated pose.
 *
 * @param stamp Message timestamp.
 * @param fun Optimization problem.
 * @param res Optimization result.
 */
void UWBLocalizationNode::publish_visual(const rclcpp::Time &stamp, const Function &fun, const Result &res){
  MarkerArray msg;
  msg.markers.reserve(fun.measures.size() + 1);

  Marker marker_del;
  marker_del.header.frame_id = global_link_;
  marker_del.header.stamp = stamp;
  marker_del.action = Marker::DELETEALL;
  msg.markers.push_back(marker_del);

  Marker marker_pos;
  marker_pos.header.frame_id = global_link_;
  marker_pos.header.stamp = stamp;
  marker_pos.action = Marker::ADD;
  marker_pos.type = Marker::SPHERE;
  marker_pos.ns = "estimation";
  marker_pos.id = 0;
  marker_pos.pose.position.x = res.position.x();
  marker_pos.pose.position.y = res.position.y();
  marker_pos.pose.position.z = res.position.z();
  marker_pos.pose.orientation.x = 0.0;
  marker_pos.pose.orientation.y = 0.0;
  marker_pos.pose.orientation.z = 0.0;
  marker_pos.pose.orientation.w = 1.0;
  marker_pos.scale.x = pos_covariance_[0];
  marker_pos.scale.y = pos_covariance_[1];
  marker_pos.scale.z = pos_covariance_[2];
  marker_pos.color.r = 1.0;
  marker_pos.color.g = 0.0;
  marker_pos.color.b = 1.0;
  marker_pos.color.a = 1.0;
  msg.markers.push_back(marker_pos);

  for(size_t i = 0; i < fun.measures.size(); i++) {
    Marker marker = Marker();
    marker.header.frame_id = global_link_;
    marker.header.stamp = stamp;
    marker.action = Marker::ADD;
    marker.type = Marker::SPHERE;
    marker.ns = "estimation";
    marker.id = i+1;
    marker.pose.position.x = fun.measures[i].anchor.x();
    marker.pose.position.y = fun.measures[i].anchor.y();
    marker.pose.position.z = fun.measures[i].anchor.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = fun.measures[i].distance;
    marker.scale.y = fun.measures[i].distance;
    marker.scale.z = fun.measures[i].distance;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.2;
    msg.markers.push_back(marker);
  }

  visual_pub_->publish(msg);
}

} // uwb_localization