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

    // Publish message
    pose_pub_->publish(msg);
}

} // uwb_localization