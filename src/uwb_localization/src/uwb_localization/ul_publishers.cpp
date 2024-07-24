/**
 * UWB localization publishers routine.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 27, 2023
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
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