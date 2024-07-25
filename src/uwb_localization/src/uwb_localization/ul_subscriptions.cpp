/**
 * UWB localization subscriptions ruotine.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 *
 * July 24, 2024
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
 * @brief Receives message from Uwb1.
 *
 * @param msg UWBTag message to parse.
 */
void UWBLocalizationNode::uwb_clbk(const UWBTag::SharedPtr msg)
{
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
}


} // uwb_localization
