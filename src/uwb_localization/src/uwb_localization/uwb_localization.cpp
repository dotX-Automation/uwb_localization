/**
 * UWB localization node definition.
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
 * @brief Builds a new UWBLocalizationNode.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError
 */
UWBLocalizationNode::UWBLocalizationNode(const rclcpp::NodeOptions &opts)
    : dua_node::NodeBase("uwb_localization", opts)
{
  // Initialize callback groups
  init_cgroups();

  // Initialize node parameters
  init_parameters();

  // Initialize structures
  init_structures();

  // Initialize TF listeners
  init_tf_listeners();

  // Initialize topic publishers
  init_publishers();

  // Initialize topic subscriptions
  init_subscriptions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Finalizes node operation.
 */
UWBLocalizationNode::~UWBLocalizationNode()
{
  RCLCPP_INFO(this->get_logger(), "Destructor called, sending stop signal.");
}

/**
 * @brief Routine to initialize callback groups.
 */
void UWBLocalizationNode::init_cgroups()
{
  uwb_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize vectors.
 */
void UWBLocalizationNode::init_structures()
{
  for(unsigned int i = 0; i < 3; i++){
      pos_[i] = initial_pos_[i];
  }

  for(unsigned int i = 0; i < 36; i++){
      cov_[i] = 0.0;
  }

  for(unsigned int i = 0; i < 3; i++){
      cov_[7*i] = pos_covariance_[i];
  }
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void UWBLocalizationNode::init_subscriptions()
{
  auto uwb_sub_opts = rclcpp::SubscriptionOptions();
  uwb_sub_opts.callback_group = uwb_cgroup_;
  uwb_sub_ = this->create_subscription<UWBTag>(
    uwb_sub_topic_,
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &UWBLocalizationNode::uwb_clbk,
      this,
      std::placeholders::_1),
    uwb_sub_opts);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void UWBLocalizationNode::init_publishers()
{
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    pose_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
}

/**
 * @brief Routine to initialize TF listeners.
 */
void UWBLocalizationNode::init_tf_listeners()
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


} // namespace uwb_localization
