/**
 * UWB localization node definition.
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
