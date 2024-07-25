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

#ifndef UWB_LOCALIZATION_HPP
#define UWB_LOCALIZATION_HPP

#include <bitset>

#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <dua_interfaces/msg/uwb_anchor.hpp>
#include <dua_interfaces/msg/uwb_tag.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <uwb_ceres/uwb_ceres.hpp>

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace uwb_ceres;

namespace uwb_localization
{

class UWBLocalizationNode : public dua_node::NodeBase
{
public:
    UWBLocalizationNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());
    ~UWBLocalizationNode();
    
private:
    /* Node parameters */
    std::string base_link_;
    std::string global_link_;
    std::vector<double> initial_pos_;
    std::vector<double> pos_covariance_;
    bool squared_cost_;
    bool two_d_mode_;
    bool use_tag_estimate_;
    bool verbose_;

    /* Node initialization routines */
    void init_cgroups();
    void init_parameters();
    void init_publishers();
    void init_structures();
    void init_subscriptions();
    void init_tf_listeners();

    /* Internal variables */
    std::array<double, 3> pos_;
    std::array<double, 36> cov_;

    /* tf2 variables */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    TransformStamped tf_base_uwb_;

    /* Subscribers */
    rclcpp::Subscription<UWBTag>::SharedPtr uwb_sub_;

    /* Subscribers Topics */
    static const std::string uwb_sub_topic_;

    /* Subscribers Callback Groups */
    rclcpp::CallbackGroup::SharedPtr uwb_cgroup_;

    /* Subscribers Callbacks */
    void uwb_clbk(const UWBTag::SharedPtr msg);

    /* Publishers */
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    /* Publishers Topics */
    static const std::string pose_pub_topic_;

    /* Publishers routines */
    void publish_pose(const rclcpp::Time &stamp);
};


} // namespace uwb_localization

#endif // UWB_LOCALIZATION_HPP
