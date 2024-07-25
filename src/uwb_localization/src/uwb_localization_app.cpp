/**
 * Uwb localization standalone application.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * October 10, 2022
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

#define MODULE_NAME "uwb_localization_app"

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <uwb_localization/uwb_localization.hpp>

using namespace uwb_localization;

int main(int argc, char **argv)
{
  // Disable I/O buffering
  if (setvbuf(stdout, NULL, _IONBF, 0))
  {
    RCLCPP_FATAL(
        rclcpp::get_logger(MODULE_NAME),
        "Failed to set I/O buffering");
    exit(EXIT_FAILURE);
  }

  // Create and initialize ROS 2 context
  rclcpp::init(argc, argv);

  // Initialize ROS 2 node
  auto sd_node = std::make_shared<UWBLocalizationNode>();

  // Create and configure executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(sd_node);

  RCLCPP_WARN(
      rclcpp::get_logger(MODULE_NAME),
      "(%d) " MODULE_NAME " online",
      getpid());

  // Spin the executor
  executor->spin();

  // Destroy ROS 2 node and context
  sd_node.reset();
  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
