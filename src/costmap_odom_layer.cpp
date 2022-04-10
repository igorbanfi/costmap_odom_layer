/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Igor Banfi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "costmap_odom_layer/costmap_odom_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace costmap_odom_layer
{

CostmapOdomLayer::~CostmapOdomLayer()
{
}

void CostmapOdomLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "observation_sources", topics_string);

  RCLCPP_INFO(node->get_logger(), "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  CostmapOdomLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double obstacle_range;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
    declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("Odometry")));
    declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.5));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "obstacle_range", obstacle_range);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);

    if (!(data_type == "Odometry")) {
      RCLCPP_FATAL(
        node->get_logger(),
        "Only topics that use Odometry are currently supported");
      throw std::runtime_error(
              "Only topics that use Odometry are currently supported");
    }

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // add topic name to vector with topics
    if (data_type == "Odometry") {
      auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
      odom_topics_.push_back( topic );
      odom_custom_qos_.push_back( custom_qos_profile );
      last_odometries_.push_back( odometry );
      std::function<void(const nav_msgs::msg::Odometry::SharedPtr message)> bound_odom_callback_func = 
      std::bind(&CostmapOdomLayer::odomCallback, std::placeholders::_1, odometry);
      auto sub = node->create_subscription<nav_msgs::msg::Odometry>( topic, 20, bound_odom_callback_func
      );
      odom_subscribers_.push_back(sub);

      RCLCPP_INFO(node->get_logger(), "Source : %s", topic.c_str());

      RCLCPP_INFO(
      logger_, "RangeSensorLayer: subscribed to "
      "topic %s", odom_subscribers_.back()->get_topic_name());

    }

    // if (sensor_frame != "") {
    //   std::vector<std::string> target_frames;
    //   target_frames.push_back(global_frame_);
    //   target_frames.push_back(sensor_frame);
    //   observation_notifiers_.back()->setTargetFrames(target_frames);
    // }
  }
}

void
CostmapOdomLayer::odomCallback(
  nav_msgs::msg::Odometry::ConstSharedPtr message,
  std::shared_ptr<nav_msgs::msg::Odometry> odometry)
{
  RCLCPP_INFO(rclcpp::get_logger("odom_callback"), "Odometry callback");

  *odometry = *message;
}

void
CostmapOdomLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  if (!enabled_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;

  // update the global current status
  current_ = current;

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
CostmapOdomLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
CostmapOdomLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{

  if (!enabled_) {
    return;
  }

  for (const auto & odom : last_odometries_) {

      unsigned int mx;
      unsigned int my;

      if(worldToMap(odom->pose.pose.position.x, odom->pose.pose.position.y, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        }
    }
}

void
CostmapOdomLayer::activate()
{
  
  // auto node = node_.lock();
  // if (!node) {
  //   throw std::runtime_error{"Failed to lock node"};
  // }
  // printf( "Subscribing ");
  // for (int i; i < odom_topics_.size(); i++) {
  //   printf( "%s ", odom_topics_[i].c_str());
  //   std::function<void(const nav_msgs::msg::Odometry::SharedPtr message)> bound_odom_callback_func = 
  //     std::bind(&CostmapOdomLayer::odomCallback, std::placeholders::_1, last_odometries_[i]);
  //   auto sub = node->create_subscription<nav_msgs::msg::Odometry>( odom_topics_.at(i), 20, bound_odom_callback_func
  //   );
  //   odom_subscribers_.push_back(sub);
  // }
}

void
CostmapOdomLayer::deactivate()
{
  // odom_subscribers_.clear();
}

void
CostmapOdomLayer::reset()
{
  resetMaps();
  current_ = true;
}

}  // namespace costmap_odom_layer

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_odom_layer::CostmapOdomLayer, nav2_costmap_2d::Layer)