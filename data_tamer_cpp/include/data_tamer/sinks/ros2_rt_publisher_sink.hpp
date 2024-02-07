#pragma once

#include "data_tamer/data_sink.hpp"
#include "data_tamer/details/mutex.hpp"
#include "data_tamer_msgs/msg/schemas.hpp"
#include "data_tamer_msgs/msg/snapshot.hpp"
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "realtime_tools/realtime_publisher.h"


namespace DataTamer
{

class ROS2RTPublisherSink : public DataSinkBase
{
public:
  ROS2RTPublisherSink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, 
    const std::string& topic_prefix);

  void addChannel(const std::string& name, const Schema& schema) override;

  bool storeSnapshot(const Snapshot& snapshot) override;

private:
//   std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::unordered_map<std::string, Schema> schemas_;
  Mutex schema_mutex_;

  rclcpp::Publisher<data_tamer_msgs::msg::Schemas>::SharedPtr schema_publisher_;
  rclcpp::Publisher<data_tamer_msgs::msg::Snapshot>::SharedPtr data_publisher_;

  using RTSchemaPublisher = realtime_tools::RealtimePublisher<data_tamer_msgs::msg::Schemas>;
  using RTSnapshotPublisher = realtime_tools::RealtimePublisher<data_tamer_msgs::msg::Snapshot>;

  std::unique_ptr<RTSchemaPublisher> schema_publisher_rt;
  std::unique_ptr<RTSnapshotPublisher> data_publisher_rt;


  bool schema_changed_ = true;
  data_tamer_msgs::msg::Snapshot data_msg_;
};

}   // namespace DataTamer
