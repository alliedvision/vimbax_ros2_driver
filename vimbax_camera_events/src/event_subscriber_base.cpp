#include <vimbax_camera_events/event_subscriber_base.hpp>

namespace vimbax_camera_events
{
EventSubscriberBase::EventSubscriberBase(rclcpp::Node::SharedPtr node, const std::string & topic)
  : std::enable_shared_from_this<EventSubscriberBase>(), base_topic_{topic}, node_{node}
{
  subscribe_service_client_ = node_->create_client<vimbax_camera_msgs::srv::SubscribeEvent>(
    subscribe_topic_name(base_topic_)
  );

  unsubscribe_service_client_ = node_->create_client<vimbax_camera_msgs::srv::UnsubscribeEvent>(
    unsubscribe_topic_name(base_topic_)
  );

  using namespace std::chrono_literals;
  subscribe_service_client_->wait_for_service(10s);
  unsubscribe_service_client_->wait_for_service(10s);
}
}