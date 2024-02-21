
#ifndef VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_

#include <string>
#include <memory>
#include <functional>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_publisher_base.hpp>


namespace vimbax_camera_events 
{

template<typename T>
class EventPublisher : public EventPublisherBase
{
public:
  EventPublisher(rclcpp::Node::SharedPtr node,
    const std::string & topic_name, 
    OnEventSubscribed on_event_subscribed,
    OnEventUnsubscribed on_event_unsubscribed) 
    : EventPublisherBase(node, topic_name, on_event_subscribed, on_event_unsubscribed) 
  {

  }


  void publish_event(const std::string & event_name, const T & event)
  {
    auto publisher = 
      std::dynamic_pointer_cast<rclcpp::Publisher<T>>(get_event_publisher(event_name));

    if (publisher) {
      publisher->publish(event);
    }
  }

protected:
  rclcpp::PublisherBase::SharedPtr create_event_publisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos) override
  {
    return node->create_publisher<T>(topic_name, qos);
  }

private:
  
};

}

#endif  // #ifndef VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_