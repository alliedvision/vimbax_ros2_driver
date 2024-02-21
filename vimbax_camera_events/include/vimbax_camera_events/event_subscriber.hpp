#ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_

#include <memory>
#include <functional>
#include <future>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_subscriber_base.hpp>

namespace vimbax_camera_events
{

template<typename T>
class EventSubscriber : public EventSubscriberBase
{
public:

  template<typename... Args>
  static std::shared_ptr<EventSubscriber<T>> make_shared(Args... args)
  {
    return std::shared_ptr<EventSubscriber<T>>(new EventSubscriber<T>(args...));
  }

  std::shared_ptr<EventSubscribtion<T>> subscribe_event(
    const std::string & event, std::function<void (const T &)> callback)
  {
    return EventSubscriberBase::subscribe_event(event, callback);
  }
  
private:
  EventSubscriber(rclcpp::Node::SharedPtr node, const std::string & topic) 
  : EventSubscriberBase(node, topic) {

  }
};

}

#endif  // #ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_