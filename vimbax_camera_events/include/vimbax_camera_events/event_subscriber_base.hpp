#ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_

#include <memory>
#include <future>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/vimbax_camera_events.hpp>

#include <vimbax_camera_msgs/srv/subscribe_event.hpp>
#include <vimbax_camera_msgs/srv/unsubscribe_event.hpp>


namespace vimbax_camera_events
{

class EventSubscriberBase : public std::enable_shared_from_this<EventSubscriberBase>
{
public:
  

  template<typename T> 
  class EventSubscribtion
  {
  public:
    ~EventSubscribtion() {
      subscribtion_.reset();

      auto request = std::make_shared<vimbax_camera_msgs::srv::UnsubscribeEvent::Request>();
      request->name = event_name_;

      event_subscriber_->unsubscribe_service_client_->async_send_request(request);
    }

    bool is_connected() const {
      return connected_;
    }

  private:
    friend class EventSubscriberBase;

    typename rclcpp::Subscription<T>::SharedPtr subscribtion_{};
    std::shared_ptr<EventSubscriberBase> event_subscriber_;
    std::atomic_bool connected_{false};
    std::string event_name_;
  };

protected:
  EventSubscriberBase(rclcpp::Node::SharedPtr node, const std::string & topic);

  template<typename T>
  std::shared_ptr<EventSubscribtion<T>> subscribe_event(
    const std::string & event, std::function<void (const T &)> callback)
  {
    auto subscription = std::make_shared<EventSubscribtion<T>>();
    subscription->event_name_ = event;
    subscription->event_subscriber_ = shared_from_this();

    auto request = std::make_shared<vimbax_camera_msgs::srv::SubscribeEvent::Request>();
    request->name = event;

    using ResonseFuture = rclcpp::Client<vimbax_camera_msgs::srv::SubscribeEvent>::SharedFuture;

    subscribe_service_client_->async_send_request(request, 
      [this, event, subscription, callback=std::move(callback)] 
      (ResonseFuture future) {
        if (future.get()->error == 0) {
          subscription->subscribtion_ = node_->create_subscription<T>(
            event_topic_name(base_topic_, event), 10, 
            [callback=std::move(callback)] (std::shared_ptr<T> msg) {
              callback(*msg);
            });  
        }
        
    });

    return subscription;
  }
  
private:

  std::string base_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<vimbax_camera_msgs::srv::SubscribeEvent>::SharedPtr subscribe_service_client_;
  rclcpp::Client<vimbax_camera_msgs::srv::UnsubscribeEvent>::SharedPtr unsubscribe_service_client_;
};

}

#endif  // #ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_