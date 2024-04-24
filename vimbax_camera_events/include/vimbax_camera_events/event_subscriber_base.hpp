// Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_

#include <memory>
#include <future>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/vimbax_camera_events.hpp>

#include <vimbax_camera_msgs/srv/subscribe_event.hpp>
#include <vimbax_camera_msgs/srv/unsubscribe_event.hpp>


namespace vimbax_camera_events
{

struct EventSubscribeException : public std::exception
{
  explicit EventSubscribeException(const vimbax_camera_msgs::msg::Error & error)
  : error{error}
  {
    std::stringstream sstream{};
    sstream << "Event subscription failed with " << error.code << " (" << error.text << ")";
    what_ = sstream.str();
  }

  const vimbax_camera_msgs::msg::Error error;
  std::string what_;

  const char * what() const noexcept override
  {
    return what_.c_str();
  }
};

class EventSubscriberBase : public std::enable_shared_from_this<EventSubscriberBase>
{
public:
  template<typename T>
  class EventSubscription
  {
    /* *INDENT-OFF* */
  public:
    /* *INDENT-ON* */
    ~EventSubscription()
    {
      subscription_.reset();

      auto request = std::make_shared<vimbax_camera_msgs::srv::UnsubscribeEvent::Request>();
      request->name = event_name_;

      event_subscriber_->unsubscribe_service_client_->async_send_request(request);
    }

    bool is_connected() const
    {
      return connected_;
    }
    /* *INDENT-OFF* */
  private:
    /* *INDENT-ON* */
    friend class EventSubscriberBase;

    typename rclcpp::Subscription<T>::SharedPtr subscription_{};
    std::shared_ptr<EventSubscriberBase> event_subscriber_;
    std::atomic_bool connected_{false};
    std::string event_name_;
  };

protected:
  EventSubscriberBase(rclcpp::Node::SharedPtr node, const std::string & topic);

  template<typename T>
  std::shared_future<std::shared_ptr<EventSubscription<T>>> subscribe_event(
    const std::string & event, std::function<void(const T &)> callback)
  {
    auto request = std::make_shared<vimbax_camera_msgs::srv::SubscribeEvent::Request>();
    request->name = event;

    using ResponseFuture = rclcpp::Client<vimbax_camera_msgs::srv::SubscribeEvent>::SharedFuture;

    auto subscription_promise =
      std::make_shared<std::promise<std::shared_ptr<EventSubscription<T>>>>();
    std::shared_future future{subscription_promise->get_future()};

    subscribe_service_client_->async_send_request(
      request,
      [this, event, subscription_promise = std::move(subscription_promise),
      callback = std::move(callback)](ResponseFuture future) -> void {
        auto const error = future.get()->error;
        if (error.code == 0) {
          auto subscription = std::make_shared<EventSubscription<T>>();
          subscription->event_name_ = event;
          subscription->event_subscriber_ = shared_from_this();

          subscription->subscription_ = node_->create_subscription<T>(
            event_topic_name(base_topic_, event), 10,
            [callback = std::move(callback)](std::shared_ptr<T> msg) {
              callback(*msg);
            });
          subscription_promise->set_value(subscription);
        } else {
          auto exception_ptr = std::make_exception_ptr(EventSubscribeException{error});
          subscription_promise->set_exception(exception_ptr);
        }
      });

    return future;
  }

private:
  std::string base_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<vimbax_camera_msgs::srv::SubscribeEvent>::SharedPtr subscribe_service_client_;
  rclcpp::Client<vimbax_camera_msgs::srv::UnsubscribeEvent>::SharedPtr unsubscribe_service_client_;
};

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_BASE_HPP_
