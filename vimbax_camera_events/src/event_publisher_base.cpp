
#include <vimbax_camera_events/vimbax_camera_events.hpp>

#include <vimbax_camera_events/event_publisher_base.hpp>

namespace vimbax_camera_events
{

EventPublisherBase::EventPublisherBase(std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name, OnEventSubscribed on_event_subscribed,
  OnEventUnsubscribed on_event_unsubscribed) 
  : base_topic_name_{topic_name}, node_{node},
    on_event_unsubscribed_{std::move(on_event_unsubscribed)}
{
  subscription_service_ = node_->create_service<vimbax_camera_msgs::srv::SubscribeEvent>(
    subscribe_topic_name(base_topic_name_),
    [this, node, on_event_subscribed=std::move(on_event_subscribed)](
      const vimbax_camera_msgs::srv::SubscribeEvent::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::SubscribeEvent::Response::SharedPtr response
    ) {
      auto const it = subscribtion_detail_map_.find(request->name);
      // New subscribtion
      if (it == subscribtion_detail_map_.end()) {
        auto const res = on_event_subscribed(request->name);
        if (res != 0) {
          response->error = res;
          return;
        }

        auto detail = std::make_unique<SubscribtionDetail>();
        detail->count_ = 1;
        detail->publisher_ = 
          create_event_publisher(node, event_topic_name(base_topic_name_, request->name), 10);
        subscribtion_detail_map_.emplace(request->name, std::move(detail));
      } else {
        it->second->count_++;
      }
    });

  unsubscription_service_ = node_->create_service<vimbax_camera_msgs::srv::UnsubscribeEvent>(
    unsubscribe_topic_name(base_topic_name_),
     [this](
      const vimbax_camera_msgs::srv::UnsubscribeEvent::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::UnsubscribeEvent::Response::SharedPtr
    ) {
      unsubscribe(request->name);
    });

  using namespace std::chrono_literals;
  event_check_timer_ = node_->create_wall_timer(500ms, [this] {
    std::vector<std::string> remove_list{};
    for (auto const & [event, detail] : subscribtion_detail_map_) {
      if (detail->publisher_->get_subscription_count() == 0) {
        on_event_unsubscribed_(event);
        remove_list.push_back(event);
      }
    }

    for (auto const & event : remove_list) {
      subscribtion_detail_map_.extract(event);
    }
  });
}

void EventPublisherBase::unsubscribe(const std::string & event)
{
  auto const it = subscribtion_detail_map_.find(event);
  if (it != subscribtion_detail_map_.end()) {
    it->second->count_--;
    if (it->second->count_ == 0) {
      on_event_unsubscribed_(event);

      subscribtion_detail_map_.erase(it);
    }
  } 
}

rclcpp::PublisherBase::SharedPtr EventPublisherBase::get_event_publisher(
  const std::string & event)
{
  auto publisher = subscribtion_detail_map_.at(event)->publisher_;

  return publisher;
}

}