#include "vimbax_camera_events/vimbax_camera_events.hpp"

namespace vimbax_camera_events
{

std::string event_topic_name(const std::string & base_topic, const std::string & event_name)
{
  return base_topic + "/event_" + event_name;
}

std::string subscribe_topic_name(const std::string & base_topic)
{
  return base_topic + "/_event_subscribe";
}

std::string unsubscribe_topic_name(const std::string & base_topic)
{
  return base_topic + "/_event_unsubscribe";
}

}  // namespace vimbax_camera_events
