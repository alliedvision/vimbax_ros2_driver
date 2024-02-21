#ifndef VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_
#define VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_

#include "vimbax_camera_events/visibility_control.h"

#include <string>

namespace vimbax_camera_events
{

std::string event_topic_name(const std::string & base_topic, const std::string & event_name);
std::string subscribe_topic_name(const std::string & base_topic);
std::string unsubscribe_topic_name(const std::string & base_topic);

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_
