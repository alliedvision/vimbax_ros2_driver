// Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_
#define VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_

#include <string>

#include "vimbax_camera_events/visibility_control.h"

namespace vimbax_camera_events
{

std::string event_topic_name(const std::string & base_topic, const std::string & event_name);
std::string subscribe_topic_name(const std::string & base_topic);
std::string unsubscribe_topic_name(const std::string & base_topic);

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__VIMBAX_CAMERA_EVENTS_HPP_
