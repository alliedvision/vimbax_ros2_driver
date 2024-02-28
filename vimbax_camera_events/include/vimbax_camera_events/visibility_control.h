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

#ifndef VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_
#define VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIMBAX_CAMERA_EVENTS_EXPORT __attribute__ ((dllexport))
    #define VIMBAX_CAMERA_EVENTS_IMPORT __attribute__ ((dllimport))
  #else
    #define VIMBAX_CAMERA_EVENTS_EXPORT __declspec(dllexport)
    #define VIMBAX_CAMERA_EVENTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIMBAX_CAMERA_EVENTS_BUILDING_LIBRARY
    #define VIMBAX_CAMERA_EVENTS_PUBLIC VIMBAX_CAMERA_EVENTS_EXPORT
  #else
    #define VIMBAX_CAMERA_EVENTS_PUBLIC VIMBAX_CAMERA_EVENTS_IMPORT
  #endif
  #define VIMBAX_CAMERA_EVENTS_PUBLIC_TYPE VIMBAX_CAMERA_EVENTS_PUBLIC
  #define VIMBAX_CAMERA_EVENTS_LOCAL
#else
  #define VIMBAX_CAMERA_EVENTS_EXPORT __attribute__ ((visibility("default")))
  #define VIMBAX_CAMERA_EVENTS_IMPORT
  #if __GNUC__ >= 4
    #define VIMBAX_CAMERA_EVENTS_PUBLIC __attribute__ ((visibility("default")))
    #define VIMBAX_CAMERA_EVENTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIMBAX_CAMERA_EVENTS_PUBLIC
    #define VIMBAX_CAMERA_EVENTS_LOCAL
  #endif
  #define VIMBAX_CAMERA_EVENTS_PUBLIC_TYPE
#endif

#endif  // VIMBAX_CAMERA_EVENTS__VISIBILITY_CONTROL_H_
