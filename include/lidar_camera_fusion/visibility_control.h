// Copyright (c) 2022 OUXT Polaris
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

#ifndef lidar_camera_fusion__VISIBILITY_CONTROL_H_
#define lidar_camera_fusion__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define lidar_camera_fusion_EXPORT __attribute__((dllexport))
#define lidar_camera_fusion_IMPORT __attribute__((dllimport))
#else
#define lidar_camera_fusion_EXPORT __declspec(dllexport)
#define lidar_camera_fusion_IMPORT __declspec(dllimport)
#endif
#ifdef lidar_camera_fusion_BUILDING_LIBRARY
#define lidar_camera_fusion_PUBLIC lidar_camera_fusion_EXPORT
#else
#define lidar_camera_fusion_PUBLIC lidar_camera_fusion_IMPORT
#endif
#define lidar_camera_fusion_PUBLIC_TYPE lidar_camera_fusion_PUBLIC
#define lidar_camera_fusion_LOCAL
#else
#define lidar_camera_fusion_EXPORT __attribute__((visibility("default")))
#define lidar_camera_fusion_IMPORT
#if __GNUC__ >= 4
#define lidar_camera_fusion_PUBLIC __attribute__((visibility("default")))
#define lidar_camera_fusion_LOCAL __attribute__((visibility("hidden")))
#else
#define lidar_camera_fusion_PUBLIC
#define lidar_camera_fusion_LOCAL
#endif
#define lidar_camera_fusion_PUBLIC_TYPE
#endif

#endif  // lidar_camera_fusion__VISIBILITY_CONTROL_H_
