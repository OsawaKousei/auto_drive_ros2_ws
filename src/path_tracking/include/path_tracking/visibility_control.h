// Copyright (c) 2023 OUXT Polaris
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

#ifndef PATH_TRACKING__VISIBILITY_CONTROL_H_
#define PATH_TRACKING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PATH_TRACKING_EXPORT __attribute__((dllexport))
#define PATH_TRACKING_IMPORT __attribute__((dllimport))
#else
#define PATH_TRACKING_EXPORT __declspec(dllexport)
#define PATH_TRACKING_IMPORT __declspec(dllimport)
#endif
#ifdef PATH_TRACKING_BUILDING_LIBRARY
#define PATH_TRACKING_PUBLIC PATH_TRACKING_EXPORT
#else
#define PATH_TRACKING_PUBLIC PATH_TRACKING_IMPORT
#endif
#define PATH_TRACKING_PUBLIC_TYPE PATH_TRACKING_PUBLIC
#define PATH_TRACKING_LOCAL
#else
#define PATH_TRACKING_EXPORT __attribute__((visibility("default")))
#define PATH_TRACKING_IMPORT
#if __GNUC__ >= 4
#define PATH_TRACKING_PUBLIC __attribute__((visibility("default")))
#define PATH_TRACKING_LOCAL __attribute__((visibility("hidden")))
#else
#define PATH_TRACKING_PUBLIC
#define PATH_TRACKING_LOCAL
#endif
#define PATH_TRACKING_PUBLIC_TYPE
#endif

#endif // PATH_TRACKING__VISIBILITY_CONTROL_H_