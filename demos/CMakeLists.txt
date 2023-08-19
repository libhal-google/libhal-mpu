# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project(demos VERSION 0.0.1 LANGUAGES CXX)

find_package(libhal-lpc40 REQUIRED CONFIG)
find_package(libhal-mpu REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)

set(platform_library $ENV{LIBHAL_PLATFORM_LIBRARY})
set(platform $ENV{LIBHAL_PLATFORM})

set(DEMOS mpu6050)

foreach(demo IN LISTS DEMOS)
    set(current_project ${demo}.elf)
    message(STATUS "Generating Demo for \"${current_project}\"")
    add_executable(${current_project}
        main.cpp
        platforms/${platform}.cpp
        applications/${demo}.cpp)
    target_compile_options(${current_project} PRIVATE -Wall -Wextra)
    target_compile_features(${current_project} PRIVATE cxx_std_20)
    target_link_options(${current_project} PRIVATE
        --oslib=semihost --crt0=minimal)
    target_link_libraries(${current_project} PRIVATE
        libhal::${platform_library}
        libhal::mpu)
    libhal_post_build(${current_project})
endforeach()