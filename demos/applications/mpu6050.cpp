// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libhal-mpu/mpu6050.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../hardware_map.hpp"

hal::status application(hal::mpu::hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& i2c = *p_map.i2c;
  hal::print(console, "mpu Application Starting...\n");
  auto mpu = HAL_CHECK(hal::mpu::mpu6050::create(i2c, 0x68));

  (void)hal::delay(clock, 500ms);
  hal::print(console, "Reading acceleration... \n");
  HAL_CHECK(mpu.configure_full_scale(hal::mpu::mpu6050::max_acceleration::g2));
  (void)hal::delay(clock, 500ms);
  auto acceleration = HAL_CHECK(mpu.read());
  hal::print<128>(console,
                  "Scale: 2g \t x = %fg, y = %fg, z = %fg \n",
                  acceleration.x,
                  acceleration.y,
                  acceleration.z);
  (void)hal::delay(clock, 500ms);
  hal::print(console, "Reading acceleration... \n");
  HAL_CHECK(mpu.configure_full_scale(hal::mpu::mpu6050::max_acceleration::g4));
  (void)hal::delay(clock, 500ms);
  acceleration = HAL_CHECK(mpu.read());
  hal::print<128>(console,
                  "Scale: 4g \t x = %fg, y = %fg, z = %fg \n\n",
                  acceleration.x,
                  acceleration.y,
                  acceleration.z);
  return hal::success();
}
