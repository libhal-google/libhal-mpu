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

#pragma once

#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>

namespace hal::mpu {
class mpu6050 : public hal::accelerometer
{
public:
  /// The device address when A0 is connected to GND.
  static constexpr hal::byte address_ground = 0b110'1000;
  /// The device address when A0 is connected to V+.
  static constexpr hal::byte address_voltage_high = 0b110'1001;

  enum class max_acceleration : hal::byte
  {
    /// 2x the average earth gravity of acceleration
    g2 = 0x00,
    /// 4x the average earth gravity of acceleration
    g4 = 0x01,
    /// 8x the average earth gravity of acceleration
    g8 = 0x02,
    /// 16x the average earth gravity of acceleration
    g16 = 0x03,
  };

  /**
   * @brief Constructs and returns MPU object
   *
   * @param p_i2c - I2C bus the MPU is connected to
   * @param p_device_address - address of the mpu6050
   * @return mpu6050 object
   * @throws std::errc::invalid_byte_sequence - when ID register does not match
   * the expected ID for the MPU6050 device.
   */
  static result<mpu6050> create(hal::i2c& p_i2c,
                                hal::byte p_device_address = address_ground);

  /**
   * @brief Changes the gravity scale that the MPU is reading. The larger the
   * scale, the less precise the reading.
   *
   * @param p_gravity_code - Scales in powers of 2 up to 16.
   * @return hal::status - success or errors from i2c communication
   */
  [[nodiscard]] hal::status configure_full_scale(
    max_acceleration p_gravity_code);

  /**
   * @brief Re-enables acceleration readings from the MPU
   *
   * @return hal::status - success or errors from i2c communication
   * @throws std::errc::invalid_byte_sequence - when ID register does not match
   * the expected ID for the MPU6050 device.
   */
  [[nodiscard]] hal::status power_on();

  /**
   * @brief Disables acceleration reading from the MPU.
   *
   * @return hal::status - success or errors from i2c communication
   */
  [[nodiscard]] hal::status power_off();

private:
  /**
   * @brief MPU6050 Constructor
   *
   * @param p_i2c - i2c peripheral used to commnicate with device.
   * @param p_address - mpu6050 device address.
   */
  explicit mpu6050(i2c& p_i2c, hal::byte p_device_address);

  hal::result<accelerometer::read_t> driver_read() override;

  /// The I2C peripheral used for communication with the device.
  hal::i2c* m_i2c;
  /// Gscale is the min and maximum gs the device will read.
  hal::byte m_gscale;
  /// The configurable device address used for communication.
  hal::byte m_address;
};

}  // namespace hal::mpu
