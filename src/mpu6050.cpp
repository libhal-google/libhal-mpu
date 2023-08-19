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

#include "mpu6050_constants.hpp"
#include <libhal-mpu/mpu6050.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>

namespace hal::mpu {

namespace {

hal::status active_mode(hal::i2c& p_i2c, hal::byte p_address, bool p_is_active)
{
  constexpr auto sleep_mask = hal::bit_mask::from<6>();

  auto control = HAL_CHECK(
    hal::write_then_read<1>(p_i2c,
                            p_address,
                            std::array{ hal::mpu::initalizing_register },
                            hal::never_timeout()));

  hal::bit_modify(control[0]).insert<sleep_mask>(!p_is_active);

  HAL_CHECK(hal::write(p_i2c,
                       p_address,
                       std::array{ hal::mpu::initalizing_register, control[0] },
                       hal::never_timeout()));
  return hal::success();
}

/// Verify that the device is the correct device
hal::status is_valid_device(hal::i2c& p_i2c, hal::byte p_address)
{
  static constexpr hal::byte expected_device_id = 0x68;
  // Read out the identity register
  auto device_id =
    HAL_CHECK(hal::write_then_read<1>(p_i2c,
                                      p_address,
                                      std::array{ hal::mpu::who_am_i_register },
                                      hal::never_timeout()));

  if (device_id[0] != expected_device_id) {
    return hal::new_error(std::errc::illegal_byte_sequence);
  }

  return hal::success();
}

}  // namespace

/**
 * @brief Construct a new mpu6050 object
 *
 * @param p_i2c - I2C bus the MPU is connected to
 * @param p_device_address - address of the mpu6050
 */
mpu6050::mpu6050(hal::i2c& p_i2c, hal::byte p_device_address)
  : m_i2c(&p_i2c)
  , m_address(p_device_address)
{
}
/**
 * @brief Constructs and returns MPU object
 *
 * @param p_i2c - I2C bus the MPU is connected to
 * @param p_device_address - address of the mpu6050
 * @return mpu6050 object
 * @throws std::errc::invalid_byte_sequence - when ID register does not match
 * the expected ID for the MPU6050 device.
 */
result<mpu6050> mpu6050::create(hal::i2c& p_i2c, hal::byte p_device_address)
{
  mpu6050 mpu(p_i2c, p_device_address);
  HAL_CHECK(mpu.power_on());
  return mpu;
}

/**
 * @brief Changes the gravity scale that the MPU is reading. The larger the
 * scale, the less precise the reading.
 *
 * @param p_gravity_code - Scales in powers of 2 up to 16.
 * @return hal::status - success or errors from i2c communication
 */
[[nodiscard]] hal::status mpu6050::configure_full_scale(
  max_acceleration p_gravity_code)
{
  m_gscale = static_cast<hal::byte>(p_gravity_code);

  constexpr auto scale_mask = hal::bit_mask::from<3, 4>();

  auto config =
    HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                      m_address,
                                      std::array{ configuration_register },
                                      hal::never_timeout()));

  hal::bit_modify(config[0]).insert<scale_mask>(m_gscale);
  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array{ configuration_register, config[0] },
                       hal::never_timeout()));
  return hal::success();
}

/**
 * @brief Re-enables acceleration readings from the MPU
 *
 * @return hal::status - success or errors from i2c communication
 * @throws std::errc::invalid_byte_sequence - when ID register does not match
 * the expected ID for the MPU6050 device.
 */
[[nodiscard]] hal::status mpu6050::power_on()
{
  HAL_CHECK(is_valid_device(*m_i2c, m_address));
  return active_mode(*m_i2c, m_address, true);
}

/**
 * @brief Disables acceleration reading from the MPU.
 *
 * @return hal::status - success or errors from i2c communication
 */
[[nodiscard]] hal::status mpu6050::power_off()
{
  return active_mode(*m_i2c, m_address, false);
}

hal::result<accelerometer::read_t> mpu6050::driver_read()
{
  accelerometer::read_t acceleration;
  constexpr uint16_t bytes_per_axis = 2;
  constexpr uint8_t number_of_axis = 3;

  std::array<hal::byte, bytes_per_axis * number_of_axis> xyz_acceleration;
  HAL_CHECK(hal::write_then_read(*m_i2c,
                                 m_address,
                                 std::array{ xyz_register },
                                 xyz_acceleration,
                                 hal::never_timeout()));
  /**
   * First X-axis Byte (MSB first)
   * =========================================================================
   * Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
   *  XD15 | XD14  |  XD13 |  XD12 |  XD11 |  XD10 |  XD9  |  XD8
   *
   * Final X-axis Byte (LSB)
   * =========================================================================
   * Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
   *   XD7 |   XD6 |   XD5 |   XD4 |   XD3 |   XD2 |   XD1 |   XD0
   *
   *
   * We simply shift and OR the bytes together to get them into a signed int
   * 16 value.
   */
  const int16_t x =
    static_cast<int16_t>(xyz_acceleration[0] << 8 | xyz_acceleration[1]);
  const int16_t y =
    static_cast<int16_t>(xyz_acceleration[2] << 8 | xyz_acceleration[3]);
  const int16_t z =
    static_cast<int16_t>(xyz_acceleration[4] << 8 | xyz_acceleration[5]);

  // Convert the 16 bit value into a floating point value m/S^2
  constexpr float max = static_cast<float>(std::numeric_limits<int16_t>::max());
  constexpr float min = static_cast<float>(std::numeric_limits<int16_t>::min());

  const float output_limits =
    static_cast<float>(1 << (static_cast<int>(m_gscale) + 1));
  auto input_range = std::make_pair(max, min);
  auto output_range = std::make_pair(-output_limits, output_limits);

  acceleration.x = hal::map(x, input_range, output_range);
  acceleration.y = hal::map(y, input_range, output_range);
  acceleration.z = hal::map(z, input_range, output_range);

  return acceleration;
}

}  // namespace hal::mpu
