#pragma once

#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>

namespace hal::mpu
{
class mpu6050 : public hal::accelerometer
{
 public:
  /// The device address when A0 is connected to GND.
  static constexpr hal::byte address_ground = 0b110'1000;
  /// The device address when A0 is connected to V+.
  static constexpr hal::byte address_voltage_high = 0b110'1001;
  /// The device address when A0 is connected to SDA.
  static constexpr hal::byte address_sda = 0b100'1010;
  /// The device address when A0 is connected to SCL.
  static constexpr hal::byte address_scl = 0b100'1011;

  /// The address of the read-only register containing the temperature data.
  static constexpr hal::byte xyz_register = 0x3B;
  /// The address of the register used to configure gravity scale the device.
  static constexpr hal::byte configuration_register = 0x1C;
  /// The address of the register used to initilize the device.
  static constexpr hal::byte initalizing_register = 0x6B;

  /// The command to enable one-shot shutdown mode.
  static constexpr hal::byte who_am_i_register = 0x75;

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
   * @param i2c - I2C bus the MPU is connected to
   * @param device_address - 0x68 if address pin is pulled low, 0x69 if pulled
   * high
   * @return result<mpu6050> std::errc::invalid_byte_sequence will be thrown
   * when MPU6050 internal register does not have the correct ID
   */
  static result<mpu6050> create(hal::i2c & p_i2c,
                                hal::byte p_device_address = address_ground)
  {
    mpu6050 mpu(p_i2c, p_device_address);
    HAL_CHECK(mpu.power_on());
    return mpu;
  }

  /**
   * @brief Changes the gravity scale that the MPU is reading. The larger the
   * scale, the less precise the reading.
   *
   * @param gravity_scale - Scales in powers of 2 up to 16.
   * @return hal::status - success or errors from i2c communication
   */
  [[nodiscard]] hal::status configure_full_scale(
      max_acceleration p_gravity_code)
  {
    m_gscale = static_cast<hal::byte>(p_gravity_code);

    constexpr auto scale_mask = hal::bit::mask::from<3, 4>();

    auto config =
        HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                          m_address,
                                          std::array{ configuration_register },
                                          hal::never_timeout()));

    hal::bit::modify(config[0]).insert<scale_mask>(m_gscale);
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
   * std::errc::invalid_byte_sequence will be thrown when MPU6050 internal
   * register does not have the correct ID
   */
  [[nodiscard]] hal::status power_on()
  {
    HAL_CHECK(is_valid_device());
    return active_mode(true);
  }

  /**
   * @brief Disables acceleration reading from the MPU.
   *
   * @return hal::status - success or errors from i2c communication
   */
  [[nodiscard]] hal::status power_off()
  {
    return active_mode(false);
  }

 private:
  /**
   * @brief MPU6050 Constructor
   *
   * @param i2c - i2c peripheral used to commnicate with device.
   * @param address - mpu6050 device address.
   */
  explicit constexpr mpu6050(i2c & p_i2c, hal::byte p_device_address = 0x68)
      : m_i2c(&p_i2c), m_address(p_device_address)
  {
  }

  hal::result<accelerometer::read_t> driver_read() override
  {
    accelerometer::read_t acceleration;
    constexpr uint16_t bytes_per_axis = 2;
    constexpr uint8_t number_of_axis  = 3;

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
    constexpr float max =
        static_cast<float>(std::numeric_limits<int16_t>::max());
    constexpr float min =
        static_cast<float>(std::numeric_limits<int16_t>::min());

    const float output_limits =
        static_cast<float>(1 << (static_cast<int>(m_gscale) + 1));
    auto input_range  = std::make_pair(max, min);
    auto output_range = std::make_pair(-output_limits, output_limits);

    acceleration.x = hal::map(x, input_range, output_range);
    acceleration.y = hal::map(y, input_range, output_range);
    acceleration.z = hal::map(z, input_range, output_range);

    return acceleration;
  }

  hal::status active_mode(bool p_is_active = true)
  {
    constexpr auto sleep_mask = hal::bit::mask::from<6>();

    auto control =
        HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                          m_address,
                                          std::array{ initalizing_register },
                                          hal::never_timeout()));

    hal::bit::modify(control[0]).insert<sleep_mask>(!p_is_active);

    HAL_CHECK(hal::write(*m_i2c,
                         m_address,
                         std::array{ initalizing_register, control[0] },
                         hal::never_timeout()));
    return hal::success();
  }

  /// Verify that the device is the correct device
  hal::status is_valid_device()
  {
    static constexpr hal::byte expected_device_id = 0x68;
    // Read out the identity register
    auto device_id =
        HAL_CHECK(hal::write_then_read<1>(*m_i2c,
                                          m_address,
                                          std::array{ who_am_i_register },
                                          hal::never_timeout()));

    if (device_id[0] != expected_device_id)
    {
      return hal::new_error(std::errc::illegal_byte_sequence);
    }

    return hal::success();
  }

  /// The I2C peripheral used for communication with the device.
  hal::i2c * m_i2c;
  /// Gscale is the min and maximum gs the device will read.
  hal::byte m_gscale = 0x00;
  /// The configurable device address used for communication.
  hal::byte m_address;
};

}  // namespace hal::mpu
