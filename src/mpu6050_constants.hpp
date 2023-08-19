#include <libhal/units.hpp>

namespace hal::mpu{


  /// The address of the read-only register containing the temperature data.
  static constexpr hal::byte xyz_register = 0x3B;
  /// The address of the register used to configure gravity scale the device.
  static constexpr hal::byte configuration_register = 0x1C;
  /// The address of the register used to initilize the device.
  static constexpr hal::byte initalizing_register = 0x6B;

  /// The command to enable one-shot shutdown mode.
  static constexpr hal::byte who_am_i_register = 0x75;

}