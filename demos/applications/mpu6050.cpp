
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <mpu6050.hpp>

#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& i2c = *p_map.i2c;

  hal::print(console, "mpu Application Starting...\n\n");
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

  return hal::success();
}
