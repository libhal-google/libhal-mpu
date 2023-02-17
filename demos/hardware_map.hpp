#pragma once

#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

struct hardware_map
{
  hal::serial* console;
  hal::i2c* i2c;
  hal::steady_clock* clock;
  hal::callback<void()> reset;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
hal::status application(hardware_map& p_map);
hal::result<hardware_map> initialize_target();
