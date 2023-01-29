cmake_minimum_required(VERSION 3.20)

project(demos VERSION 0.0.1 LANGUAGES CXX)

find_package(libhal-lpc40xx REQUIRED CONFIG)
find_package(libmpu REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)

set(DEMOS mpu6050)
set(TARGETS lpc4078 lpc4074)

foreach(target IN LISTS TARGETS)
    foreach(demo IN LISTS DEMOS)
        set(current_project ${target}_${demo}.elf)
        message(STATUS "Generating Demo for \"${current_project}\"")
        add_executable(${current_project}
            main.cpp
            newlib.cpp
            targets/${target}/initializer.cpp
            applications/${demo}.cpp)
        target_include_directories(${current_project} PUBLIC
            ./
            targets/${target})
        target_compile_features(${current_project} PRIVATE cxx_std_20)
        target_link_options(${current_project} PRIVATE -u _printf_float)
        target_link_libraries(${current_project} PRIVATE
            libhal::${target}
            libhal::mpu
            libhal::util)
        arm_post_build(${current_project})
    endforeach()
endforeach()
