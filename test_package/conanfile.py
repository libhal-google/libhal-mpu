from conan import ConanFile
from conan.tools.build import cross_building
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain
import os


class TestPackageConan(ConanFile):
    settings = "compiler", "build_type"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def layout(self):
        self.conf_info.define("tools.build:cxxflags", ["-Dtesting=1"])
        self.conf_info.define(
            "tools.cmake.cmaketoolchain:toolchain_file", [""])
        cmake_layout(self)

    def package_info(self):
        f = os.path.join(self.package_folder, "mytoolchain.cmake")
        # Appending the value to any existing one
        self.conf_info.append("tools.cmake.cmaketoolchain:user_toolchain", f)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cmake_flags_init = ""
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        if not cross_building(self):
            bin_path = os.path.join(self.cpp.build.bindirs[0], "test_package")
            self.run(bin_path, env="conanrun")