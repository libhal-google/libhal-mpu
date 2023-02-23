# libhal-mpu

[![✅ Checks](https://github.com/libhal/libhal-mpu/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-mpu/actions/workflows/ci.yml)
[![coverage](https://libhal.github.io/libhal-mpu/coverage/coverage.svg)](https://libhal.github.io/libhal-mpu/coverage/)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b084e6d5962d49a9afcb275d62cd6586)](https://www.codacy.com/gh/libhal/libhal-mpu/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=libhal/libhal-mpu&amp;utm_campaign=Badge_Grade)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-mpu.svg)](https://github.com/libhal/libhal-mpu/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-mpu.svg)](https://github.com/libhal/libhal-mpu/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-mpu.svg)](https://github.com/libhal/libhal-mpu/issues)
[![Latest Version](https://libhal.github.io/libhal-mpu/latest_version.svg)](https://github.com/libhal/libhal-mpu/blob/main/conanfile.py)
[![ConanCenter Version](https://repology.org/badge/version-for-repo/conancenter/libhal-mpu.svg)](https://conan.io/center/libhal-mpu)


# [📚 Software APIs](https://libhal.github.io/libhal-mpu/api)

# 📥 Install

## [Install libhal Prerequisites](https://libhal.github.io/prerequisites/)

## Install using conan via from Conan Center Index

For future use. `libhal-mpu` is not currently on the Conan Center Index.

```bash
conan install libhal-mpu
```

## Install using conan via libhal-trunk

Trunk represents the latest code on github.

In order to get the latest code remote version of this repo as well as its
dependencies, enter this command to add the `libhal-trunk` remote server to your
list.

This command will insert `libhal-trunk` as the first server to check before
checking the conan center index.
The second command will enable revision mode which is required to use
`libhal-trunk`.

```bash
conan remote add libhal-trunk https://libhal.jfrog.io/artifactory/api/conan/trunk-conan
```

Now when you run

```
conan install libhal-mpu
```

The library will be pulled from the `libhal-trunk`.
