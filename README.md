# ruckig-frc [![Maven Central](https://img.shields.io/maven-central/v/org.recordrobotics.ruckig/ruckig-frc.svg?label=Maven%20Central)](https://central.sonatype.com/artifact/org.recordrobotics.ruckig/ruckig-frc)

[Ruckig](https://github.com/pantor/ruckig) frc swerve simulation and frc-specific java bindings.

<img width="1879" height="1417" alt="Screenshot of the simulation app" src="https://github.com/user-attachments/assets/4a6b3ade-2679-47d5-893d-3b36bc002f0c" />

## Installation Instructions (WPILib Vendordep)

Online installation url: [https://maven.recordrobotics.org/ruckig-frc/vendordep/ruckig-frc.json](https://maven.recordrobotics.org/ruckig-frc/vendordep/ruckig-frc.json)

## Example usage

An example of using the library can be found at [jni/org/recordrobotics/ruckig/test/TestRuckig.java](https://github.com/recordrobotics/ruckig-frc/blob/main/jni/org/recordrobotics/ruckig/test/TestRuckig.java)

More information about the underlying Ruckig library can be found here [https://docs.ruckig.com/](https://docs.ruckig.com/)

## Build Instructions (Simulation App)

> **Prerequisite:** Make sure you have [CMake](https://cmake.org/download/) installed (version 3.28 or newer is required).
---

### Windows

1. **Configure CMake**

   Open a terminal in the project root and run:

   ```powershell
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   ```

   ### Graphics Backend (Windows)

   To select a graphics backend (default is `OpenGL`), add `-DBACKEND=OpenGL|DX11`.

   Example:

   ```powershell
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBACKEND=DX11
   ```

   **Note:** Metal backend is not supported on Windows.

   ### Custom output directory

   To change the output directory use `-DBUILD_OUTPUT_DIR=path/to/output`

   **Note:** Relative paths are relative to the build folder, not the current directory. (`-DBUILD_OUTPUT_DIR=output` is `build/output`)

2. **Build the project**

   ```powershell
   cmake --build build --config Release
   ```

3. **Run the executable**

   ```powershell
   .\build\output\ruckig_frc.exe
   ```

---

### macOS

1. **Install Ninja**

   ```sh
   brew install ninja
   ```

2. **Install LLVM/Clang**

   ```sh
   brew install llvm
   ```

3. **Configure CMake**

   Open a terminal in the project root and run:

   ```sh
   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_C_COMPILER=/usr/local/opt/llvm/bin/clang \
     -DCMAKE_CXX_COMPILER=/usr/local/opt/llvm/bin/clang++ \
     -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=/usr/local/opt/llvm/bin/clang-scan-deps \
     -DCMAKE_OSX_SYSROOT="$(xcrun --show-sdk-path)"
   ```

   ### Graphics Backend (macOS)

   To select a graphics backend (default is OpenGL), add `-DBACKEND=OpenGL|Metal`.

   Example:

   ```sh
   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release ... -DBACKEND=Metal ...
   ```

   **Important!:** For Metal backend, download the [Metal C++ library](https://developer.apple.com/metal/cpp/) from Apple and set the cmake `METAL_CPP_INCLUDE_PATH` option to the extracted folder:

   ```sh
   -DMETAL_CPP_INCLUDE_PATH=/path/to/metal-cpp
   ```

   Full command:

   ```sh
   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_C_COMPILER=/usr/local/opt/llvm/bin/clang \
     -DCMAKE_CXX_COMPILER=/usr/local/opt/llvm/bin/clang++ \
     -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=/usr/local/opt/llvm/bin/clang-scan-deps \
     -DCMAKE_OSX_SYSROOT="$(xcrun --show-sdk-path)" \
     -DBACKEND=Metal \
     -DMETAL_CPP_INCLUDE_PATH=./metal-cpp
   ```

   ### Custom output directory

   To change the output directory use `-DBUILD_OUTPUT_DIR=path/to/output`

   **Note:** Relative paths are relative to the build folder, not the current directory. (`-DBUILD_OUTPUT_DIR=output` is `build/output`)

4. **Build the project**

   ```sh
   cmake --build build --config Release
   ```

5. **Run the executable**

   ```sh
   ./build/output/ruckig_frc
   ```

---

### Linux

1. **Install Ninja**

   On Ubuntu/Debian:

   ```sh
   sudo apt update
   sudo apt install ninja-build
   ```

   On Fedora:

   ```sh
   sudo dnf install ninja-build
   ```

   On Arch:

   ```sh
   sudo pacman -S ninja
   ```

2. **Install Clang and clang-scan-deps**

   On Ubuntu/Debian:

   ```sh
   sudo apt install clang-19 clang-tools-19
   ```

   On Fedora:

   ```sh
   sudo dnf install clang clang-tools-extra
   ```

   On Arch:

   ```sh
   sudo pacman -S clang clang-tools-extra
   ```

3. **Install dependencies for Wayland/X11 support**

   On Ubuntu/Debian:

   ```sh
   sudo apt install wayland-protocols libwayland-dev libxkbcommon-dev libxrandr-dev libx11-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev libgl1-mesa-dev
   ```

   On Fedora:

   ```sh
   sudo dnf install wayland-protocols-devel libwayland-devel libxkbcommon-devel libXrandr-devel libX11-devel libXinerama-devel libXcursor-devel libXi-devel libXext-devel mesa-libGL-devel
   ```

   On Arch:

   ```sh
   sudo pacman -S wayland libxkbcommon libxrandr libx11 libxinerama libxcursor libxi libxext mesa
   ```

4. **Configure CMake**

   Open a terminal in the project root and run:

   ```sh
   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_C_COMPILER=clang-19 \
     -DCMAKE_CXX_COMPILER=clang++-19 \
     -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=clang-scan-deps-19
   ```

   **Note**: The only graphics backend supported on Linux is OpenGL

   ### Custom output directory

   To change the output directory use `-DBUILD_OUTPUT_DIR=path/to/output`

   **Note:** Relative paths are relative to the build folder, not the current directory. (`-DBUILD_OUTPUT_DIR=output` is `build/output`)

5. **Build the project**

   ```sh
   cmake --build build --config Release
   ```

6. **Run the executable**

   ```sh
   ./build/output/ruckig_frc
   ```

---

## Build Instructions (JNI)

### Native Build (Host Compilation)

This is for building JNI on your current system (no cross-compiling).

1. **Navigate to the JNI directory:**

   ```sh
   cd jni
   ```

2. **Configure CMake for a Release build:**

   ```sh
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   ```

3. **Build the native library:**

   ```sh
   cmake --build build --config Release
   ```

4. **Build the Java bindings:**

   ```sh
   ./gradlew build
   ```

5. **Publish the Maven package:**

   ```sh
   ./gradlew publish
   ```

   The published Maven package will be located at:

   ```sh
   jni/build/deploy
   ```

---

### Cross-Compiling (Target: Athena or ARM64)

Use this section if you need to build for a different architecture (e.g., roboRIO Athena or ARM64).

#### 1. Download the WPILib toolchain

- Go to [WPILib OpenSDK Releases](https://github.com/wpilibsuite/opensdk/releases/latest)
- Download the toolchain matching your target and host system.
- Extract the `.tgz` file.

#### 2. Set the `TOOLCHAIN_ROOT` environment variable

Set this to the root folder of the extracted toolchain. Example (adjust path as needed):

**Windows (PowerShell):**

```pwsh
$env:TOOLCHAIN_ROOT="C:\path\to\toolchain"
```

**Linux/macOS (bash):**

```sh
export TOOLCHAIN_ROOT=/path/to/toolchain
```

#### 3. Navigate to the JNI directory

```sh
cd jni
```

#### 4. Configure CMake for cross-compiling

Choose the correct toolchain file for your target (`athena` or `arm64`):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/athena-toolchain.cmake
```

or

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/arm64-toolchain.cmake
```

#### 5. Build the native library

```sh
cmake --build build --config Release
```

#### 6. Build the Java bindings

```sh
./gradlew build
```

#### 7. Publish the Maven package

```sh
./gradlew publish
```

The published Maven package will be located at:

```sh
jni/build/deploy
```

---
