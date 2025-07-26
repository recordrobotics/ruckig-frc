# ruckig-frc [![Maven Central](https://img.shields.io/maven-central/v/org.recordrobotics.ruckig/ruckig-frc.svg?label=Maven%20Central)](https://central.sonatype.com/artifact/org.recordrobotics.ruckig/ruckig-frc)

[Ruckig](https://github.com/pantor/ruckig) frc swerve simulation and frc-specific java bindings.

<img width="1879" height="1417" alt="Screenshot of the simulation app" src="https://github.com/user-attachments/assets/4a6b3ade-2679-47d5-893d-3b36bc002f0c" />

## Installation Instructions (WPILib Vendordep)

Online installation url: [https://maven.recordrobotics.org/ruckig-frc/vendordep/ruckig-frc.json](https://maven.recordrobotics.org/ruckig-frc/vendordep/ruckig-frc.json)

## Example usage

An example of using the library can be found at [jni/org/recordrobotics/ruckig/test/TestRuckig.java](https://github.com/recordrobotics/ruckig-frc/blob/main/jni/org/recordrobotics/ruckig/test/TestRuckig.java)

More information about the underlying Ruckig library can be found here [https://docs.ruckig.com/](https://docs.ruckig.com/)

## Build Instructions (simulation app)

1. Run CMake to generate build files:

   ```sh
   cmake -S . -B build
   ```

2. Build the project:

   ```sh
   cmake --build build
   ```

3. Run the executable:

   ```sh
   ./build/ruckig_frc
   ```

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
