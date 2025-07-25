# ruckig-frc

Ruckig frc swerve simulation and frc-specific java bindings.

[![Maven Central](https://img.shields.io/maven-central/v/org.recordrobotics.ruckig/ruckig-frc.svg?label=Maven%20Central)](https://central.sonatype.com/artifact/org.recordrobotics.ruckig/ruckig-frc)

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
