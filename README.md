# esp32_celest

**Celest** is an ESP-IDF-based firmware designed for robust UART-to-TCP bridging. It serves as a flexible framework for communication between embedded devices and network interfaces, with a focus on modularity and maintainability.

## Features

- **UART-to-TCP Bridge**: Efficient communication via serial and TCP/IP protocols.
- **Modular Architecture**: Ongoing refactoring to enhance structure and reusability.
- **ESP-IDF Integration**: Built to leverage the robust features of the ESP-IDF ecosystem.

## Project Status

This repository is currently in pre-alpha with an emphasis on:

- Refactoring the primary logic in `app_main.c` into a modular file structure.
- Improving readability, maintainability, and scalability.

## Working of `app_main`

The `app_main` function serves as the entry point for the firmware and initializes critical components:

1. **Hardware Initialization**: 
   - Configures UART for serial communication.
   - Sets up GPIO and other required peripherals.

2. **Task Creation**: 
   - Creates FreeRTOS tasks for handling TCP and UART communication asynchronously.

3. **Network Initialization**: 
   - Configures the ESP32 to connect to a network via Wi-Fi.
   - Establishes a TCP server for incoming connections.

4. **Event Handling**: 
   - Manages UART data transmission to TCP and vice versa using buffers.
   - Ensures error handling and reconnections for robust performance.

The goal of the ongoing refactor is to modularize these responsibilities, separating concerns into distinct files to simplify maintenance and testing.

## Directory Structure

Key components include:

- **`src/`**: Contains the core application logic.  
  - `app_main.c`: Entry point for the firmware, managing initialization and task handling. (Under refactoring)
- **`include/`**: Header files for modularized components.
- **`build.sh`**: Script for building the firmware.

## Build Instructions

1. Set up the ESP-IDF environment.  
2. Clone the repository:
   ```bash
   git clone https://github.com/vikramvel/esp32_celest.git
   cd esp32_celest
   ```
3. Execute the build script:
   ```bash
   ./build.sh
   ```

## Future Enhancements

- Modular separation of UART and TCP handling logic.
- Integration of testing frameworks for better reliability.
- Improved documentation and examples.

## License

This project is licensed under the GPL-2.0 License.

---
