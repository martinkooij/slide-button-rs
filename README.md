# Slide Button RS

Slide Button RS is a Rust-based project designed for embedded systems, specifically targeting the ESP32-C3 microcontroller. It leverages the ESP HAL for button control of WiFi-automated curtains from the Slide company. This project uses embedded Rust with ESP HAL, if only for fun.

## Features
- Controls the "Slide" automated curtains via its WiFi local API.
- Uses the ESP32-C3 microcontroller.
- Integration with ESP HAL (no-std Rust).
- Optimized for embedded systems with low memory usage.
- Includes networking capabilities using Smoltcp.


## Requirements
- Rust 1.88 or later.
- ESP32-C3 development board wired with an LED and a 220-ohm resistor in series on pin 10, and a button (high when pressed) on pin 0.
- Cargo for building and managing dependencies.
- Slide automation on the network with a fixed IP address.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/martinkooij/slide-button-rs.git
   ```
2. Navigate to the project directory:
   ```bash
   cd slide-button-rs
   ```
3. Create the local variables for SSID/PASSWORD. *Inspect the code for the SLIDE_IP address (and port) and change if necessary.*
    ```bash
    SSID=myssid
    PASSWORD=mypassword
    export SSID
    export PASSWORD
    ```
    Alternatively, use the `.vscode/settings.json` file to set these variables, or any other method your development environment allows.

4. Build the project:
   ```bash
   cargo build --release
   ```

## Usage
To run the default binary:
```bash
cargo run --release
```

## Configuration
The project uses a `Cargo.toml` file for managing dependencies and build configurations. The `default-run` key is set to `slide-button-rs` to specify the default binary.

## User Manual
### Button Control
- Short press the button once to control the curtains. If the curtains are open (less than half closed), they will close. If the curtains are closed (more than half closed), they will open.
- If the button is pressed while the curtains are moving, the movement will stop.

### LED Behavior
- At startup, the LED will remain continuously on until a WiFi connection is successfully established.
- After that, the LED will blink on and off during operation.
- The LED will turn off again once the curtain operation is finished.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Future Plans
### Short-Term Goals
- Allow the Slide IP and port to be set via environment variables.
- Clean up code and documentation.
- Potentially adapt LED behavior to be more intuitive while maintaining the one LED/one button structure for interaction.

### Long-Term Goals
- **Option 1:** Create an interface to set variables (e.g., enter AP mode on a long button press and present a simple web interface with four variables and an "Enter" button).
- **Option 2:** Create a button interface with Home Assistant and let Home Assistant handle the Slide logic.
- **Option 3:** Do nothing. The simplicity of the current setup is quite satisfying.
