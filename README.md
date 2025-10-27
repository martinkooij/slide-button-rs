# Slide Button RS

Slide Button RS is a Rust-based project designed for embedded systems, specifically targeting the ESP32-C3 microcontroller. It leverages the ESP HAL for a operating systemless button control of my WiFi automated curtains of the Slide company. ESP HAL is used, if only just for fun. 

## Features
- controls the "Slide" automated curtains via its WiFi local API
- Uses the ESP32-C3 microcontroller.
- Integration with ESP HAL (no-std Rust)
- Optimized for embedded systems with low memory usage.
- Includes networking capabilities using Smoltcp.


## Requirements
- Rust 1.88 or later.
- ESP32-C3 development board wired with a led and 220Ohm resistor in series on pin 10, and a button (high when pressed) on pin 0. 
- Cargo for building and managing dependencies.
- Slide automation on network- with fixed IP address

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/martinkooij/slide-button-rs.git
   ```
2. Navigate to the project directory:
   ```bash
   cd slide-button-rs
   ```
3. create the local variables for SSID/PASSWORD, *inpect the code for the SLIDE_IP address (and port) and change if necessary*
    ```bash
    SSID=myssid
    PASSWORD=mypasswword
    export SSID
    export PASSWORD
    ```
    slternative is to use .vscode settings json file to set these variables, or any other way your dev environment allows. 

3. Build the project:
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

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Future plans
### short future
- allow slide IP and Port to be set by environment variables 
- clean up code and documentation
- maybe adapt led behavior to be more clear but...
- keep the one led/one button structure for interaction
### long future
- OR: create an interface to set the variables (e.g. at long press of button go to AP mode and present a simple web interface with 4 variables and an "Enter button")
- OR: create a button to interface with Home asistant and let Home Assitant handle the slide logic. 
- OR: do nothing, I am actually quite happy with the simplicity. 
