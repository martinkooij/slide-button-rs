#![no_std]
#![no_main]

/// esp-slide-rs: Hobby button for ESP32-S2/S3
/// Controls the "Slide" curtain by a button press.
/// When the button is pressed, the curtain will either open or close,
/// depending on its current position. When moving, when the button is pressed again,
/// the curtain will stop its movement
///
/// Martin Kooij 2025-06-10, MIT Licenced
extern crate alloc;
mod util;
use crate::util::timed_loop;
use core::net::Ipv4Addr;
use core::str::FromStr;
use serde::Deserialize;
use serde_json_core as json;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_alloc as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{self, Input, InputConfig, Level, Output, OutputConfig, Pull},
    main, ram,
    rng::Rng,
    rtc_cntl::{
        Rtc,
        sleep::{RtcioWakeupSource, WakeupLevel},
        wakeup_cause,
    },
    time,
    timer::timg::TimerGroup,
};

use esp_println::{dbg, println};
use esp_radio::wifi::{ClientConfig, ModeConfig};
use esp_rtos as _;
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    socket::dhcpv4::{RetryConfig, Socket as Dhcpv4Socket},
    time::Duration as SmolDuration,
    wire::{DhcpOption, IpAddress},
};
#[derive(Deserialize)]
#[allow(unused)]
struct SlideData<'b> {
    slide_id: &'b str,
    mac: &'b str,
    board_rev: u8,
    device_name: &'b str,
    zone_name: &'b str,
    curtain_type: u8,
    calib_time: i64,
    pos: f32,
    touch_go: bool,
}

esp_bootloader_esp_idf::esp_app_desc!();
// just do a reset on panic
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    esp_hal::system::software_reset()
}
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SLIDE_IP: &str = "192.168.68.104";
const SLIDE_PORT: u16 = 80;
// const SLIDE_IP: &str = "80.114.243.107";
// const SLIDE_PORT: u16 = 12012;
enum SlideCommand {
    Open,
    Close,
    Stop,
}

#[main]
fn main() -> ! {
    // ----------------------------------------------
    // Initialization with a lot of boilerplate
    // from esp-hal examples
    // ----------------------------------------------
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // The HAL peripherals is mutable to allow repurposing the button pin in the course of the program
    let mut peripherals = esp_hal::init(config);
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut red_led = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    red_led.set_low();

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    // ----------------------------------------------
    // Main Program Logic
    // ----------------------------------------------
    //
    // Open a block tagged 'wifi that encapsulates the wifi interaction.
    // Leaving the block will shut down the wifi driver, free associated resources
    // and free the reborrowed pin for the button,
    // (1) to allow for further repurposing the pin of the button after this block.
    // (2) to allow for a graceful shutdown of the wifi driver to avoid spurious wifi problems

    'wifi: {
        // if we enter this block because of any other wakeup reason
        // then button interrupt skip the interaction part and goto deepsleep again
        let wake_reason = wakeup_cause();
        println!("wake reason: {:?}", wake_reason);
        match wake_reason {
            esp_hal::system::SleepSource::Gpio => {
                println!("woke up from button press");
                red_led.set_high();
            }
            _ => {
                println!("not a button press wakeup");
                break 'wifi;
            }
        }

        let esp_radio_ctrl = esp_radio::init().unwrap();
        let (mut controller, interfaces) =
            esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();
        let mut sta_device = interfaces.sta;
        let sta_iface = create_interface(&mut sta_device);

        let mut socket_set_entries: [SocketStorage; 3] = Default::default();
        let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
        let mut dhcp_socket = Dhcpv4Socket::new();
        let mut retry_config = RetryConfig::default();
        retry_config.discover_timeout = SmolDuration::from_millis(500);
        dhcp_socket.set_retry_config(retry_config);
        // we can set a hostname here (or add other DHCP options)
        dhcp_socket.set_outgoing_options(&[DhcpOption {
            kind: 12,
            data: b"esp-slide-button",
        }]);
        socket_set.add(dhcp_socket);

        let rng = Rng::new();
        let now = || time::Instant::now().duration_since_epoch().as_millis();
        let stack = Stack::new(sta_iface, sta_device, socket_set, now, rng.random());

        controller
            .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
            .unwrap();

        let client_config = ModeConfig::Client(
            ClientConfig::default()
                .with_ssid(SSID.into())
                .with_password(PASSWORD.into()),
        );
        let res = controller.set_config(&client_config);
        println!("wifi_set_configuration returned {:?}", res);

        controller.start().unwrap();
        println!("is wifi started: {:?}", controller.is_started());

        println!("{:?}", controller.capabilities());
        println!("wifi connecting... {:?}", controller.connect());

        // wait to get connected, no longer then 5 seconds
        let success = timed_loop!(5, {
            match controller.is_connected() {
                Ok(true) => true,
                Ok(false) => false,
                Err(err) => {
                    println!("{:?}", err);
                    Delay::new().delay_millis(1000u32);
                    false
                }
            }
        });
        if !success {
            break 'wifi;
        };
        println!("Wifi connected:{:?}", controller.is_connected());

        // wait for getting an ip address
        println!("Wait to get an ip address");
        let success = timed_loop!(12, {
            stack.work();
            if stack.is_iface_up() {
                println!("got ip {:?}", stack.get_ip_info());
                true
            } else {
                false
            }
        });
        if !success {
            break 'wifi;
        };

        let mut rx_buffer = [0u8; 1536];
        let mut tx_buffer = [0u8; 1536];
        let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

        // --------------------------------------------
        // Sliding curtain control
        // --------------------------------------------
        println!("Starting slide control");
        let mut button_pin_input = Input::new(
            peripherals.GPIO0.reborrow(),
            InputConfig::default().with_pull(Pull::Down),
        );
        slide_communication(&mut socket, &mut red_led, &mut button_pin_input);
        // always break down just to be sure ;-)
        core::mem::drop(button_pin_input);
        let _ = controller.disconnect();
        let _ = controller.stop();
    } //drop wifi and associated resources

    // ----------------------------------------------
    // Go to Deep Sleep, Low power usage mode
    // ----------------------------------------------

    println!("Setting things up for deep sleep");
    //set the Pull down and Input type on the button pin
    //needed because we might have skipped the wifi section
    //and be left with uninitialsed button pin
    //there is no harm in initialising it twice
    //drop the input pin handler immedately to be able to reuse
    let button_pin_input = Input::new(
        peripherals.GPIO0.reborrow(),
        InputConfig::default().with_pull(Pull::Down),
    );
    core::mem::drop(button_pin_input);

    let mut button_pin_wake = peripherals.GPIO0.reborrow();
    let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] =
        &mut [(&mut button_pin_wake, WakeupLevel::High)];
    let pin_wake_source = RtcioWakeupSource::new(wakeup_pins);
    println!("Setup done - going to deep sleep now");
    Delay::new().delay_millis(75u32);
    rtc.sleep_deep(&[&pin_wake_source]);
}

fn slide_communication<'a, 's, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'s, 'n, D>,
    red_led: &mut Output<'a>,
    button_pin: &mut Input<'a>,
) -> () {
    red_led.set_low();
    let mut retrieved_position = if let Ok(pos) = get_slide_position(socket) {
        pos
    } else {
        return;
    };
    red_led.set_high();
    println!("Start position = {}", retrieved_position);
    if retrieved_position < 0.5 {
        println!("Slide is open, closing it");
        if set_slide_position(socket, SlideCommand::Close).is_err() {
            return;
        };
    } else {
        println!("Slide is closed, opening it");
        if set_slide_position(socket, SlideCommand::Open).is_err() {
            return;
        }
    };
    println!("Showing for slide to move...");

    let _not_timed_out = timed_loop!(40, 'button_loop: {
        // block waiting until button is not pressed anymore + wait bounce time
        while button_pin.is_high() {
            //busy_wait until low
        }
        Delay::new().delay_millis(75u32);
        let mut start_time = time::Instant::now();
        let deadline = start_time + time::Duration::from_secs(4);
        red_led.set_low();
        while time::Instant::now() < deadline {
            if start_time.elapsed() > time::Duration::from_millis(600) && button_pin.is_low() {
                red_led.set_high();
            };
            if start_time.elapsed() > time::Duration::from_millis(800) {
                red_led.set_low();
                start_time = time::Instant::now();
            };
            if button_pin.is_high() {
                println!("Button pressed during slide movement, stopping slide");
                let _ = set_slide_position(socket, SlideCommand::Stop);
                break 'button_loop true;
            }
        }
        if let Ok(current_position) = get_slide_position(socket) {
            if compare(current_position, retrieved_position) {
                println!(
                    "current position= {:?}, old position= {:?}",
                    current_position, retrieved_position
                );
                //position did not change
                println!("Slide position did not change, assuming it reached end position");
                break 'button_loop true;
            } else {
                println!(
                    "current position= {:?}, old position= {:?}",
                    current_position, retrieved_position
                );
                retrieved_position = current_position;
            }
        } else {
            println!("Could not get slide position");
        }
        false
    });
    socket.close();
    socket.disconnect();
    println!("Done slide communication\n");
    Delay::new().delay_millis(500u32);
}

fn compare(f1: f32, f2: f32) -> bool {
    let diff = if f1 > f2 { f1 - f2 } else { f2 - f1 };
    diff < 0.01
}

fn get_slide_position<'a, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'a, 'n, D>,
) -> Result<f32, ()> {
    let position_request = b"\r\nPOST /rpc/Slide.GetInfo HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n";
    let mut buffer = [0u8; 1024];
    let len = request_and_wait_for_answer(socket, position_request, &mut buffer)?;
    let str_slice = core::str::from_utf8(&buffer[..len]).unwrap();
    let possible =
        json::from_slice::<SlideData<'_>>(&buffer[str_slice.find('{').unwrap_or(0)..len]);
    if let Ok((slide_data, _)) = possible {
        Ok(slide_data.pos)
    } else {
        println!("Could not parse json data due to {:?}", possible.err());
        Err(())
    }
}

fn set_slide_position<'a, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'a, 'n, D>,
    command: SlideCommand,
) -> Result<(), ()> {
    match command {
        SlideCommand::Open => {
            let request = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 11\r\n\r\n{\"pos\":0.0}\r\n";
            let mut buffer = [0u8; 1024];
            let _len = request_and_wait_for_answer(socket, request, &mut buffer)?;
            Ok(())
        }
        SlideCommand::Close => {
            let request = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 11\r\n\r\n{\"pos\":1.0}\r\n";
            let mut buffer = [0u8; 1024];
            let _len = request_and_wait_for_answer(socket, request, &mut buffer)?;
            Ok(())
        }
        SlideCommand::Stop => soft_stop(socket),
    }
}

fn soft_stop<'a, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'a, 'n, D>,
) -> Result<(), ()> {
    let pos_value = get_slide_position(socket)?;
    let setposrequest = alloc::format!(
        "\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 12\r\n\r\n{{\"pos\":{:.2}}}\r\n",
        pos_value
    );
    let mut buffer = [0u8; 1024];
    let _len = request_and_wait_for_answer(socket, &setposrequest.as_bytes(), &mut buffer)?;
    Ok(())
}

fn request_and_wait_for_answer<'a, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'a, 'n, D>,
    request: &[u8],
    response_buffer: &mut [u8; 1024],
) -> Result<usize, ()> {
    println!("Sending request. Socket open? {}", socket.is_open());
    let printable_request = core::str::from_utf8(request).unwrap();
    dbg!("Request to be sent:\n{}", printable_request);
    socket.work();
    if socket.is_open() {
        socket.work();
        socket.close();
        socket.disconnect();
    };
    if socket
        .open(
            IpAddress::Ipv4(Ipv4Addr::from_str(SLIDE_IP).unwrap()),
            SLIDE_PORT,
        )
        .is_err()
    {
        println!("Error opening socket");
        return Err(());
    };
    socket.work();
    match socket.write(request) {
        Ok(len) => println!("Wrote {len} bytes"),
        Err(e) => {
            println!("Error on write {e:?}");
            return Err(());
        }
    }
    socket.flush().unwrap();
    socket.work();

    let mut length: usize = 0;
    let success = timed_loop!(5, {
        socket.work();
        if let Ok(len) = socket.read(response_buffer) {
            println!("received {} bytes of http response", len);
            let str_slice = core::str::from_utf8(&response_buffer[..len]).unwrap();
            dbg!("Response:{}", str_slice);
            Delay::new().delay_millis(100u32);
            length = len;
            socket.flush().unwrap();
            socket.work();
            true
        } else {
            false
        }
    });
    socket.work();
    socket.close();
    socket.disconnect();
    if !success {
        println!("Timed out waiting for response");
        return Err(());
    }
    Ok(length)
}

//some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
    // users could create multiple instances but since they only have one WifiDevice
    // they probably can't do anything bad with that
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}
