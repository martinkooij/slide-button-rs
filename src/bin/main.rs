#![no_std]
#![no_main]

extern crate alloc;
use core::cell::Cell;
use critical_section::Mutex;
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
        sleep::{RtcioWakeupSource, TimerWakeupSource, WakeSource, WakeupLevel},
    },
    time,
    timer::timg::TimerGroup,
};

use esp_println::println;
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
    calib_time: u32,
    pos: f32,
    touch_go: bool,
}
static B_PRESSED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

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
// Outside

#[main]
fn main() -> ! {
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

    // All configurations are now set
    //
    // Open a block tagged 'wifi that encapsulates the wifi interaction.
    // Leaving the block will shut down the wifi driver, free associated resources
    // and free the reborrowed pin for the button,
    // (1) to allow for further repurposing the pin of the button after this block.
    // (2) to allow for a graceful shutdown of the wifi driver to avoid spurious wifi problems

    'wifi: {
        let esp_radio_ctrl = esp_radio::init().unwrap();
        red_led.set_high();

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

        println!("Start  main work.");
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

    // goto deepsleep
    red_led.set_low();
    let mut button_pin_wake = peripherals.GPIO0.reborrow();
    let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] =
        &mut [(&mut button_pin_wake, WakeupLevel::High)];
    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    goto_deepsleep(&mut rtc, &rtcio);
}

fn goto_deepsleep(rtc: &mut Rtc, pin_wake_source: &dyn WakeSource) -> ! {
    println!("Going to deep sleep now");
    Delay::new().delay_millis(75u32);
    rtc.sleep_deep(&[pin_wake_source]);
}

fn check_button_pressed<'a>(button_pin: &mut Input<'a>, red_led: &mut Output<'a>) {
    //button is active low
    // println!("Checking button state...");
    if button_pin.is_low() {
        println!("Button is pressed!");
        red_led.toggle();
        loop {
            if button_pin.is_high() {
                println!("Button released, exiting loop");
                break;
            }
            Delay::new().delay_millis(50u32);
        }
    }
}

fn slide_communication<'a, 's, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'s, 'n, D>,
    red_led: &mut Output<'a>,
    button_pin: &mut Input<'a>,
) -> () {
    let open_request = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 11\r\n\r\n{\"pos\":0.0}\r\n";
    let close_request = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 11\r\n\r\n{\"pos\":1.0}\r\n";
    let stop_request = b"\r\nPOST /rpc/Slide.Stop HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n";

    println!("Opening socket connection");
    socket.work();

    if socket
        .open(
            IpAddress::Ipv4(Ipv4Addr::from_str(SLIDE_IP).unwrap()),
            SLIDE_PORT,
        )
        .is_err()
    {
        println!("Error opening socket");
        return;
    }

    for _i in 0..10 {
        let result = get_slide_position(socket);
        println!("result = {:?}", result);
        println!("socket is still open? {}", socket.is_open());
        Delay::new().delay_millis(2000u32);
        red_led.toggle();
    }
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
    println!("{}", str_slice);
    let possible =
        json::from_slice::<SlideData<'_>>(&buffer[str_slice.find('{').unwrap_or(0)..len]);
    if let Ok((slide_data, _)) = possible {
        Ok(slide_data.pos)
    } else {
        println!("Could not parse json data due to {:?}", possible.err());
        Err(())
    }
}

fn request_and_wait_for_answer<'a, 'n, D: smoltcp::phy::Device>(
    socket: &mut blocking_network_stack::Socket<'a, 'n, D>,
    request: &[u8],
    response_buffer: &mut [u8; 1024],
) -> Result<usize, ()> {
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
    let success = timed_loop!(20, {
        socket.work();
        if let Ok(len) = socket.read(response_buffer) {
            println!("\n------------ len is {len}  ------------");
            let str_slice = core::str::from_utf8(&response_buffer[..len]).unwrap();
            println!("{}", str_slice);
            length = len;
            true
        } else {
            false
        }
    });
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
