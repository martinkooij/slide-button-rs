#![no_std]
#![no_main]
macro_rules! timed_loop {
    ($t:expr,  $b: expr) => {{
        let deadline = time::Instant::now() + Duration::from_secs($t);
        let timed_out = 'out: loop {
            if $b {
                break 'out false;
            }
            if time::Instant::now() > deadline {
                break 'out true;
            }
        };
        !timed_out
    }};
    ($t:expr,  $b: block, $delay:expr) => {{
        let deadline = time::Instant::now() + Duration::from_secs($t);
        let timed_out = 'out: loop {
            if $b {
                break 'out false;
            }
            if time::Instant::now() > deadline {
                break 'out true;
            }
            Delay::new().delay_millis($delay);
        };
        !timed_out
    }};
}

extern crate alloc;
use core::net::Ipv4Addr;
use core::str::FromStr;

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
        Rtc, SocResetReason, reset_reason,
        sleep::{RtcioWakeupSource, TimerWakeupSource, WakeSource, WakeupLevel},
        wakeup_cause,
    },
    system::Cpu,
    time::{self, Duration},
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

esp_bootloader_esp_idf::esp_app_desc!();
// just do a reset on panic
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    esp_hal::system::software_reset()
}
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SLIDE_IP: &str = "192.168.68.104";

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // The HAL peripherals is mutable to allow repurposing the button pin in the course of the program
    let mut peripherals = esp_hal::init(config);
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut green_led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let mut red_led = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());

    green_led.set_low();
    red_led.set_low();
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    let wake_reason = wakeup_cause();
    println!("reset reason: {:?}", reason);
    println!("wake reason: {:?}", wake_reason);
    match wake_reason {
        esp_hal::system::SleepSource::Timer => {
            println!("woke up from timer");
            red_led.set_high();
            green_led.set_high();
        }
        esp_hal::system::SleepSource::Gpio => {
            println!("woke up from gpio");
            red_led.set_high();
            green_led.set_high();
        }
        _ => {
            println!("not a timer wakeup");
            green_led.set_high();
            red_led.set_low();
        }
    }
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
    let delay = Delay::new();

    // All configurations are now set
    //
    // Open a block tagged 'wifi that encapsulates the wifi interaction.
    // Leaving the block will shut down the wifi driver, free associated resources
    // and free the reborrowed pin for the button,
    // (1) to allow for further repurposing the pin of the button after this block.
    // (2) to allow for a graceful shutdown of the wifi driver to avoid spurious wifi problems

    'wifi: {
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
                    delay.delay_millis(2000u32);
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
            peripherals.GPIO2.reborrow(),
            InputConfig::default().with_pull(Pull::Up),
        );
        let success = slide_communication(
            &mut socket,
            &mut green_led,
            &mut red_led,
            &mut button_pin_input,
        );
        // just to be sure ;-)
        core::mem::drop(button_pin_input);

        if !success {
            break 'wifi;
        };

        let _ = controller.disconnect();
        let _ = controller.stop();
    } //drop wifi and associated resources

    green_led.set_low();
    red_led.set_low();
    let mut button_pin_wake = peripherals.GPIO2.reborrow();
    let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] =
        &mut [(&mut button_pin_wake, WakeupLevel::Low)];

    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    goto_deepsleep(&mut rtc, &rtcio);
}

fn goto_deepsleep(rtc: &mut Rtc, pin_wake_source: &dyn WakeSource) -> ! {
    println!("Going to deep sleep now");
    Delay::new().delay_millis(100u32);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(15));
    rtc.sleep_deep(&[&timer, pin_wake_source]);
}

fn check_button_pressed<'a>(button_pin: &mut Input<'a>, red_led: &mut Output<'a>) {
    //button is active low
    println!("Checking button state...");
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
    green_led: &mut Output<'a>,
    red_led: &mut Output<'a>,
    button_pin: &mut Input<'a>,
) -> bool {
    let delay = Delay::new();
    println!("Making HTTP request");

    // let postreq = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nContent-Length: 11\r\n\r\n{\"pos\":0.8}\r\n";
    let postreq = b"\r\nPOST /rpc/Slide.GetInfo HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n";

    socket.work();
    println!("Opening socket connection");
    check_button_pressed(button_pin, red_led);
    socket.work();

    if socket
        .open(IpAddress::Ipv4(Ipv4Addr::from_str(SLIDE_IP).unwrap()), 80)
        .is_err()
    {
        println!("Error opening socket");
        return false;
    }

    for _i in 1..=2 {
        green_led.set_low();
        socket.work();
        check_button_pressed(button_pin, red_led);

        println!(
            "Sending request: {}",
            core::str::from_utf8(postreq).unwrap()
        );
        socket.work();
        match socket.write(postreq) {
            Ok(len) => println!("Wrote {len} bytes"),
            Err(e) => {
                println!("Error on write {e:?}");
                return false;
            }
        }
        socket.flush().unwrap();
        socket.work();
        check_button_pressed(button_pin, red_led);
        let _success = timed_loop!(20, {
            socket.work();
            check_button_pressed(button_pin, red_led);
            let mut buffer = [0u8; 1024];
            if let Ok(len) = socket.read(&mut buffer) {
                println!("\n------------ len is {len}  ------------");
                println!("{}", core::str::from_utf8(&buffer[..len]).unwrap()); // there might be more data, continue reading
                true
            } else {
                false
            }
        });
        println!("Bye1 socket is still open {}", socket.is_open());
        //        sta_socket.disconnect();
        let _success = timed_loop!(
            3,
            {
                socket.work();
                check_button_pressed(button_pin, red_led);
                false
            },
            200u32
        );
        green_led.set_high();
        delay.delay_millis(500u32);
    }
    socket.close();
    socket.disconnect();
    println!("Done\n");
    true
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
