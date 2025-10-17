#![no_std]
#![no_main]

extern crate alloc;
use core::net::Ipv4Addr;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main, ram,
    rng::Rng,
    rtc_cntl::{sleep::TimerWakeupSource, wakeup_cause, Rtc},
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_radio::wifi::{ClientConfig, ModeConfig};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    socket::dhcpv4::{RetryConfig, Socket as Dhcpv4Socket},
    time::Duration as SmolDuration,
    wire::{DhcpOption, IpAddress},
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const _SLIDE_IP: &str = "192.168.68.104";

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut green_led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let mut red_led = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    green_led.set_low();
    red_led.set_low();
    let wake_reason = wakeup_cause();
    match wake_reason {
        esp_hal::system::SleepSource::Timer => {
            println!("woke up from timer");
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

    {
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

        // wait to get connected
        loop {
            match controller.is_connected() {
                Ok(true) => break,
                Ok(false) => {}
                Err(err) => {
                    println!("{:?}", err);
                    loop {}
                }
            }
        }
        println!("Wifi connected:{:?}", controller.is_connected());

        // wait for getting an ip address
        println!("Wait to get an ip address");
        loop {
            stack.work();

            if stack.is_iface_up() {
                println!("got ip {:?}", stack.get_ip_info());
                break;
            }
        }

        println!("Start busy loop on main");

        let mut rx_buffer = [0u8; 1536];
        let mut tx_buffer = [0u8; 1536];
        let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

        println!("Start  main work.");

        println!("Making HTTP request");

        // let postreq = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nContent-Length: 11\r\n\r\n{\"pos\":0.8}\r\n";
        let postreq = b"\r\nPOST /rpc/Slide.GetInfo HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n";

        socket.work();
        println!("Opening socket connection");

        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Addr::new(192, 168, 68, 104)), 80)
            .unwrap();
        for _i in 1..=2 {
            green_led.set_low();
            socket.work();

            println!(
                "Sending request: {}",
                core::str::from_utf8(postreq).unwrap()
            );
            socket.work();
            match socket.write(postreq) {
                Ok(len) => println!("Wrote {len} bytes"),
                Err(e) => {
                    println!("Error on write {e:?}");
                    break;
                }
            }
            socket.flush().unwrap();
            socket.work();

            let deadline = time::Instant::now() + Duration::from_secs(20);
            loop {
                socket.work();
                let mut buffer = [0u8; 1024];
                if let Ok(len) = socket.read(&mut buffer) {
                    println!("\n------------ len is {len}  ------------");
                    print!("{}", core::str::from_utf8(&buffer[..len]).unwrap()); // there might be more data, continue reading
                    break;
                };
                if time::Instant::now() > deadline {
                    println!("Timeout");
                    break;
                }
            }
            println!("Bye1 socket is still open {}", socket.is_open());
            //        sta_socket.disconnect();
            delay.delay_millis(3000u32);
            green_led.set_high();
            delay.delay_millis(500u32);
        }
        println!("Done\n");
        socket.close();
        socket.disconnect();
        let _ = controller.disconnect();
        let _ = controller.stop();
    } //drop wifi and associated resources
    green_led.set_low();
    red_led.set_low();
    println!("sleep/delay starts now");
    delay.delay_millis(200u32);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(15));
    rtc.sleep_deep(&[&timer]);
}

// some smoltcp boilerplate
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
