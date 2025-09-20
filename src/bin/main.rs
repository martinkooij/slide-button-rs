//! Access point with station
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! Gets an ip address via DHCP, creates an open access-point with SSID
//! `esp-wifi` You can connect to it using a static IP in range 192.168.2.2 ..
//! 192.168.2.255, gateway 192.168.2.1 Open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the
//! WiFi has no internet connection, Chrome might not want to load the URL - you
//! can use a shell and try `curl` and `ping`

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/smoltcp esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_alloc as _;

const SSID: &str = core::env!("SSID");
const PASSWORD: &str = core::env!("PASSWORD");
const _SLIDE_IP: &str = "192.168.68.104";

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Level, Output, OutputConfig, Pull},
    main,
    rng::Rng,
    rtc_cntl::{sleep::TimerWakeupSource, wakeup_cause, Rtc},
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_wifi::{
    init,
    wifi::{ClientConfiguration, Configuration},
};

use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::IpAddress,
};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut red_led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let mut led6 = Output::new(
        peripherals.GPIO10,
        Level::Low,
        OutputConfig::default().with_pull(Pull::None),
    );
    led6.set_low();
    let wake_reason = wakeup_cause();
    match wake_reason {
        esp_hal::system::SleepSource::Timer => {
            println!("woke up from timer");
            led6.set_high();
        }
        _ => {
            println!("not a timer wakeup");
        }
    }

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = Rng::new(peripherals.RNG);

    let esp_wifi_ctrl = init(timg0.timer0, rng.clone()).unwrap();

    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let mut sta_device = interfaces.sta;
    let sta_interface = create_interface(&mut sta_device);
    let now = || time::Instant::now().duration_since_epoch().as_millis();

    let mut sta_socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut sta_socket_set = SocketSet::new(&mut sta_socket_set_entries[..]);
    sta_socket_set.add(smoltcp::socket::dhcpv4::Socket::new());
    let sta_stack = Stack::new(sta_interface, sta_device, sta_socket_set, now, rng.random());

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        bssid: Some([0x66, 0x32, 0xb1, 0x35, 0xe3, 0x1f]),
        channel: Some(5),
        ..Default::default()
    });

    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    let mut sta_rx_buffer = [0u8; 1536];
    let mut sta_tx_buffer = [0u8; 1536];

    let mut sta_socket = sta_stack.get_socket(&mut sta_rx_buffer, &mut sta_tx_buffer);

    loop {
        red_led.set_high();
        controller.start().unwrap();
        println!("is wifi started: {:?}", controller.is_started());

        println!("{:?}", controller.capabilities());

        println!("wifi_connect {:?}", controller.connect());

        // wait for STA getting an ip address
        println!("Wait to get an ip address");
        loop {
            sta_stack.work();

            if sta_stack.is_iface_up() {
                println!("got ip {:?}", sta_stack.get_ip_info());
                break;
            }
        }

        println!("Start  main work.");

        println!("Making HTTP request");

        // let postreq = b"\r\nPOST /rpc/Slide.SetPos HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nContent-Length: 11\r\n\r\n{\"pos\":0.8}\r\n";
        let postreq = b"\r\nPOST /rpc/Slide.GetInfo HTTP/1.1\r\nAccept: */*\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n";

        let delay = Delay::new();
        sta_socket.work();
        println!("Opening socket connection");
        sta_socket
            .open(IpAddress::Ipv4(Ipv4Addr::new(192, 168, 68, 104)), 80)
            .unwrap();
        for _i in 1..=2 {
            red_led.set_low();
            sta_socket.work();

            println!(
                "Sending request: {}",
                core::str::from_utf8(postreq).unwrap()
            );
            sta_socket.work();
            match sta_socket.write(postreq) {
                Ok(len) => println!("Wrote {len} bytes"),
                Err(e) => {
                    println!("Error on write {e:?}");
                    break;
                }
            }
            sta_socket.flush().unwrap();
            sta_socket.work();

            let deadline = time::Instant::now() + Duration::from_secs(20);
            loop {
                sta_socket.work();
                let mut buffer = [0u8; 1024];
                if let Ok(len) = sta_socket.read(&mut buffer) {
                    println!("\n------------ len is {len}  ------------");
                    print!("{}", core::str::from_utf8(&buffer[..len]).unwrap()); // there might be more data, continue reading
                    break;
                };
                if time::Instant::now() > deadline {
                    println!("Timeout");
                    break;
                }
            }
            println!("Bye1 socket is still open {}", sta_socket.is_open());
            //        sta_socket.disconnect();
            delay.delay_millis(3000u32);
            red_led.set_high();
            delay.delay_millis(500u32);
        }
        println!("Done\n");
        sta_socket.close();
        sta_socket.disconnect();
        let _ = controller.disconnect();
        let _ = controller.stop();

        red_led.set_low();
        led6.set_low();
        println!("sleep/delay");
        delay.delay_millis(200u32);
        let timer = TimerWakeupSource::new(core::time::Duration::from_secs(15));
        rtc.sleep_deep(&[&timer]);
    }
}

fn _parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").into_iter().enumerate() {
        result[idx] = u8::from_str_radix(octet, 10).unwrap();
    }
    result
}

// some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice) -> smoltcp::iface::Interface {
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
