#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, Ipv4Address, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::time::Instant;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, UartRx};
use esp_radio::wifi::Interface as WifiInterface;
use esp_radio::wifi::{self, AuthenticationMethod, Config, sta::StationConfig};
use lawnmower_esp_rs::app_config::APP_CONFIG;
use lawnmower_esp_rs::basestation_protocol::{AckPacket, GpsTelemetryPacket, MowerConfigPacket};
use log::info;
use static_cell::StaticCell;

extern crate alloc;
use alloc::string::String;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_INTERFACE: StaticCell<WifiInterface<'static>> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, &'static mut WifiInterface<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn basestation_task(stack: Stack<'static>) -> ! {
    stack.wait_config_up().await;
    let config = stack.config_v4().unwrap();
    let local_ip = config.address.address();
    info!("DHCP address acquired: {}", local_ip);

    let remote = (
        parse_ipv4(APP_CONFIG.base_station.ip),
        APP_CONFIG.base_station.registration_port,
    );

    loop {
        let assigned_ctrl_port = match register_with_basestation(stack, remote, local_ip).await {
            Ok(port) => port,
            Err(err) => {
                info!("Base station registration failed: {}. Retrying.", err);
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        info!("Assigned control port from server: {}", assigned_ctrl_port);

        let mut rx_meta = [PacketMetadata::EMPTY; 4];
        let mut rx_buffer = [0_u8; 1024];
        let mut tx_meta = [PacketMetadata::EMPTY; 4];
        let mut tx_buffer = [0_u8; 1024];
        let mut socket = UdpSocket::new(
            stack,
            &mut rx_meta,
            &mut rx_buffer,
            &mut tx_meta,
            &mut tx_buffer,
        );

        if let Err(err) = socket.bind(assigned_ctrl_port) {
            info!(
                "Failed to bind UDP control socket on port {}: {:?}. Retrying.",
                assigned_ctrl_port, err
            );
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }

        info!(
            "Listening for UDP commands on {}:{}.",
            local_ip, assigned_ctrl_port
        );

        let server_endpoint = (remote.0, assigned_ctrl_port);
        let fake_gps = GpsTelemetryPacket::new(
            1,
            APP_CONFIG.base_station.initial_mower_id,
            unix_time_u16(),
            unix_time_u16(),
            1,
            42.3601,
            b'N',
            71.0589,
            b'W',
            0.0,
            0.0,
        );
        if let Err(err) = socket.send_to(fake_gps.as_bytes(), server_endpoint).await {
            info!("Failed to send fake GPS telemetry: {:?}.", err);
        } else {
            info!("Sent fake GPS telemetry packet to base station.");
        }

        loop {
            let mut cmd_buffer = [0_u8; 1024];
            let (bytes_read, _endpoint) = match socket.recv_from(&mut cmd_buffer).await {
                Ok(result) => result,
                Err(err) => {
                    info!("UDP command receive failed: {:?}. Re-registering.", err);
                    break;
                }
            };

            let orig_seq_num = parse_u16_le(&cmd_buffer, 2).unwrap_or(0);
            info!(
                "Received UDP command packet: bytes={} seq={}.",
                bytes_read, orig_seq_num
            );

            let ack = AckPacket::new(orig_seq_num, 0, unix_time_u16());
            if let Err(err) = socket.send_to(ack.as_bytes(), server_endpoint).await {
                info!("Failed to send UDP ACK: {:?}. Re-registering.", err);
                break;
            }

            info!("Sent UDP ACK for seq {}.", orig_seq_num);
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn register_with_basestation(
    stack: Stack<'static>,
    remote: (Ipv4Address, u16),
    local_ip: Ipv4Address,
) -> Result<u16, &'static str> {
    let mut rx_buffer = [0_u8; 1024];
    let mut tx_buffer = [0_u8; 1024];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

    info!(
        "Connecting to base station {}:{}.",
        APP_CONFIG.base_station.ip, APP_CONFIG.base_station.registration_port
    );

    socket
        .connect(remote)
        .await
        .map_err(|_| "tcp connect failed")?;
    info!(
        "Connected to base station {}:{}.",
        APP_CONFIG.base_station.ip, APP_CONFIG.base_station.registration_port
    );

    let cfg_packet =
        MowerConfigPacket::new(1, 0, unix_time_u16(), ipv4_to_wire_u32(local_ip), 0, 0);
    socket
        .write_all(cfg_packet.as_bytes())
        .await
        .map_err(|_| "tcp write failed")?;
    info!("Sent configuration packet to base station.");

    let mut response = [0_u8; 64];
    let bytes_read = socket
        .read(&mut response)
        .await
        .map_err(|_| "tcp read failed")?;
    if bytes_read == 0 {
        return Err("empty registration response");
    }

    parse_port_ascii(&response[..bytes_read]).ok_or("invalid registration response")
}

fn parse_ipv4(ip: &str) -> Ipv4Address {
    let mut octets = [0_u8; 4];
    for (index, part) in ip.split('.').enumerate() {
        if index >= 4 {
            break;
        }
        octets[index] = part.parse::<u8>().unwrap_or(0);
    }
    Ipv4Address::new(octets[0], octets[1], octets[2], octets[3])
}

fn ipv4_to_wire_u32(ip: Ipv4Address) -> u32 {
    u32::from_be_bytes(ip.octets())
}

fn parse_port_ascii(bytes: &[u8]) -> Option<u16> {
    let text = core::str::from_utf8(bytes)
        .ok()?
        .trim_matches(char::from(0))
        .trim();
    text.parse::<u16>().ok()
}

fn parse_u16_le(bytes: &[u8], offset: usize) -> Option<u16> {
    let lo = *bytes.get(offset)?;
    let hi = *bytes.get(offset + 1)?;
    Some(u16::from_le_bytes([lo, hi]))
}

fn unix_time_u16() -> u16 {
    (Instant::now().duration_since_epoch().as_secs() & 0xFFFF) as u16
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c6 -o esp32c6-wroom-1 -o alloc -o neovim -o unstable-hal -o embassy -o log -o esp-backtrace -o stable-x86_64-unknown-linux-gnu

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO4
    // - GPIO5
    // - GPIO8
    // - GPIO9
    // - GPIO15
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _ = peripherals.GPIO24;
    let _ = peripherals.GPIO25;
    let _ = peripherals.GPIO26;
    let _ = peripherals.GPIO27;
    let _ = peripherals.GPIO28;
    let _ = peripherals.GPIO29;
    let _ = peripherals.GPIO30;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let (mut wifi_controller, wifi_interfaces) =
        wifi::new(peripherals.WIFI, Default::default()).unwrap();
    let wifi_interface = WIFI_INTERFACE.init(wifi_interfaces.station);
    let net_seed = u64::from(APP_CONFIG.base_station.initial_mower_id) << 32
        | u64::from(APP_CONFIG.base_station.registration_port);
    let (net_stack, net_runner) = embassy_net::new(
        wifi_interface,
        NetConfig::dhcpv4(Default::default()),
        NET_RESOURCES.init(StackResources::new()),
        net_seed,
    );

    spawner.spawn(net_task(net_runner).unwrap());

    let wifi_config = APP_CONFIG.wifi;
    let auth_method = if wifi_config.password.is_empty() {
        AuthenticationMethod::None
    } else {
        AuthenticationMethod::Wpa2Personal
    };
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(wifi_config.ssid)
            .with_password(String::from(wifi_config.password))
            .with_auth_method(auth_method),
    );

    info!("Starting Wi-Fi station for SSID '{}'.", wifi_config.ssid);
    wifi_controller.set_config(&station_config).unwrap();
    loop {
        match wifi_controller.connect_async().await {
            Ok(_connected) => {
                info!("Wi-Fi connected to '{}'.", wifi_config.ssid);
                break;
            }
            Err(err) => {
                info!(
                    "Wi-Fi connect to '{}' failed: {:?}. Retrying in 2s.",
                    wifi_config.ssid, err
                );
                Timer::after(Duration::from_secs(2)).await;
            }
        }
    }
    spawner.spawn(basestation_task(net_stack).unwrap());

    // GT-U7 modules typically speak NMEA at 9600 baud over UART.
    // Keep the pin/baud in APP_CONFIG so hardware changes stay in one place.
    let mut gps_rx = UartRx::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(APP_CONFIG.gps.uart_baudrate),
    )
    .unwrap()
    .with_rx(peripherals.GPIO17);

    info!("Embassy initialized!");
    info!(
        "Reading GPS NMEA from UART1 on GPIO{} / D7 at {} baud.",
        APP_CONFIG.gps.rx_gpio, APP_CONFIG.gps.uart_baudrate
    );
    info!("Connect GT-U7 TX -> D7, GND -> GND, and power the module.");
    info!(
        "Configured base station target: {}:{} mower_id={}.",
        APP_CONFIG.base_station.ip,
        APP_CONFIG.base_station.registration_port,
        APP_CONFIG.base_station.initial_mower_id
    );
    loop {
        let _ = &mut gps_rx;
        Timer::after(Duration::from_secs(60)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
