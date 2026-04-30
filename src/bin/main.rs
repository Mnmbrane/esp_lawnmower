#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, Ipv4Address, Runner, Stack, StackResources};
use embassy_time::{Duration, Instant as EmbassyInstant, Timer};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::time::Instant;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::Interface as WifiInterface;
use esp_radio::wifi::{self, AuthenticationMethod, Config, sta::StationConfig};
use lawnmower_esp_rs::app_config::APP_CONFIG;
use lawnmower_esp_rs::basestation_protocol::{
    AckPacket, DecodedMowerCommand, GpsFixStatus, GpsTelemetryPacket, MowerCommandId,
    MowerCommandPacket, MowerConfigPacket,
};
use log::info;
use static_cell::StaticCell;

extern crate alloc;
use alloc::string::String;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_INTERFACE: StaticCell<WifiInterface<'static>> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

#[derive(Clone, Copy, Debug)]
enum RegistrationError {
    TcpConnectFailed,
    TcpWriteFailed,
    TcpReadFailed,
    EmptyResponse,
    InvalidResponse,
}

#[derive(Clone, Copy, Debug)]
struct MotionCommand {
    packet_seq_num: u16,
    command_id: Option<lawnmower_esp_rs::basestation_protocol::MowerCommandId>,
    raw_command_id: u16,
    ack_requested: bool,
    param_len: usize,
    param_bytes: [u8; 8],
}

#[derive(Clone, Copy, Debug)]
struct MowerPose {
    lat: f32,
    lon: f32,
    heading_deg: f32,
    speed_mps: f32,
    remaining_distance_m: f32,
    motion_state: MotionState,
    telemetry_enabled: bool,
}

impl MowerPose {
    const fn new() -> Self {
        Self {
            lat: 37.775299,
            lon: -122.419800,
            heading_deg: 0.0,
            speed_mps: 0.5,
            remaining_distance_m: 0.0,
            motion_state: MotionState::Idle,
            telemetry_enabled: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotionState {
    Idle,
    MovingForward,
    Paused,
    Stopped,
}

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
        let assigned_ctrl_port = match tcp_register_with_basestation(stack, remote, local_ip).await
        {
            Ok(port) => port,
            Err(err) => {
                info!("Base station registration failed: {:?}. Retrying.", err);
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        if let Err(err) = run_udp_session(stack, remote.0, local_ip, assigned_ctrl_port).await {
            info!("UDP session ended: {:?}. Re-registering.", err);
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn tcp_register_with_basestation(
    stack: Stack<'static>,
    remote: (Ipv4Address, u16),
    local_ip: Ipv4Address,
) -> Result<u16, RegistrationError> {
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
        .map_err(|_| RegistrationError::TcpConnectFailed)?;
    info!(
        "Connected to base station {}:{}.",
        APP_CONFIG.base_station.ip, APP_CONFIG.base_station.registration_port
    );

    let cfg_packet =
        MowerConfigPacket::new(1, 0, unix_time_u16(), ipv4_to_wire_u32(local_ip), 0, 0);
    socket
        .write_all(cfg_packet.as_bytes())
        .await
        .map_err(|_| RegistrationError::TcpWriteFailed)?;
    info!("Sent configuration packet to base station.");

    let mut response = [0_u8; 64];
    let bytes_read = socket
        .read(&mut response)
        .await
        .map_err(|_| RegistrationError::TcpReadFailed)?;
    if bytes_read == 0 {
        return Err(RegistrationError::EmptyResponse);
    }

    parse_port_ascii(&response[..bytes_read]).ok_or(RegistrationError::InvalidResponse)
}

#[derive(Clone, Copy, Debug)]
enum UdpSessionError {
    BindFailed,
    ReceiveFailed,
    AckSendFailed,
    TelemetrySendFailed,
}

async fn run_udp_session(
    stack: Stack<'static>,
    server_ip: Ipv4Address,
    local_ip: Ipv4Address,
    assigned_ctrl_port: u16,
) -> Result<(), UdpSessionError> {
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

    socket
        .bind(assigned_ctrl_port)
        .map_err(|_| UdpSessionError::BindFailed)?;

    info!(
        "UDP session established on {}:{}; expecting BST traffic from {}:{}.",
        local_ip, assigned_ctrl_port, server_ip, assigned_ctrl_port
    );

    let server_endpoint = (server_ip, assigned_ctrl_port);
    let mut pose = MowerPose::new();
    let mut telemetry_seq_num = 1_u16;
    let telemetry_interval = Duration::from_secs(1);
    let mut next_telemetry_at = EmbassyInstant::now() + telemetry_interval;
    let poll_interval = Duration::from_millis(100);

    loop {
        let mut cmd_buffer = [0_u8; 1024];
        let now = EmbassyInstant::now();
        let time_until_tick = next_telemetry_at
            .checked_duration_since(now)
            .unwrap_or(Duration::from_millis(0));
        let wait_duration = if time_until_tick < poll_interval {
            time_until_tick
        } else {
            poll_interval
        };
        match select(
            socket.recv_from(&mut cmd_buffer),
            Timer::after(wait_duration),
        )
        .await
        {
            Either::First(recv_result) => {
                let (bytes_read, _endpoint) =
                    recv_result.map_err(|_| UdpSessionError::ReceiveFailed)?;
                info!("UDP RX bytes={}", bytes_read);
                log_hexdump("UDP RX", &cmd_buffer[..bytes_read], 32);

                let orig_seq_num = parse_u16_le(&cmd_buffer, 2).unwrap_or(0);
                let mut should_ack = false;
                if let Some(command) = MowerCommandPacket::decode(&cmd_buffer[..bytes_read]) {
                    log_command(command, bytes_read);
                    should_ack = command.ack_request_flag != 0;
                    let motion_command = MotionCommand {
                        packet_seq_num: command.packet_seq_num,
                        command_id: command.command_id,
                        raw_command_id: command.raw_command_id,
                        ack_requested: command.ack_request_flag != 0,
                        param_len: command.control_parameter.len(),
                        param_bytes: first_param_bytes(command.control_parameter),
                    };
                    apply_motion_command(&mut pose, motion_command);
                } else {
                    info!(
                        "Received UDP command packet: bytes={} seq={} decode=failed.",
                        bytes_read, orig_seq_num
                    );
                }

                if should_ack {
                    let ack = AckPacket::new(
                        orig_seq_num,
                        APP_CONFIG.base_station.initial_mower_id,
                        unix_time_u16(),
                    );
                    socket
                        .send_to(ack.as_bytes(), server_endpoint)
                        .await
                        .map_err(|_| UdpSessionError::AckSendFailed)?;
                    info!("Sent UDP ACK for seq {}.", orig_seq_num);
                } else {
                    info!("UDP ACK not requested for seq {}.", orig_seq_num);
                }
            }
            Either::Second(_) => {}
        }

        while EmbassyInstant::now() >= next_telemetry_at {
            let maybe_pose = if !pose.telemetry_enabled {
                None
            } else {
                if pose.motion_state == MotionState::MovingForward {
                    if pose.remaining_distance_m > 0.0 {
                        let distance_m = if pose.speed_mps < pose.remaining_distance_m {
                            pose.speed_mps
                        } else {
                            pose.remaining_distance_m
                        };
                        advance_pose(&mut pose, distance_m);
                        pose.remaining_distance_m -= distance_m;
                        if pose.remaining_distance_m <= 0.0 {
                            pose.remaining_distance_m = 0.0;
                            pose.motion_state = MotionState::Idle;
                        }
                    } else {
                        pose.motion_state = MotionState::Idle;
                    }
                }
                Some(pose)
            };

            if let Some(pose) = maybe_pose {
                info!(
                    "Telemetry tick: seq={} lat={} lon={} heading={} state={:?} speed_mps={} remaining_distance_m={}.",
                    telemetry_seq_num,
                    pose.lat,
                    pose.lon,
                    pose.heading_deg,
                    pose.motion_state,
                    pose.speed_mps,
                    pose.remaining_distance_m,
                );
                let telemetry = fake_gps_telemetry(telemetry_seq_num, pose);
                socket
                    .send_to(telemetry.as_bytes(), server_endpoint)
                    .await
                    .map_err(|_| UdpSessionError::TelemetrySendFailed)?;
                info!("Sent telemetry packet seq={}.", telemetry.packet_seq_num);
                telemetry_seq_num = telemetry_seq_num.wrapping_add(1);
            }

            next_telemetry_at = next_telemetry_at + telemetry_interval;
        }
    }
}

fn apply_motion_command(pose: &mut MowerPose, command: MotionCommand) {
    info!(
        "Applying motion command: seq={} cmd={:?}/0x{:04x} ack_req={} param_len={}.",
        command.packet_seq_num,
        command.command_id,
        command.raw_command_id,
        command.ack_requested,
        command.param_len,
    );

    match command.command_id {
        Some(MowerCommandId::Move) => {
            if let Some(distance_m) = decode_f32_param(command) {
                pose.remaining_distance_m = if distance_m.is_finite() && distance_m > 0.0 {
                    distance_m
                } else {
                    0.0
                };
            }
            pose.motion_state = MotionState::MovingForward;
            pose.telemetry_enabled = true;
            info!(
                "MOVE command applied: seq={} distance_m={} lat={} lon={} heading={} state={:?} speed_mps={}.",
                command.packet_seq_num,
                pose.remaining_distance_m,
                pose.lat,
                pose.lon,
                pose.heading_deg,
                pose.motion_state,
                pose.speed_mps,
            );
        }
        Some(MowerCommandId::Turn) => {
            if let Some(angle_deg) = decode_f32_param(command) {
                pose.heading_deg = normalize_heading(pose.heading_deg + angle_deg);
                info!(
                    "TURN command applied: seq={} angle_deg={} -> lat={} lon={} heading={} state={:?}.",
                    command.packet_seq_num,
                    angle_deg,
                    pose.lat,
                    pose.lon,
                    pose.heading_deg,
                    pose.motion_state,
                );
            } else {
                info!(
                    "TURN command failed to decode: seq={} ack_req={} param_len={}.",
                    command.packet_seq_num, command.ack_requested, command.param_len,
                );
            }
        }
        Some(MowerCommandId::Pause) => {
            pose.motion_state = MotionState::Paused;
            info!(
                "PAUSE command applied: seq={} remaining_distance_m={} lat={} lon={} heading={} state={:?}.",
                command.packet_seq_num,
                pose.remaining_distance_m,
                pose.lat,
                pose.lon,
                pose.heading_deg,
                pose.motion_state,
            );
        }
        Some(MowerCommandId::Resume) => {
            pose.motion_state = MotionState::MovingForward;
            info!(
                "RESUME command applied: seq={} remaining_distance_m={} lat={} lon={} heading={} state={:?} speed_mps={}.",
                command.packet_seq_num,
                pose.remaining_distance_m,
                pose.lat,
                pose.lon,
                pose.heading_deg,
                pose.motion_state,
                pose.speed_mps,
            );
        }
        Some(MowerCommandId::Stop) => {
            pose.motion_state = MotionState::Stopped;
            pose.remaining_distance_m = 0.0;
            info!(
                "STOP command applied: seq={} ack_req={} lat={} lon={} heading={} state={:?}.",
                command.packet_seq_num,
                command.ack_requested,
                pose.lat,
                pose.lon,
                pose.heading_deg,
                pose.motion_state,
            );
        }
        _ => {
            info!(
                "Unhandled motion command: seq={} cmd={:?}/0x{:04x} ack_req={} param_len={}.",
                command.packet_seq_num,
                command.command_id,
                command.raw_command_id,
                command.ack_requested,
                command.param_len,
            );
        }
    }
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
    // The BST reads this IP from a raw little-endian C struct. To make the
    // packet bytes appear as the normal dotted-quad order, build the integer
    // from little-endian bytes on this little-endian target.
    u32::from_le_bytes(ip.octets())
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

fn fake_gps_telemetry(packet_seq_num: u16, pose: MowerPose) -> GpsTelemetryPacket {
    let timestamp = unix_time_u16();

    GpsTelemetryPacket::new(
        packet_seq_num,
        APP_CONFIG.base_station.initial_mower_id,
        timestamp,
        timestamp,
        GpsFixStatus::Valid.as_u8(),
        pose.lat,
        if pose.lat >= 0.0 { b'N' } else { b'S' },
        pose.lon,
        if pose.lon >= 0.0 { b'E' } else { b'W' },
        if pose.motion_state == MotionState::MovingForward {
            pose.speed_mps * 1.94384
        } else {
            0.0
        },
        pose.heading_deg,
    )
}

fn log_command(command: DecodedMowerCommand<'_>, bytes_read: usize) {
    info!(
        "Received UDP command: bytes={} seq={} mower_id={} utc={} ack_req={} cmd={:?}/0x{:04x} param_len={}.",
        bytes_read,
        command.packet_seq_num,
        command.mower_id,
        command.utc_timestamp,
        command.ack_request_flag,
        command.command_id,
        command.raw_command_id,
        command.control_parameter.len(),
    );
}

fn log_hexdump(label: &str, bytes: &[u8], max_len: usize) {
    let dump_len = core::cmp::min(bytes.len(), max_len);
    let mut offset = 0_usize;

    while offset < dump_len {
        let end = core::cmp::min(offset + 16, dump_len);
        let chunk = &bytes[offset..end];
        info!("{} {:04x}: {}", label, offset, HexLine { chunk });
        offset = end;
    }

    if bytes.len() > dump_len {
        info!(
            "{} ....: truncated {} of {} bytes",
            label,
            dump_len,
            bytes.len()
        );
    }
}

struct HexLine<'a> {
    chunk: &'a [u8],
}

impl core::fmt::Display for HexLine<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for index in 0..16 {
            if index < self.chunk.len() {
                write!(f, "{:02x} ", self.chunk[index])?;
            } else {
                write!(f, "   ")?;
            }

            if index == 7 {
                write!(f, " ")?;
            }
        }

        write!(f, " |")?;
        for &byte in self.chunk {
            let ch = if byte.is_ascii_graphic() || byte == b' ' {
                byte as char
            } else {
                '.'
            };
            write!(f, "{}", ch)?;
        }
        write!(f, "|")
    }
}

fn decode_f32_param(command: MotionCommand) -> Option<f32> {
    if command.param_len < 6 {
        return None;
    }
    Some(f32::from_le_bytes([
        command.param_bytes[2],
        command.param_bytes[3],
        command.param_bytes[4],
        command.param_bytes[5],
    ]))
}

fn first_param_bytes(bytes: &[u8]) -> [u8; 8] {
    let mut out = [0_u8; 8];
    let copy_len = core::cmp::min(bytes.len(), out.len());
    out[..copy_len].copy_from_slice(&bytes[..copy_len]);
    out
}

fn normalize_heading(mut heading_deg: f32) -> f32 {
    if !heading_deg.is_finite() {
        return 0.0;
    }

    let normalized = libm::fmodf(heading_deg, 360.0);
    if normalized < 0.0 {
        normalized + 360.0
    } else {
        normalized
    }
}

fn advance_pose(pose: &mut MowerPose, distance_m: f32) {
    let heading_rad = pose.heading_deg.to_radians();
    let north_m = distance_m * libm::cosf(heading_rad);
    let east_m = distance_m * libm::sinf(heading_rad);

    pose.lat += north_m / 111_320.0;

    let lat_rad = pose.lat.to_radians();
    let meters_per_lon_degree = 111_320.0 * libm::cosf(lat_rad);
    if meters_per_lon_degree.abs() > 1.0 {
        pose.lon += east_m / meters_per_lon_degree;
    }
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

    info!("Embassy initialized!");
    info!(
        "Configured base station target: {}:{} mower_id={}.",
        APP_CONFIG.base_station.ip,
        APP_CONFIG.base_station.registration_port,
        APP_CONFIG.base_station.initial_mower_id
    );
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
