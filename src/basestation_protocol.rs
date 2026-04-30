//! Packet definitions matching the current Pi base-station code.
//!
//! The C++ side sends and receives raw `struct` bytes with native little-endian
//! field encoding. This is not a robust wire format, but the ESP side needs to
//! match it exactly for interoperability with the current implementation.

#[repr(u16)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PacketType {
    Ack = 0x0000,
    Config = 0x0001,
    GpsTelemetry = 0x0002,
    Command = 0x0003,
}

#[repr(u16)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MowerCommandId {
    Move = 0x0001,
    Turn = 0x0002,
    Stop = 0x0003,
    Pause = 0x0004,
    Resume = 0x0005,
    SetSpeed = 0x0006,
    Blades = 0x0007,
}

impl MowerCommandId {
    pub fn from_u16(value: u16) -> Option<Self> {
        match value {
            0x0001 => Some(Self::Move),
            0x0002 => Some(Self::Turn),
            0x0003 => Some(Self::Stop),
            0x0004 => Some(Self::Pause),
            0x0005 => Some(Self::Resume),
            0x0006 => Some(Self::SetSpeed),
            0x0007 => Some(Self::Blades),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GpsFixStatus {
    Void = b'V' as isize,
    Valid = b'A' as isize,
}

impl GpsFixStatus {
    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DecodedMowerCommand<'a> {
    pub packet_seq_num: u16,
    pub mower_id: u16,
    pub utc_timestamp: u16,
    pub ack_request_flag: u8,
    pub command_id: Option<MowerCommandId>,
    pub raw_command_id: u16,
    pub control_parameter: &'a [u8],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MowerConfigPacket {
    pub packet_type: u16,
    pub packet_seq_num: u16,
    pub mower_id: u16,
    pub utc_timestamp: u16,
    pub mower_ip_address: u32,
    pub mower_port: u16,
    pub fault_flags: u16,
    pub reserved1: u32,
    pub reserved2: u32,
    // The current Pi-side implementation still uses a 16-bit checksum here.
    // The design notes mention a 32-bit checksum, but changing this now would
    // break wire compatibility with the existing base station.
    pub checksum: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AckPacket {
    pub packet_type: u16,
    pub orig_packet_seq_num: u16,
    pub mower_id: u16,
    pub utc_timestamp: u16,
    pub checksum: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsTelemetryPacket {
    pub packet_type: u16,
    pub packet_seq_num: u16,
    pub mower_id: u16,
    pub utc_timestamp: u16,
    pub utc_of_gps_fix: u16,
    // The BST currently uses a char field carrying RMC-style 'A' / 'V'.
    pub gps_fix_status: u8,
    pub latitude: f32,
    pub lat_hemisphere: u8,
    pub longitude: f32,
    pub lon_hemisphere: u8,
    pub speed_in_knots: f32,
    pub heading: f32,
    pub checksum: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MowerCommandPacket {
    pub packet_type: u16,
    pub packet_seq_num: u16,
    pub mower_id: u16,
    pub utc_timestamp: u16,
    pub ack_request_flag: u8,
    pub reserved: [u8; 3],
    pub control_command: u16,
    pub control_param_length: u16,
    pub control_parameter: [u8; 256],
    pub checksum: u32,
}

impl Default for MowerCommandPacket {
    fn default() -> Self {
        Self {
            packet_type: PacketType::Command as u16,
            packet_seq_num: 0,
            mower_id: 0,
            utc_timestamp: 0,
            ack_request_flag: 0,
            reserved: [0; 3],
            control_command: 0,
            control_param_length: 0,
            control_parameter: [0; 256],
            checksum: 0,
        }
    }
}

impl MowerConfigPacket {
    pub fn new(
        packet_seq_num: u16,
        mower_id: u16,
        utc_timestamp: u16,
        mower_ip_address: u32,
        mower_port: u16,
        fault_flags: u16,
    ) -> Self {
        let mut packet = Self {
            packet_type: PacketType::Config as u16,
            packet_seq_num,
            mower_id,
            utc_timestamp,
            mower_ip_address,
            mower_port,
            fault_flags,
            reserved1: 0,
            reserved2: 0,
            checksum: 0,
        };
        packet.checksum = checksum16(packet.bytes_without_checksum());
        packet
    }

    pub fn as_bytes(&self) -> &[u8] {
        as_bytes(self)
    }

    fn bytes_without_checksum(&self) -> &[u8] {
        prefix_bytes(
            self,
            core::mem::size_of::<Self>() - core::mem::size_of::<u16>(),
        )
    }
}

impl AckPacket {
    pub fn new(orig_packet_seq_num: u16, mower_id: u16, utc_timestamp: u16) -> Self {
        let mut packet = Self {
            packet_type: PacketType::Ack as u16,
            orig_packet_seq_num,
            mower_id,
            utc_timestamp,
            checksum: 0,
        };
        packet.checksum = checksum16(packet.bytes_without_checksum());
        packet
    }

    pub fn as_bytes(&self) -> &[u8] {
        as_bytes(self)
    }

    fn bytes_without_checksum(&self) -> &[u8] {
        prefix_bytes(
            self,
            core::mem::size_of::<Self>() - core::mem::size_of::<u16>(),
        )
    }
}

impl GpsTelemetryPacket {
    #[expect(clippy::too_many_arguments, reason = "Matches existing wire format")]
    pub fn new(
        packet_seq_num: u16,
        mower_id: u16,
        utc_timestamp: u16,
        utc_of_gps_fix: u16,
        gps_fix_status: u8,
        latitude: f32,
        lat_hemisphere: u8,
        longitude: f32,
        lon_hemisphere: u8,
        speed_in_knots: f32,
        heading: f32,
    ) -> Self {
        let mut packet = Self {
            packet_type: PacketType::GpsTelemetry as u16,
            packet_seq_num,
            mower_id,
            utc_timestamp,
            utc_of_gps_fix,
            gps_fix_status,
            latitude,
            lat_hemisphere,
            longitude,
            lon_hemisphere,
            speed_in_knots,
            heading,
            checksum: 0,
        };
        // The C++ code stores a 16-bit checksum in a 32-bit field.
        packet.checksum = checksum16(packet.bytes_without_checksum()) as u32;
        packet
    }

    pub fn as_bytes(&self) -> &[u8] {
        as_bytes(self)
    }

    fn bytes_without_checksum(&self) -> &[u8] {
        prefix_bytes(
            self,
            core::mem::size_of::<Self>() - core::mem::size_of::<u32>(),
        )
    }
}

impl MowerCommandPacket {
    pub const PACKET_LEN: usize = core::mem::size_of::<Self>();
    // With ack_request_flag + reserved[3], the BST currently lays out:
    // control_command at byte 12, control_param_length at byte 14, and the
    // variable parameter payload at byte 16.
    const HEADER_LEN: usize = 16;
    const PARAM_OFFSET: usize = 16;
    const PARAM_CAPACITY: usize = 256;

    pub fn decode(bytes: &[u8]) -> Option<DecodedMowerCommand<'_>> {
        if bytes.len() < Self::HEADER_LEN {
            return None;
        }

        let raw_command_id = read_u16_le(bytes, 12)?;
        let param_len = read_u16_le(bytes, 14)? as usize;
        let available_param_bytes = bytes.len().saturating_sub(Self::PARAM_OFFSET);
        let param_len = core::cmp::min(
            core::cmp::min(param_len, Self::PARAM_CAPACITY),
            available_param_bytes,
        );
        let param_end = Self::PARAM_OFFSET + param_len;

        Some(DecodedMowerCommand {
            packet_seq_num: read_u16_le(bytes, 2)?,
            mower_id: read_u16_le(bytes, 4)?,
            utc_timestamp: read_u16_le(bytes, 6)?,
            ack_request_flag: *bytes.get(8)?,
            command_id: MowerCommandId::from_u16(raw_command_id),
            raw_command_id,
            control_parameter: bytes.get(Self::PARAM_OFFSET..param_end)?,
        })
    }
}

pub fn checksum16(data: &[u8]) -> u16 {
    let mut sum = 0_u32;
    for &byte in data {
        sum += byte as u32;
    }
    (sum & 0xFFFF) as u16
}

fn as_bytes<T>(value: &T) -> &[u8] {
    unsafe {
        core::slice::from_raw_parts((value as *const T).cast::<u8>(), core::mem::size_of::<T>())
    }
}

fn prefix_bytes<T>(value: &T, len: usize) -> &[u8] {
    unsafe { core::slice::from_raw_parts((value as *const T).cast::<u8>(), len) }
}

fn read_u16_le(bytes: &[u8], offset: usize) -> Option<u16> {
    let lo = *bytes.get(offset)?;
    let hi = *bytes.get(offset + 1)?;
    Some(u16::from_le_bytes([lo, hi]))
}
