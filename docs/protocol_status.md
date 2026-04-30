# Protocol Status

This document maps the current mower firmware against the project protocol notes.

## Current Mower-Side Behavior

- Wi-Fi station connects to the BST-managed network using the credentials in `src/app_config.rs`.
- Mower opens a TCP connection to the BST registration port.
- Mower sends a `MowerConfigPacket`.
- BST responds with an assigned UDP control port as ASCII.
- Mower binds that UDP port locally.
- Mower:
  - receives UDP control packets,
  - decodes control metadata,
  - ACKs each command with an `AckPacket`,
  - sends fake GPS telemetry once per second.

## Packet Coverage

- `Configuration Packet`: implemented
- `GPS & Telemetry Packet`: implemented
- `Mower Control Packet`: implemented for decode + ACK path
- `Course Correction Packet`: not implemented
- `Vision Frame Packet`: not implemented
- `Keep Alive Packet`: not implemented

## Command Coverage

The documented command IDs are now defined in Rust:

- `MOVE` = `0x0001`
- `TURN` = `0x0002`
- `STOP` = `0x0003`
- `PAUSE` = `0x0004`
- `RESUME` = `0x0005`
- `SET_SPEED` = `0x0006`
- `BLADES` = `0x0007`

Current runtime behavior only decodes and logs these commands, then sends an ACK.
No physical motion execution is implemented in this repo yet.

## Intentional Wire-Compatibility Notes

The code keeps the current wire format compatible with the existing BST implementation, even where it differs from the design notes:

- `MowerConfigPacket.checksum` is currently `u16`, not `u32`.
  - The design notes mention a 32-bit checksum.
  - The mower code preserves the current 16-bit field because that is what the existing BST-side code expects.

- `GpsTelemetryPacket.gps_fix_status` is currently numeric (`0` = void, `1` = valid).
  - The design notes describe RMC-style `A` / `V`.
  - The current wire format keeps a compact numeric status.

- `GpsTelemetryPacket.heading` is the current field name in code.
  - The design notes also discuss “magnetic variation”.
  - The firmware currently transmits heading only.

## Known Spec Mismatches Outside Packet Layout

- The design notes mention an `ESP32-S3` mower CPU.
- This firmware currently targets `ESP32-C6`.

That mismatch should be resolved before hardware/software interface assumptions are treated as stable.

## Recommended Next Steps

- Replace fake GPS telemetry with parsed live GPS telemetry once the UART path is finalized.
- Decode command parameters by command type:
  - `MOVE(distance)`
  - `TURN(angle)`
  - `SET_SPEED(tier)`
  - `BLADES(on/off)`
- Add explicit keep-alive packets and link-loss behavior.
- Define BST/MWR command parameter endianness and units in one place.
