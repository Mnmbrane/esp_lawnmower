#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, UartRx};
use lawnmower_esp_rs::app_config::APP_CONFIG;
use log::info;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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

    // TODO: Spawn some tasks
    let _ = spawner;

    let mut byte = [0_u8; 1];
    let mut line = [0_u8; 128];
    let mut line_len = 0_usize;

    loop {
        let read = gps_rx.read(&mut byte).unwrap();
        if read == 0 {
            continue;
        }

        match byte[0] {
            b'\r' => {}
            b'\n' => {
                if line_len > 0 {
                    if let Ok(sentence) = core::str::from_utf8(&line[..line_len]) {
                        info!("GPS {}", sentence);
                    }
                    line_len = 0;
                }
            }
            value => {
                if line_len < line.len() {
                    line[line_len] = value;
                    line_len += 1;
                } else {
                    // Drop oversized lines and resync on the next newline.
                    line_len = 0;
                }
            }
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
