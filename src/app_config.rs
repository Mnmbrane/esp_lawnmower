//! Central place for hard-coded application settings while the project is still
//! in bring-up.

pub struct GpsConfig {
    pub uart_baudrate: u32,
    pub rx_gpio: u8,
}

pub struct WifiConfig {
    pub ssid: &'static str,
    pub password: &'static str,
}

pub struct BaseStationConfig {
    pub ip: &'static str,
    pub registration_port: u16,
    pub initial_mower_id: u16,
}

pub struct AppConfig {
    pub gps: GpsConfig,
    pub wifi: WifiConfig,
    pub base_station: BaseStationConfig,
}

pub const APP_CONFIG: AppConfig = AppConfig {
    gps: GpsConfig {
        uart_baudrate: 9_600,
        rx_gpio: 17, // XIAO ESP32-C6 D7
    },
    wifi: WifiConfig {
        ssid: "replace-me",
        password: "replace-me",
    },
    base_station: BaseStationConfig {
        ip: "192.168.50.1",
        registration_port: 59_000,
        initial_mower_id: 1,
    },
};
