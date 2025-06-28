use vcnl4040::{AmbientIntegrationTime, LedCurrent, Vcnl4040};
mod i2c;
use i2cdev::linux::LinuxI2CBus;
use simple_logger::SimpleLogger;
use log::info;
use std::time::Duration;

#[tokio::main]
async fn main() {
    SimpleLogger::new().init().unwrap();
    let dev = i2c::LinuxI2c::new(
        LinuxI2CBus::new("/dev/i2c-2").unwrap()
    );

    let mut sensor = Vcnl4040::new(dev);
    sensor.init(true).await.unwrap();

    // Set some configuration
    sensor.set_proximity_led_current(LedCurrent::Current100mA).await.unwrap();
    sensor.set_ambient_integration_time(AmbientIntegrationTime::Time160ms).await.unwrap();

    loop {
        let proximity = sensor.get_proximity().await.unwrap();
        info!("Proximity reading is {}", proximity);
        let ambli = sensor.get_ambient_light().await.unwrap();
        info!("Ambient light reading is {}", ambli);
        let wili = sensor.get_white_light().await.unwrap();
        info!("White light reading is {}", wili);
        let lux = sensor.get_lux().await.unwrap();
        info!("Lux reading is {}", lux);
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}