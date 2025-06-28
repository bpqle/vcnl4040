use vcnl4040::{AmbientIntegrationTime, LedCurrent, Vcnl4040};
use embedded_hal::i2c;
use i2cdev::linux::LinuxI2CBus;
use simple_logger::SimpleLogger;
use log::info;
use std::time::Duration;

#[tokio::main]
async fn main() {
    SimpleLogger::new().init().unwrap();
    //let args: CliArgs = argh::from_env();
    let dev = i2c::LinuxI2c::new(
        LinuxI2CBus::new("/dev/i2c-2").unwrap()
    );

    let mut sensor = Vcnl4040::new(dev);
    // sensor.set_range_timing(30, 30).await.unwrap();
    sensor.init(
        proximity=true,
        white_light=true,
        ambient_light=true,
        highres_proximity=true
    ).await.unwrap();

    sensor.enable_proximity(true).await.unwrap();
    sensor.enable_ambient_light(true).await.unwrap();

    // Set some configuration
    sensor.set_proximity_led_current(LedCurrent::Current100mA).await.unwrap();
    sensor.set_ambient_integration_time(AmbientIntegrationTime::Time160ms).await.unwrap();

    loop {
        let proximity = sensor.get_proximity().await.unwrap();
        info!("Proximity is {}", proximity);
        let ambli = sensor.get_ambient_light().await.unwrap();
        info!("Ambient light is {}", ambli);
        let wili = sensor.get_white_light().await.unwrap();
        info!("White light is {}", wili);
        let lux = sensor.get_lux().await.unwrap();
        info!("Lux is {}", lux);
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}