#![no_std]
#![no_main]

use fugit::RateExtU32;
use hal::pac;
use hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::Clock;
use rp_pico::entry;

use embedded_hal::digital::v2::OutputPin;

use panic_halt as _;
use rp2040_hal as hal;

use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;

use crate::pac::I2C0;
use crate::pac::UART0;
use rp2040_hal::gpio::bank0::*;
use rp2040_hal::gpio::*;
use rp2040_hal::uart::Enabled;
use rp2040_hal::uart::UartPeripheral;
use rp2040_hal::I2C;

const DEVICE_ADRS: u8 = 0x48;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        // (8-6-7)
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());

    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"i2c & uart_tx example started...\r\n");

    let mut led_pin = pins.led.into_push_pull_output();

    let mut i2c = hal::I2C::i2c0(
        // (1)
        pac.I2C0,
        pins.gpio16.into_function(), // sda
        pins.gpio17.into_function(), // scl
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
        delay.delay_ms(1000);
        read_temperature(&mut i2c, &mut uart); // (2)
    }
}

fn read_temperature(
    i2c: &mut I2C<
        I2C0,
        (
            Pin<Gpio16, FunctionI2c, PullDown>,
            Pin<Gpio17, FunctionI2c, PullDown>,
        ),
    >,
    uart: &mut UartPeripheral<
        Enabled,
        UART0,
        (
            Pin<Gpio0, FunctionUart, PullDown>,
            Pin<Gpio1, FunctionUart, PullDown>,
        ),
    >,
) {
    let mut readbuf: [u8; 2] = [0; 2]; // (3)
    i2c.write_read(DEVICE_ADRS, &[0], &mut readbuf).unwrap(); // (4)

    let mut temp: u16 = (readbuf[0] as u16) << 8; // (5)
    temp += readbuf[1] as u16; // (5)
    temp >>= 3; // (5)
    let mut f_temp = temp as f64 / 16.0; // (5)

    f_temp += 0.05; // (6)
    let i_temp = (f_temp * 10.0) as u16; // (7)

    let u10 = (i_temp / 100) as u16; // (8)
    let u1 = i_temp.wrapping_sub(u10 * 100) / 10; // (8)
    let u0_1 = i_temp.wrapping_sub(u10 * 100).wrapping_sub(u1 * 10); // (8)

    let mut buf = [0u8; 9]; // (9)

    buf[0] = u10 as u8 + b'0'; // 文字にする
    buf[1] = u1 as u8 + b'0';
    buf[2] = b'.';
    buf[3] = u0_1 as u8 + b'0';
    buf[4] = 0xe2; // e2 84 83 = ℃の文字コード(utf-8)
    buf[5] = 0x84;
    buf[6] = 0x83;
    buf[7] = 0x0d; // CR
    buf[8] = 0x0a; // LF

    uart.write_full_blocking(&buf);
}
