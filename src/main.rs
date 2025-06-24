#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
// Use the logging macros provided by defmt.
use defmt::*;

use embassy_rp::adc::Channel;
use embassy_rp::adc::{Adc, AdcPin, Config as AdcConfig};
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::ADC;
use embassy_rp::{bind_interrupts, init, peripherals};

// Import interrupts definition module
mod irqs;

use embassy_time::Instant;

use embedded_graphics::Drawable;
use embedded_graphics::geometry::Size;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};

use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_rp::gpio::Level;
use embassy_rp::gpio::Output;
use embassy_rp::gpio::Pin;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embassy_time::Delay;
use mipidsi::models::ST7735s;
use mipidsi::options::{Orientation, Rotation};

use core::fmt::Write;
use heapless::String;

use embassy_rp::gpio::Input;

use core::fmt::Debug;

//pt fading
use embassy_rp::pwm::{Config as PwmConfig, Pwm};

#[embassy_executor::task]
async fn breathing_led_task(mut pwm: Pwm<'static>) {
    let mut config: PwmConfig = Default::default();
    config.top = 37000;

    loop {
        // Fade in
        for compare in (0..=config.top).step_by(1000) {
            config.compare_a = compare;
            pwm.set_config(&config);
            Timer::after(Duration::from_millis(100)).await;
        }

        // Fade out
        for compare in (0..=config.top).rev().step_by(1000) {
            config.compare_a = compare;
            pwm.set_config(&config);
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

//Meniu pentru selectare mod
fn draw_menu<D>(screen: &mut D, selected_mode: u8)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let style = MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE);
    let highlight = MonoTextStyle::new(&FONT_8X13, Rgb565::YELLOW);

    screen.clear(Rgb565::BLACK).ok();

    Text::new("Select mode:", Point::new(10, 10), style)
        .draw(screen)
        .unwrap();

    let exercise_style = if selected_mode == 0 { highlight } else { style };
    let relax_style = if selected_mode == 1 { highlight } else { style };

    Text::new("1. Exercise", Point::new(10, 30), exercise_style)
        .draw(screen)
        .unwrap();

    Text::new("2. Relaxing", Point::new(10, 50), relax_style)
        .draw(screen)
        .unwrap();
}

//Functie pt desenarea bpm-ului
fn draw_bpm<D>(screen: &mut D, bpm: u32, mode: u8)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let style = MonoTextStyle::new(&FONT_8X13, Rgb565::CYAN);

    // Șterge zona unde afișăm BPM-ul (opțional, ca să nu se suprapună textul vechi)
    let clear_area = Rectangle::new(Point::new(10, 80), Size::new(150, 20));
    screen.fill_solid(&clear_area, Rgb565::BLACK).unwrap();

    let mut msg: String<32> = String::new();
    core::write!(msg, "Mode {} BPM: {}", mode + 1, bpm).unwrap();
    Text::new(&msg, Point::new(10, 90), style)
        .draw(screen)
        .unwrap();
}

const THRESHOLD: u16 = 2600;
const BPM_WINDOW_SIZE: usize = 5;
const SAMPLE_INTERVAL_MS: u64 = 5; //mai mic
const WINDOW_SIZE: usize = 10;

use core::str;
use embassy_rp::adc::InterruptHandler;
bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    //Pt ecran
    let mut screen_config = embassy_rp::spi::Config::default();
    screen_config.frequency = 32_000_000u32;
    screen_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    screen_config.polarity = embassy_rp::spi::Polarity::IdleHigh;
    let screen_rst = Output::new(p.PIN_20, Level::Low);
    let screen_dc = Output::new(p.PIN_21, Level::Low);
    let screen_cs = Output::new(p.PIN_17, Level::High);

    let spi = Spi::new_blocking(
        p.SPI0,
        p.PIN_18, // SCK
        p.PIN_19, // MOSI
        p.PIN_16, // MISO not used
        screen_config,
    );

    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi));
    let display_spi = SpiDevice::new(&spi_bus, screen_cs);

    let di = SPIInterface::new(display_spi, screen_dc);
    let mut screen = mipidsi::Builder::new(ST7735s, di)
        .reset_pin(screen_rst)
        .orientation(Orientation::new().rotate(Rotation::Deg270))
        .init(&mut Delay)
        .unwrap();

    let style = MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE);

    screen.clear(Rgb565::BLACK).unwrap();

    //Pt button
    let button = Input::new(p.PIN_11, Pull::Up);
    let mut mode: u8 = 0;

    draw_menu(&mut screen, mode);

    //Pt ledul rosu
    let mut config: PwmConfig = Default::default();
    config.top = 37000;
    config.compare_a = 0; // start stins

    let pwm = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, config.clone());
    //Pt celelalte leduri
    let mut green = Output::new(p.PIN_3, Level::Low);
    let mut blue = Output::new(p.PIN_4, Level::Low);

    green.set_low();
    blue.set_low();

    // Creează ADC-ul
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    // //12 biti rezolutie
    let mut adc_pin = Channel::new_pin(p.PIN_26, Pull::None);

    let mut last_value: u16 = 0;
    let mut last_peak_time = Instant::now();
    let mut bpm = 0;

    //Fereastră pentru media mobilă
    let mut adc_window: [u16; WINDOW_SIZE] = [0; WINDOW_SIZE];
    let mut adc_index = 0;

    let mut bpm_window: [u32; BPM_WINDOW_SIZE] = [0; BPM_WINDOW_SIZE];
    let mut bpm_index = 0;

    let mut incepe = breathing_led_task(pwm);
    let _ = spawner.spawn(incepe);

    loop {
        if button.is_low() {
            Timer::after(Duration::from_millis(50)).await;
            if button.is_low() {
                mode = (mode + 1) % 2;
                draw_menu(&mut screen, mode);
                while button.is_low() {
                    Timer::after(Duration::from_millis(10)).await;
                }
            }
        }

        // Citește ADC
        let raw_value = adc.read(&mut adc_pin).await.unwrap();
        //info!("raw {}", raw_value);
        adc_window[adc_index] = raw_value;
        adc_index = (adc_index + 1) % WINDOW_SIZE;

        let sum: u32 = adc_window.iter().map(|&v| v as u32).sum();
        let value = (sum / WINDOW_SIZE as u32) as u16;

        if value >= THRESHOLD {
            let now = Instant::now();
            let dt_ms = (now - last_peak_time).as_millis();
            last_peak_time = now;

            if dt_ms > 400 && dt_ms < 2000 {
                bpm = (60_000 / dt_ms as u32) as u32;
                bpm_window[bpm_index] = bpm;
                bpm_index = (bpm_index + 1) % BPM_WINDOW_SIZE;

                let bpm_avg: u32 = bpm_window.iter().sum::<u32>() / BPM_WINDOW_SIZE as u32;
                draw_bpm(&mut screen, bpm_avg, mode);

                if (mode == 0 && bpm_avg >= 60 && bpm_avg <= 160)
                    || (mode == 1 && bpm_avg >= 50 && bpm_avg <= 100)
                {
                    green.set_high();
                } else {
                    green.set_low();
                }
            }
            Timer::after_millis(SAMPLE_INTERVAL_MS).await;
        }
    }
}
