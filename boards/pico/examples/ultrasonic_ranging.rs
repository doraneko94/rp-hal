#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// GPIO traits
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::{InputPin, OutputPin};

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Convert a number to a string
use numtoa::NumToA;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM1
    let pwm = &mut pwm_slices.pwm1;
    pwm.set_ph_correct();
    pwm.enable();
    pwm.set_top(24999);
    pwm.set_div_int(100);
    pwm.set_div_frac(0);

    // Output channel B on PWM1 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio3);

    // Set an input from a switch to gpio5
    let switch = pins.gpio5.into_pull_up_input();

    // Set an input from a ultrasonic ranging sensor (echo) to gpio16
    let echo = pins.gpio16.into_pull_down_input();

    // Set an output to a ultrasonic ranging sensor (trigger) from gpio17
    let mut trigger = pins.gpio17.into_push_pull_output();

    // Set the USB bus
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set the serial port
    let mut serial = SerialPort::new(&usb_bus);

    // Set a USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    // Create a timer
    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    // Buffer fo NumToA
    let mut buf = [0u8; 20];
    // Switch on/off
    let mut switch_flg = false;
    channel.set_duty(0);
    loop {
        delay.delay_ms(5);
        let _ = usb_dev.poll(&mut [&mut serial]);
        
        // Switch on
        if switch.is_low().ok().unwrap() {
            if switch_flg {
                continue;
            } else {
                // Trigger ultrasonic pulse
                trigger.set_low().ok().unwrap();
                delay.delay_us(2);
                trigger.set_high().ok().unwrap();
                delay.delay_us(10);
                trigger.set_low().ok().unwrap();

                // Measure the time it took for the pulse to come back
                let mut time_low = 0;
                let mut time_high = 0;
                while echo.is_low().ok().unwrap() {
                    time_low = timer.get_counter();
                }
                while echo.is_high().ok().unwrap() {
                    time_high = timer.get_counter();
                }
                let time = time_high - time_low;

                // Convert the time to the distance (cm)
                let distance = time as f64 / 1.25 * 0.0343 / 2.0;

                // Display the integer and decimal parts of the distance separately
                let int = distance as u16;
                let frac = ((distance - int as f64) * 100.0) as u16;
                let s_int = int.numtoa(10, &mut buf);
                let _ = serial.write(s_int);
                let _ = serial.write(b".");
                let s_frac = frac.numtoa(10, &mut buf);
                let _ = serial.write(s_frac);
                let _ = serial.write(b"cm\r\n");

                switch_flg = true;

                // Adjust the brightness of the LED according to the distance
                if distance > 100.0 {
                    channel.set_duty(1000);
                } else {
                    channel.set_duty((64535 as f64 * ((100.0 - distance) / 100.0)) as u16 + 1000);
                }
                
            }
        } else {
            switch_flg = false;
        }
    }
}

// End of file