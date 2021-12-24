#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// GPIO traits
use embedded_hal::PwmPin;

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

use pico::hal::gpio::{FunctionPio0, Pin};
use pico::hal::pio::PIOExt;
//use pio_proc::pio;

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
    let pwm1 = &mut pwm_slices.pwm1;
    pwm1.set_ph_correct();
    pwm1.enable();
    pwm1.set_top(24); // 24
    pwm1.set_div_int(125); // 125

    // Output channel B on PWM1 to the LED pin
    let channel_b = &mut pwm1.channel_b;
    channel_b.output_to(pins.gpio3);

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
    channel_b.set_duty(0);

    let _dat: Pin<_, FunctionPio0> = pins.gpio17.into_mode();
    let _clk: Pin<_, FunctionPio0> = pins.gpio16.into_mode();

    let side_set = pio::SideSet::new(true, 1, false);
    let mut a = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut jump = a.label();
    
    a.set(pio::SetDestination::X, 31);
    a.bind(&mut jump);
    a.nop_with_side_set(0);
    a.nop();
    a.nop();
    a.bind(&mut wrap_target);
    a.nop_with_side_set(1);
    a.r#in(pio::InSource::PINS, 1);
    a.jmp(pio::JmpCondition::XDecNonZero, &mut jump);
    a.push_with_side_set(true, false, 0);
    a.set(pio::SetDestination::X, 31);
    a.nop();
    a.bind(&mut wrap_source);
    
    let program = a.assemble_with_wrap(wrap_source, wrap_target);
    
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let div = 20.83333;
    
    let (sm, mut rx, _) = hal::pio::PIOBuilder::from_program(installed)
        .clock_divisor(div)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .in_pin_base(17)
        .side_set_pin_base(16)
        .build(sm0);
    sm.start();
    
    let mut flg = false;
    loop {
        if flg {
            channel_b.set_duty(0);
            flg = false;
        } else {
            channel_b.set_duty(0x8000);
            flg = true;
        }
        for i in 0..600 {
            delay.delay_ms(5);
            let _ = usb_dev.poll(&mut [&mut serial]);
            if (i + 1) % 20 == 0 {
                match rx.read() {
                    Some(val) => {
                        let s = val.numtoa(10, &mut buf);
                        let _ = serial.write(s);
                        let _ = serial.write(b"\r\n");
                    }
                    None => {
                        let _ = serial.write(b"None\r\n");
                    }
                };
            }
        }
    }
}

// End of file