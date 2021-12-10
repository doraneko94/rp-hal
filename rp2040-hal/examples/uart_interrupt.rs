//! # UART Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection. In particular, UART reads happen under interrupt and data is
//! stored in a queue for the main thread to access later.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Our interrupt macro
use pac::interrupt;

/// The linker will place this boot block at the start of our program image. We
// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// This is the length of a UART receive data queue
const RX_QUEUE_SIZE: usize = 64;

/// This holds our received UART data
static RX_QUEUE: bbqueue::BBBuffer<RX_QUEUE_SIZE> = bbqueue::BBBuffer::new();

/// This is the handle the IRQ needs for adding data to the RX_QUEUE
static RX_QUEUE_PRODUCER: cortex_m::interrupt::Mutex<core::cell::RefCell<Option<bbqueue::Producer<'static, RX_QUEUE_SIZE>>>> = cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an inifinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
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

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart = hal::uart::UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    let (prod, mut cons) = RX_QUEUE.try_split().unwrap();

    cortex_m::interrupt::free(|cs| {
        RX_QUEUE_PRODUCER.borrow(cs).replace(Some(prod));
    });

    uart.enable_rx_interrupt();

    // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
    let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
    // UART RX (characters reveived by RP2040) on pin 2 (GPIO1)
    let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

    uart.write_full_blocking(b"UART interrupt echo started!\r\n");

    loop {
        cortex_m::asm::wfi();
        while let Ok(mut grant) = cons.read() {
            let buf = grant.buf();
            let count = buf.len();
            uart.write_full_blocking(buf);
            drop(buf);
            grant.to_release(count);
        }
    }
}

#[interrupt]
fn UART0_IRQ() {
    cortex_m::interrupt::free(|cs| {
        if let Ok(prod) = RX_QUEUE_PRODUCER.borrow(cs).try_borrow_mut() {
            if let Ok(mut grant) = prod.grant_max_remaining(64) {
                let mut count = 0;
                for slot in grant.buf().iter_mut() {
                    let fifo_status: u32 = core::ptr::read_volatile(0x4003_4018);
                    if (fifo_status & (1 << 4)) != 0 {
                        // FIFO is empty
                        break;
                    }
                    let data: u32 = core::ptr::read_volatile(0x4003_4000);
                    let byte: u8 = data & 0xFF;
                    *slot = data;
                    count += 1;
                }
                grant.commit(count);
            } else {
                // TODO: Disable interrupt as the queue is full
            }

        }
    });
    cortex_m::asm::sev();
}

// End of file
