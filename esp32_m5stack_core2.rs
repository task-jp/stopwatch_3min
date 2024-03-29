use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterfaceNoCS;
use dummy_pin::DummyPin;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_hal::digital::v2::OutputPin;
use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    i2c::I2C,
    i2s::{DataFormat, I2s, I2s0New, I2sWriteDma, NoMclk, PinsBclkWsDout, Standard},
    interrupt::{self, Priority},
    pdma::Dma,
    peripherals::{self, Peripherals, TIMG0},
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::{Timer, Timer0, TimerGroup},
    Delay, Rtc, IO,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use mipidsi::Display;
use shared_bus;
use slint::platform::WindowEvent;
pub use xtensa_lx_rt::entry;

use critical_section::Mutex;
static TIMER00: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static mut BEEP_COUNTER: u32 = 0;
static mut SYSTEM_COUNTER: u64 = 0;

const SINE: [i16; 64] = [
    0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
    32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
    15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787, -23169,
    -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30272,
    -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511, -6392, -3211,
];

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 100 * 1024; // 100KB, original for CoreS3 is 200KB
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(&mut HEAP as *mut u8, core::mem::size_of_val(&HEAP)) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
}

pub fn beep(ms: u32) {
    let samples = ms * 44100u32 / 1000u32;
    let samples = samples - samples % SINE.len() as u32;
    unsafe {
        core::ptr::write_volatile(&mut BEEP_COUNTER, samples);
    }
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(1u64.millis());
            unsafe {
                core::ptr::write_volatile(&mut SYSTEM_COUNTER, SYSTEM_COUNTER.wrapping_add(1));
            }
        }
    });
}

#[derive(Default)]
struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(unsafe { core::ptr::read_volatile(&mut SYSTEM_COUNTER) })
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        let mut timer00 = timer_group0.timer0;
        interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();
        timer00.start(1u64.millis());
        timer00.listen();
        critical_section::with(|cs| {
            TIMER00.borrow_ref_mut(cs).replace(timer00);
        });

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        let i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio21, //sda
            io.pins.gpio22, //scl
            400u32.kHz(),
            &clocks,
        );

        let i2c_bus = shared_bus::BusManagerSimple::new(i2c);
        let mut axp = axp192::Axp192::new(i2c_bus.acquire_i2c());
        // core power
        axp.set_dcdc1_voltage(3350).unwrap();
        // LCD power
        axp.set_ldo2_voltage(3300).unwrap();
        axp.set_ldo2_on(true).unwrap();
        // LCD backlight
        axp.set_dcdc3_voltage(3300).unwrap();
        axp.set_dcdc3_on(true).unwrap();
        // LCD reset
        axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)
            .unwrap();
        axp.set_gpio4_output(false).unwrap();
        delay.delay_us(100 * 1000u32);
        axp.set_gpio4_output(true).unwrap();
        // power LED
        axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)
            .unwrap();
        axp.set_gpio1_output(false).unwrap();

        let mut touch = ft6x36::Ft6x36::new(i2c_bus.acquire_i2c(), ft6x36::Dimension(320, 240));
        touch.init().unwrap();

        let spi = Spi::new(
            peripherals.SPI3,
            io.pins.gpio18, //sclk
            io.pins.gpio23, //sdo
            io.pins.gpio38, //sdi
            io.pins.gpio5,  //cs
            60u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        );

        let dc = io.pins.gpio15.into_push_pull_output();
        let rst = DummyPin::new_low(); // connected with AXP192

        let di = SPIInterfaceNoCS::new(spi, dc);
        let display = mipidsi::Builder::ili9342c_rgb565(di)
            .with_display_size(320, 240)
            .with_invert_colors(mipidsi::ColorInversion::Inverted)
            .with_color_order(mipidsi::options::ColorOrder::Bgr)
            .init(&mut delay, Some(rst))
            .unwrap();

        let size = display.size();
        let size = slint::PhysicalSize::new(size.width, size.height);
        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        let mut last_touch = None;
        let button = slint::platform::PointerEventButton::Left;

        // Sound
        let dma = Dma::new(system.dma);
        let dma_channel = dma.i2s0channel;

        let mut tx_descriptors = [0u32; 20 * 3];
        let mut rx_descriptors = [0u32; 8 * 3];

        let i2s = I2s::new(
            peripherals.I2S0,
            NoMclk {},
            Standard::Philips,
            DataFormat::Data16Channel16,
            44100u32.Hz(),
            dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ),
            &clocks,
        );

        let i2s_tx = i2s.i2s_tx.with_pins(PinsBclkWsDout::new(
            io.pins.gpio12,
            io.pins.gpio0,
            io.pins.gpio2,
        ));

        let data =
            unsafe { core::slice::from_raw_parts(&SINE as *const _ as *const u8, SINE.len() * 2) };

        let buffer = dma_buffer();
        let mut transfer = i2s_tx.write_dma_circular(buffer).unwrap();

        let mut idx = 0;
        loop {
            let avail = transfer.available();
            if avail > 0 {
                let mut filler = [0u8; 10000];
                let avail = usize::min(10000, avail);
                let mut beep_counter = unsafe { core::ptr::read_volatile(&mut BEEP_COUNTER) };
                let beeping = beep_counter > 0;
                if beeping {
                    esp_println::println!("avail: {}, beep_counter: {}", avail, beep_counter);
                }
                for bidx in 0..avail {
                    if beep_counter > 0 {
                        beep_counter -= 1;
                        filler[bidx] = data[idx];
                        idx += 1;

                        if idx >= data.len() {
                            idx = 0;
                        }
                    } else {
                        idx = 0;
                        filler[bidx] = 0;
                    }
                }
                if beeping {
                    esp_println::println!("beep_counter: {}", beep_counter);
                    unsafe {
                        core::ptr::write_volatile(&mut BEEP_COUNTER, beep_counter);
                    }
                }
                transfer.push(&filler[0..avail]).unwrap();
            }
            slint::platform::update_timers_and_animations();
            if let Some(window) = self.window.borrow().clone() {
                touch.get_touch_event().ok().and_then(|touch_event| {
                    let time = self.duration_since_start();
                    let event = touch_event
                        .p1
                        .map(|p1| {
                            let position = slint::PhysicalPosition::new(
                                ((p1.x as f32) * size.width as f32 / 319.) as _,
                                (p1.y as f32 * size.height as f32 / 239.) as _,
                            )
                            .to_logical(window.scale_factor());
                            let _ = last_touch.replace(position);
                            match p1.touch_type {
                                ft6x36::TouchType::Press => {
                                    WindowEvent::PointerPressed { position, button }
                                }
                                ft6x36::TouchType::Contact => {
                                    WindowEvent::PointerMoved { position }
                                }
                                ft6x36::TouchType::Release => todo!(),
                                ft6x36::TouchType::Invalid => todo!(),
                            }
                        })
                        .or_else(|| match last_touch {
                            Some(_) => last_touch
                                .take()
                                .map(|position| WindowEvent::PointerReleased { position, button }),
                            None => None,
                        });
                    if !event.is_none() {
                        let e = event.unwrap();
                        let is_pointer_release_event =
                            matches!(e, WindowEvent::PointerReleased { .. });
                        window.dispatch_event(e);
                        // removes hover state on widgets
                        if is_pointer_release_event {
                            window.dispatch_event(WindowEvent::PointerExited);
                        }
                    }
                    touch.process_event(time, touch_event)
                });

                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
        }
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ILI9342CRgb565, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}

fn dma_buffer() -> &'static mut [u8; 32000] {
    static mut BUFFER: [u8; 32000] = [0u8; 32000];
    unsafe { &mut BUFFER }
}
