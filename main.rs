#![no_std]
#![no_main]

extern crate alloc;

mod esp32_m5stack_core2;
use esp32_m5stack_core2::*;

slint::include_modules!();

#[entry]
fn main() -> ! {
    init();
    let timer = slint::Timer::default();

    let main_window = MainWindow::new().unwrap();

    let weak = main_window.as_weak();
    main_window.on_reset_clicked(move || {
        let main_window = weak.upgrade().unwrap();
        main_window.set_min(0);
        main_window.set_sec(0);
        let time = slint::format!("0:00");
        main_window.set_time(time);
        beep(100);
    });

    let weak = main_window.as_weak();
    main_window.on_button_clicked(move || {
        beep(100);
        let main_window = weak.upgrade().unwrap();
        let running = main_window.get_running();
        if running {
            main_window.set_running(false);
        } else {
            main_window.set_running(true);
            let weak = main_window.as_weak();
            timer.start(
                slint::TimerMode::Repeated,
                core::time::Duration::from_millis(1000),
                move || {
                    let main_window = weak.upgrade().unwrap();
                    let running = main_window.get_running();
                    if !running {
                        return;
                    }
                    let mut min = main_window.get_min();
                    let mut sec = main_window.get_sec();
                    if sec < 59 {
                        sec += 1;
                    } else {
                        sec = 0;
                        min += 1;
                        if min == 3 {
                            main_window.set_running(false);
                            beep(500);
                        }
                    }
                    main_window.set_min(min);
                    main_window.set_sec(sec);
                    let time = slint::format!("{:01}:{:02}", min, sec);
                    main_window.set_time(time);
                },
            );
        }
    });

    main_window.run().unwrap();

    panic!("The MCU demo should not quit")
}
