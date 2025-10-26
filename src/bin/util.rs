#![no_std]
#![no_main]

macro_rules! timed_loop {
    ($t:expr,  $b: expr) => {{
        let deadline = time::Instant::now() + time::Duration::from_secs($t);
        let timed_out = 'out: loop {
            if $b {
                break 'out false;
            }
            if time::Instant::now() > deadline {
                break 'out true;
            }
        };
        !timed_out
    }};
    ($t:expr,  $b: block, $delay:expr) => {{
        let deadline = time::Instant::now() + time::Duration::from_secs($t);
        let timed_out = 'out: loop {
            if $b {
                break 'out false;
            }
            if time::Instant::now() > deadline {
                break 'out true;
            }
            Delay::new().delay_millis($delay);
        };
        !timed_out
    }};
}
pub(crate) use timed_loop;
