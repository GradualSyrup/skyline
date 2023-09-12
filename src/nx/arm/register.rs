use std::arch::asm;

macro_rules! reg {
    ( $func_name:ident, $register_name:ident ) => {
            pub fn $func_name() -> u64 {
                let ret: u64;

                unsafe {
                    // changed to match https://github.com/rust-lang/stdarch/pull/1052/files#diff-6df5bb0aa0a043cbfbc1e4a55a283c394fd2d216d9a9dbc709745d11c08f8d01
                    // also used https://github.com/rust-lang/rfcs/blob/master/text/2873-inline-asm.md
                    // asm!(
                    //     concat!("mrs x0, ", stringify!($register_name))
                    //     : "={x0}"(ret)
                    // )

                    // this should work, but maybe try replacing "x0" with {} and then reg if not
                    // first try removing the options and see if that works
                    asm!(concat!("mrs x0,", stringify!($register_name)), out("x0") ret, options(nomem, nostack));
                }

                ret
            }
    };
}


reg!(get_system_tick,       cntpct_el0);
reg!(get_system_tick_freq,  cntfrq_el0);
reg!(get_cache_type,        ctr_el0);
reg!(get_tls,               tpidrro_el0);