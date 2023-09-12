use super::register;
use std::arch::asm;

pub fn flush_data(mut address: usize, size: usize) {
    let mut cache_line_words = register::get_cache_type();

    /* Get bits 16:19, dcache line size in words */
    cache_line_words <<= 16;
    cache_line_words &= 0xF;

    let cache_line_size = 4 << cache_line_words;

    /* Round to nearest cache line. */
    address &= !(cache_line_size - 1);

    let end = address + size;
    while address < end {
        unsafe {
            //switched using https://github.com/MrElectrify/ntapi/commit/0634d28b1dacb9cdad9e3968907926b0672dafea
            //llvm_asm!("dc cvac, $0" :: "x"(address));
            asm!("dc cvac, {0}", in(reg) address);
        }
        address += cache_line_size;
    }

    /* Block until cache is finished flushing. */
    unsafe {
        asm!("dsb sy");
    }
}

pub fn invalidate_instruction(mut address: usize, size: usize) {
    let mut cache_line_words = register::get_cache_type();

    /* Get bits 0:3, icache line size in words */
    cache_line_words &= 0xF;

    let cache_line_size = 4 << cache_line_words;

    /* Round to nearest cache line. */
    address &= !(cache_line_size - 1);

    let end = address + size;
    while address < end  {
        unsafe {
            //llvm_asm!("ic ivau, $0" :: "x"(address));
            asm!("ic ivau, {0}", in(reg) address);
        }
        address += cache_line_size;
    }

    /* Block until cache is finished flushing. */
    unsafe {
        asm!("dsb sy");
    }
}