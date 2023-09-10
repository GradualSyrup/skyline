use std::mem::size_of;
use std::ptr::null;

// TODO: deal with these imports being missing in rust

// #define __STDC_FORMAT_MACROS
// #include <inttypes.h>
// #include "nn/os.h"
// #include "skyline/inlinehook/And64InlineHook.hpp"
// #include "skyline/utils/cpputils.hpp"

static A64_MAX_INSTRUCTIONS: usize = 5;
static A64_MAX_REFERENCES: usize = (A64_MAX_INSTRUCTIONS * 2);
static A64_NOP: usize = 0xd503201fu;

type Instruction = &'static mut &'static mut u32;

// TODO: may need to make structs/struct fields mutable

#[repr(C)]
pub struct FixInfo {
    bprx: *mut u32,
    bprw: *mut u32,
    ls: u32, // left-shift counts
    ad: u32, // & operand
}

#[repr(C)]
pub struct InstructionsInfo {
    insu: u64,
    ins: i64,
    insp: *const u64, // TODO: what is this - a pointer to a u64? was a void pointer in C++
    fmap: [FixInfo; A64_MAX_REFERENCES], // Fix Map
}

#[repr(C)]
pub struct Context {
    basep: u64,
    endp: u64,
    dat: [InstructionsInfo; A64_MAX_INSTRUCTIONS],
}

impl Context {
    #[inline]
    pub fn is_in_fixing_range(mut self, absolute_addr: u64) -> bool {
        absolute_addr >= self.basep && absolute_addr < self.endp
    }

    #[inline]
    pub fn get_ref_ins_index(mut self, absolute_addr: u64) -> usize {
        ((absolute_addr - self.basep) / sizeof(u64)) as usize
    }

    #[inline]
    pub fn get_and_set_current_index(mut self, inp: &mut u64, outp: &mut u64) -> usize { // was u32s?
        current_idx = self.get_ref_ins_index(*inp);
        this.dat[current_idx].insp = outp;
        current_idx
    }

    #[inline]
    pub fn reset_current_ins(mut self, idx: usize, outp: &mut u64) {
        self.dat[idx].insp = outp;
    }
    
    // Used to have default args uint32_t ls = 0u, uint32_t ad = 0xffffffffu. Neither seemed to ever be used.
    pub fn insert_fix_map(mut self, idx: usize, bprw: *mut u32, bprx: *mut u32, ls: u32, ad: u32) {
        for mut f in self.dat[idx].fmap {
            if f.bprw.is_null() {
                f.bprw = bprw;
                f.bprx = bprx;
                f.ls = ls;
                f.ad = ad;
                return;
            }  // if
        }
        // What? GGing..
    }
    pub unsafe fn process_fix_map(mut self, idx: usize) {
        for mut f in self.dat[idx].fmap {
            if f.bprw.is_null() {
                break;
            }
            *(f.bprw) =
                *(f.bprx) | ((((self.dat[idx].ins - *(f.bprx)) as u32 >> 2) << f.ls) & f.ad);
            f.bprw = null();
            f.bprx = null();
        }
    }
}