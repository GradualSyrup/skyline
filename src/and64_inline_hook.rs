use std::mem::size_of;
use std::ptr::{null_mut, copy};

// TODO: deal with these imports being missing in rust

// #define __STDC_FORMAT_MACROS
// #include <inttypes.h>
// #include "nn/os.h"
// #include "skyline/inlinehook/And64InlineHook.hpp"
// #include "skyline/utils/cpputils.hpp"

const A64_MAX_INSTRUCTIONS: usize = 5;
const A64_MAX_REFERENCES: usize = A64_MAX_INSTRUCTIONS * 2;
const A64_NOP: u32 = 0xd503201f;

type Instruction = &'static mut *mut u32;

// TODO: may need to make structs/struct fields mutable

// TODO: The typing is probably getting destroyed here from what it was in C++
//  all of the .add() functions and recasting needs to be double checked when this is testable
//  so if we get down to this point, fix_branch_imm and all the types below need to be rewritten

#[repr(C)]
#[derive(Clone, Copy,)]
pub struct FixInfo {
    bprx: *mut u32,
    bprw: *mut u32,
    ls: u32, // left-shift counts
    ad: u32, // & operand
}

#[repr(C)]
#[derive(Clone, Copy,)]
pub struct InstructionsInfo {
    insu: u64,
    ins: i64,
    insp: *const u64, // TODO: what is this - a pointer to a u64? was a void pointer in C++
    fmap: [FixInfo; A64_MAX_REFERENCES], // Fix Map
}

#[repr(C)]
#[derive(Clone, Copy,)]
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
        (absolute_addr - self.basep) as usize / size_of::<usize>()
    }

    #[inline]
    pub unsafe fn get_and_set_current_index(mut self, inp: *mut u32, outp: *mut u32) -> usize { // was u32s?
        let current_idx = self.get_ref_ins_index(inp as u64);
        self.dat[current_idx].insp = std::mem::transmute::<*mut u32, *const u64>(outp);
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
                *(f.bprx) | ((((self.dat[idx].ins as u32 - *(f.bprx)) >> 2) << f.ls) & f.ad);
            f.bprw = null_mut();
            f.bprx = null_mut();
        }
    }
}

//-------------------------------------------------------------------------

unsafe fn fix_branch_imm(inprwp: Instruction, inprxp: Instruction, outprw: Instruction, outprx: Instruction, ctxp: &mut Context) -> bool {
    let mbits: u32 = 6u32;
    let mask: i64 = 0xfc000000u32 as i64;   // 0b11111100000000000000000000000000
    let rmask: u32 = 0x03ffffffu32;  // 0b00000011111111111111111111111111
    let op_b: u32 = 0x14000000u32;   // "b"  ADDR_PCREL26
    let op_bl: u32 = 0x94000000u32;  // "bl" ADDR_PCREL26
    let ins: u32 = *(*inprwp);
    let opc: i64 = ins as i64 & mask;
    
    if opc == op_b.into() || opc == op_bl.into() {
        let current_idx: usize = ctxp.get_and_set_current_index(*inprxp, *outprx);
        let absolute_addr = (*inprxp).add(((ins << mbits) >> (mbits - 2)) as usize); // sign-extended
        let mut new_pc_offset: i64 = (absolute_addr.offset_from(*outprx)) as i64 >> 2; // shifted
        let special_fix_type: bool = ctxp.is_in_fixing_range(absolute_addr as u64);
        // whether the branch should be converted to absolute jump
        if !special_fix_type && new_pc_offset.abs() >= (rmask >> 1).into() {
            let b_aligned: bool = (((*outprx).add(2)) as u64 & 7) == 0;
            if opc == op_b.into() {
                if !b_aligned {
                    *(*outprw).offset(0) = A64_NOP;
                    *outprx = (*outprx).add(1);
                    ctxp.reset_current_ins(current_idx,&mut (*(*outprx) as u64));
                    *outprw= (*outprw).add(1);
                }                              // if
                *(*outprw).offset(0) = 0x58000051u32;  // LDR X17, #0x8
                *(*outprw).offset(1) = 0xd61f0220u32;  // BR X17
                //memcpy((*outprw).add(2), &absolute_addr, size_of::<*mut u32>());
                copy(absolute_addr, (*outprw).add(2), size_of::<*mut u32>());
                *outprx = (*outprx).add(4);
                *outprw= (*outprw).add(4);
            } else {
                if b_aligned {
                    *(*outprw).offset(0) = A64_NOP;
                    *outprx = (*outprx).add(1);
                    ctxp.reset_current_ins(current_idx, &mut (*(*outprx) as u64));
                    *outprw = (*outprw).add(1);
                }                              // if
                *(*outprw).offset(0) = 0x58000071u32;  // LDR X17, #12
                *(*outprw).offset(1) = 0x1000009eu32;  // ADR X30, #16
                *(*outprw).offset(2) = 0xd61f0220u32;  // BR X17
                //memcpy((*outprw).add(3), &absolute_addr, size_of::<*mut u32>());
                copy(absolute_addr, (*outprw).add(3), size_of::<*mut u32>());
                *outprw = (*outprw).add(5);
                *outprx = (*outprx).add(5);
            }  // if
        } else {
            if special_fix_type {
                let ref_idx: usize = ctxp.get_ref_ins_index(absolute_addr as u64);
                if ref_idx <= current_idx {
                    new_pc_offset = (ctxp.dat[ref_idx].ins - (*outprx) as i64) >> 2;
                } else {
                    ctxp.insert_fix_map(ref_idx, *outprw, *outprx, 0u32, rmask);
                    new_pc_offset = 0;
                }  // if
            }      // if

            *(*outprw).offset(0) = (opc | (new_pc_offset & -mask)) as u32;
            *outprw = (*outprw).add(1);
            *outprx = (*outprx).add(1);
        }  // if

        *inprxp= (*inprxp).add(1);
        *inprwp= (*inprwp).add(1);
        ctxp.process_fix_map(current_idx);
        return true;
    }
    false
}