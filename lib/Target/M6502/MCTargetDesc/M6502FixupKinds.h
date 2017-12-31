//===-- M6502FixupKinds.h - M6502 Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502FIXUPKINDS_H
#define LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502FIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace M6502 {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the same order of
  // MCFixupKindInfo Infos[M6502::NumTargetFixupKinds]
  // in M6502AsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_M6502_NONE.
    fixup_M6502_NONE = FirstTargetFixupKind,

    // Branch fixups resulting in R_M6502_16.
    fixup_M6502_16,

    // Pure 32 bit data fixup resulting in - R_M6502_32.
    fixup_M6502_32,

    // Full 32 bit data relative data fixup resulting in - R_M6502_REL32.
    fixup_M6502_REL32,

    // Jump 26 bit fixup resulting in - R_M6502_26.
    fixup_M6502_26,

    // Pure upper 16 bit fixup resulting in - R_M6502_HI16.
    fixup_M6502_HI16,

    // Pure lower 16 bit fixup resulting in - R_M6502_LO16.
    fixup_M6502_LO16,

    // 16 bit fixup for GP offest resulting in - R_M6502_GPREL16.
    fixup_M6502_GPREL16,

    // 16 bit literal fixup resulting in - R_M6502_LITERAL.
    fixup_M6502_LITERAL,

    // Symbol fixup resulting in - R_M6502_GOT16.
    fixup_M6502_GOT,

    // PC relative branch fixup resulting in - R_M6502_PC16.
    fixup_M6502_PC16,

    // resulting in - R_M6502_CALL16.
    fixup_M6502_CALL16,

    // resulting in - R_M6502_GPREL32.
    fixup_M6502_GPREL32,

    // resulting in - R_M6502_SHIFT5.
    fixup_M6502_SHIFT5,

    // resulting in - R_M6502_SHIFT6.
    fixup_M6502_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_M6502_64.
    fixup_M6502_64,

    // resulting in - R_M6502_TLS_GD.
    fixup_M6502_TLSGD,

    // resulting in - R_M6502_TLS_GOTTPREL.
    fixup_M6502_GOTTPREL,

    // resulting in - R_M6502_TLS_TPREL_HI16.
    fixup_M6502_TPREL_HI,

    // resulting in - R_M6502_TLS_TPREL_LO16.
    fixup_M6502_TPREL_LO,

    // resulting in - R_M6502_TLS_LDM.
    fixup_M6502_TLSLDM,

    // resulting in - R_M6502_TLS_DTPREL_HI16.
    fixup_M6502_DTPREL_HI,

    // resulting in - R_M6502_TLS_DTPREL_LO16.
    fixup_M6502_DTPREL_LO,

    // PC relative branch fixup resulting in - R_M6502_PC16
    fixup_M6502_Branch_PCRel,

    // resulting in - R_M6502_GPREL16/R_M6502_SUB/R_M6502_HI16
    fixup_M6502_GPOFF_HI,

    // resulting in - R_M6502_GPREL16/R_M6502_SUB/R_M6502_LO16
    fixup_M6502_GPOFF_LO,

    // resulting in - R_M6502_PAGE
    fixup_M6502_GOT_PAGE,

    // resulting in - R_M6502_GOT_OFST
    fixup_M6502_GOT_OFST,

    // resulting in - R_M6502_GOT_DISP
    fixup_M6502_GOT_DISP,

    // resulting in - R_M6502_GOT_HIGHER
    fixup_M6502_HIGHER,

    // resulting in - R_M6502_HIGHEST
    fixup_M6502_HIGHEST,

    // resulting in - R_M6502_GOT_HI16
    fixup_M6502_GOT_HI16,

    // resulting in - R_M6502_GOT_LO16
    fixup_M6502_GOT_LO16,

    // resulting in - R_M6502_CALL_HI16
    fixup_M6502_CALL_HI16,

    // resulting in - R_M6502_CALL_LO16
    fixup_M6502_CALL_LO16,

    // resulting in - R_M6502_PC18_S3
    fixup_M6502_PC18_S3,

    // resulting in - R_M6502_PC19_S2
    fixup_M6502_PC19_S2,

    // resulting in - R_M6502_PC21_S2
    fixup_M6502_PC21_S2,

    // resulting in - R_M6502_PC26_S2
    fixup_M6502_PC26_S2,

    // resulting in - R_M6502_PCHI16
    fixup_M6502_PCHI16,

    // resulting in - R_M6502_PCLO16
    fixup_M6502_PCLO16,

    // resulting in - R_MICROM6502_26_S1
    fixup_MICROM6502_26_S1,

    // resulting in - R_MICROM6502_HI16
    fixup_MICROM6502_HI16,

    // resulting in - R_MICROM6502_LO16
    fixup_MICROM6502_LO16,

    // resulting in - R_MICROM6502_GOT16
    fixup_MICROM6502_GOT16,

    // resulting in - R_MICROM6502_PC7_S1
    fixup_MICROM6502_PC7_S1,

    // resulting in - R_MICROM6502_PC10_S1
    fixup_MICROM6502_PC10_S1,

    // resulting in - R_MICROM6502_PC16_S1
    fixup_MICROM6502_PC16_S1,

    // resulting in - R_MICROM6502_PC26_S1
    fixup_MICROM6502_PC26_S1,

    // resulting in - R_MICROM6502_PC19_S2
    fixup_MICROM6502_PC19_S2,

    // resulting in - R_MICROM6502_PC18_S3
    fixup_MICROM6502_PC18_S3,

    // resulting in - R_MICROM6502_PC21_S1
    fixup_MICROM6502_PC21_S1,

    // resulting in - R_MICROM6502_CALL16
    fixup_MICROM6502_CALL16,

    // resulting in - R_MICROM6502_GOT_DISP
    fixup_MICROM6502_GOT_DISP,

    // resulting in - R_MICROM6502_GOT_PAGE
    fixup_MICROM6502_GOT_PAGE,

    // resulting in - R_MICROM6502_GOT_OFST
    fixup_MICROM6502_GOT_OFST,

    // resulting in - R_MICROM6502_TLS_GD
    fixup_MICROM6502_TLS_GD,

    // resulting in - R_MICROM6502_TLS_LDM
    fixup_MICROM6502_TLS_LDM,

    // resulting in - R_MICROM6502_TLS_DTPREL_HI16
    fixup_MICROM6502_TLS_DTPREL_HI16,

    // resulting in - R_MICROM6502_TLS_DTPREL_LO16
    fixup_MICROM6502_TLS_DTPREL_LO16,

    // resulting in - R_MICROM6502_TLS_GOTTPREL.
    fixup_MICROM6502_GOTTPREL,

    // resulting in - R_MICROM6502_TLS_TPREL_HI16
    fixup_MICROM6502_TLS_TPREL_HI16,

    // resulting in - R_MICROM6502_TLS_TPREL_LO16
    fixup_MICROM6502_TLS_TPREL_LO16,

    // resulting in - R_M6502_SUB/R_MICROM6502_SUB
    fixup_M6502_SUB,
    fixup_MICROM6502_SUB,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace M6502
} // namespace llvm


#endif
