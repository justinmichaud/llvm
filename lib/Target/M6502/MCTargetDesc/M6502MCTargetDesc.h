//===-- M6502MCTargetDesc.h - M6502 Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides M6502 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502MCTARGETDESC_H
#define LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502MCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

Target &getTheM6502Target();
Target &getTheM6502elTarget();
Target &getTheM650264Target();
Target &getTheM650264elTarget();

MCCodeEmitter *createM6502MCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);
MCCodeEmitter *createM6502MCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);

MCAsmBackend *createM6502AsmBackend(const Target &T, const MCRegisterInfo &MRI,
                                   const Triple &TT, StringRef CPU,
                                   const MCTargetOptions &Options);

std::unique_ptr<MCObjectWriter>
createM6502ELFObjectWriter(raw_pwrite_stream &OS, const Triple &TT, bool IsN32);

} // End llvm namespace

// Defines symbolic names for M6502 registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "M6502GenRegisterInfo.inc"

// Defines symbolic names for the M6502 instructions.
#define GET_INSTRINFO_ENUM
#include "M6502GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "M6502GenSubtargetInfo.inc"

#endif
