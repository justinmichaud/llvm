//===-- M6502MCAsmInfo.h - M6502 Asm Info ------------------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the M6502MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502MCASMINFO_H
#define LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502MCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class M6502MCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit M6502MCAsmInfo(const Triple &TheTriple);
};

} // namespace llvm

#endif
