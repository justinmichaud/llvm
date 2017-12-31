//===-- M6502SERegisterInfo.h - M650232/64 Register Information ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650232/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502SEREGISTERINFO_H
#define LLVM_LIB_TARGET_M6502_M6502SEREGISTERINFO_H

#include "M6502RegisterInfo.h"

namespace llvm {
class M6502SEInstrInfo;

class M6502SERegisterInfo : public M6502RegisterInfo {
public:
  M6502SERegisterInfo();

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;

  const TargetRegisterClass *intRegClass(unsigned Size) const override;

private:
  void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                   int FrameIndex, uint64_t StackSize,
                   int64_t SPOffset) const override;
};

} // end namespace llvm

#endif
